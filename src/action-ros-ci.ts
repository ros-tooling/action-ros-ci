import * as core from "@actions/core";
import * as github from "@actions/github";
import * as im from "@actions/exec/lib/interfaces"; // eslint-disable-line no-unused-vars
import * as tr from "@actions/exec/lib/toolrunner";
import * as io from "@actions/io";
import * as os from "os";
import * as path from "path";
import fs from "fs";
import retry from "async-retry";
import * as dep from "./dependencies";

// All command line flags passed to curl when invoked as a command.
const curlFlagsArray = [
	// (HTTP)  Fail  silently  (no  output at all) on server errors. This is mostly done to better enable
	// scripts etc to better deal with failed attempts. In normal cases  when  a  HTTP  server  fails  to
	// deliver  a  document,  it  returns an HTML document stating so (which often also describes why and
	// more). This flag will prevent curl from outputting that and return error 22.
	// This method is not fail-safe and there are occasions where non-successful response codes will slip
	// through, especially when authentication is involved (response codes 401 and 407).
	"--fail",

	// Silent or quiet mode. Don't show progress meter or error messages.  Makes Curl mute.
	"--silent",

	// When used with -s it makes curl show an error message if it fails.
	"--show-error",

	// (HTTP/HTTPS) If the server reports that the requested page  has  moved  to  a  different  location
	// (indicated  with  a Location: header and a 3XX response code), this option will make curl redo the
	// request on the new place. If used together with -i, --include or  -I,  --head,  headers  from  all
	// requested pages will be shown. When authentication is used, curl only sends its credentials to the
	// initial host. If a redirect takes curl to a different host, it won't  be  able  to  intercept  the
	// user+password.  See  also  --location-trusted  on  how to change this. You can limit the amount of
	// redirects to follow by using the --max-redirs option.
	//
	// When curl follows a redirect and the request is not a plain GET (for example POST or PUT), it will
	// do  the  following  request  with a GET if the HTTP response was 301, 302, or 303. If the response
	// code was any other 3xx code, curl will re-send the following request  using  the  same  unmodified
	// method.
	"--location",
];

const validROS1Distros: string[] = ["kinetic", "lunar", "melodic", "noetic"];
const validROS2Distros: string[] = ["dashing", "eloquent", "foxy", "rolling"];
const targetROS1DistroInput: string = "target-ros1-distro";
const targetROS2DistroInput: string = "target-ros2-distro";
const isLinux: boolean = process.platform == "linux";
const isWindows: boolean = process.platform == "win32";

/**
 * Convert local paths to URLs.
 *
 * The user can pass the VCS repo file either as a URL or a path.
 * If it is a path, this function will convert it into a URL (file://...).
 * If the file is already passed as an URL, this function does nothing.
 *
 * @param   vcsRepoFileUrl     path or URL to the repo file
 * @returns                    URL to the repo file
 */
function resolveVcsRepoFileUrl(vcsRepoFileUrl: string): string {
	if (fs.existsSync(vcsRepoFileUrl)) {
		return "file://" + path.resolve(vcsRepoFileUrl);
	} else {
		return vcsRepoFileUrl;
	}
}

/**
 * Execute a command in bash and wrap the output in a log group.
 *
 * @param   commandLine     command to execute (can include additional args). Must be correctly escaped.
 * @param   commandPrefix    optional string used to prefix the command to be executed.
 * @param   options         optional exec options.  See ExecOptions
 * @param   log_message     log group title.
 * @returns Promise<number> exit code
 */
export async function execBashCommand(
	commandLine: string,
	commandPrefix?: string,
	options?: im.ExecOptions,
	log_message?: string
): Promise<number> {
	commandPrefix = commandPrefix || "";
	const bashScript = `${commandPrefix}${commandLine}`;
	const message = log_message || `Invoking: bash -c '${bashScript}'`;

	let toolRunnerCommandLine = "";
	let toolRunnerCommandLineArgs: string[] = [];
	if (isWindows) {
		toolRunnerCommandLine = "C:\\Windows\\system32\\cmd.exe";
		// This passes the same flags to cmd.exe that "run:" in a workflow.
		// https://help.github.com/en/actions/automating-your-workflow-with-github-actions/workflow-syntax-for-github-actions#using-a-specific-shell
		// Except for /D, which disables the AutoRun functionality from command prompt
		// and it blocks Python virtual environment activation if one configures it in
		// the previous steps.
		toolRunnerCommandLineArgs = [
			"/E:ON",
			"/V:OFF",
			"/S",
			"/C",
			"call",
			"%programfiles(x86)%\\Microsoft Visual Studio\\2019\\Enterprise\\VC\\Auxiliary\\Build\\vcvarsall.bat",
			"amd64",
			"&",
			"C:\\Program Files\\Git\\bin\\bash.exe",
			"-c",
			bashScript,
		];
	} else {
		toolRunnerCommandLine = "bash";
		toolRunnerCommandLineArgs = ["-c", bashScript];
	}
	const runner: tr.ToolRunner = new tr.ToolRunner(
		toolRunnerCommandLine,
		toolRunnerCommandLineArgs,
		options
	);
	return core.group(message, () => {
		return runner.exec();
	});
}

//Determine whether all inputs name supported ROS distributions.
export function validateDistros(
	ros1Distro: string,
	ros2Distro: string
): boolean {
	if (!ros1Distro && !ros2Distro) {
		core.setFailed(
			`Neither '${targetROS1DistroInput}' or '${targetROS2DistroInput}' inputs were set, at least one is required.`
		);
		return false;
	}
	if (ros1Distro && validROS1Distros.indexOf(ros1Distro) <= -1) {
		core.setFailed(
			`Input ${ros1Distro} was not a valid ROS 1 distribution for '${targetROS1DistroInput}'. Valid values: ${validROS1Distros}`
		);
		return false;
	}
	if (ros2Distro && validROS2Distros.indexOf(ros2Distro) <= -1) {
		core.setFailed(
			`Input ${ros2Distro} was not a valid ROS 2 distribution for '${targetROS2DistroInput}'. Valid values: ${validROS2Distros}`
		);
		return false;
	}
	return true;
}

/**
 * Install ROS dependencies for given packages in the workspace, for all ROS distros being used.
 */
async function installRosdeps(
	upToPackages: string,
	workspaceDir: string,
	ros1Distro?: string,
	ros2Distro?: string
): Promise<number> {
	const scriptName = "install_rosdeps.sh";
	const scriptPath = path.join(workspaceDir, scriptName);
	const scriptContent = `#!/bin/bash
	set -euxo pipefail
	if [ $# != 1 ]; then
		echo "Specify rosdistro name as single argument to this script"
		exit 1
	fi
	DISTRO=$1
	package_paths=$(colcon list --paths-only --packages-up-to ${upToPackages})
	# suppress errors from unresolved install keys to preserve backwards compatibility
	# due to difficulty reading names of some non-catkin dependencies in the ros2 core
	# see https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/#install-dependencies-using-rosdep
	rosdep install -r --from-paths $package_paths --ignore-src --skip-keys rti-connext-dds-5.3.1 --rosdistro $DISTRO -y || true`;
	fs.writeFileSync(scriptPath, scriptContent, { mode: 0o766 });

	let exitCode = 0;
	const options = { cwd: workspaceDir };
	if (ros1Distro) {
		exitCode += await execBashCommand(
			`./${scriptName} ${ros1Distro}`,
			"",
			options
		);
	}
	if (ros2Distro) {
		exitCode += await execBashCommand(
			`./${scriptName} ${ros2Distro}`,
			"",
			options
		);
	}
	return exitCode;
}

async function run() {
	try {
		const repo = github.context.repo;
		const workspace = process.env.GITHUB_WORKSPACE as string;

		const colconMixinName = core.getInput("colcon-mixin-name");
		const colconMixinRepo = core.getInput("colcon-mixin-repository");
		const extraCmakeArgs = core.getInput("extra-cmake-args");
		const colconExtraArgs = core.getInput("colcon-extra-args");
		const importToken = core.getInput("import-token");
		const packageNames = core
			.getInput("package-name", { required: true })
			.split(RegExp("\\s"))
			.join(" ");
		const rosWorkspaceName = "ros_ws";
		core.setOutput("ros-workspace-directory-name", rosWorkspaceName);
		const rosWorkspaceDir = path.join(workspace, rosWorkspaceName);
		const targetRos1Distro = core.getInput(targetROS1DistroInput);
		const targetRos2Distro = core.getInput(targetROS2DistroInput);
		const vcsRepoFileUrlListAsString = core.getInput("vcs-repo-file-url") || "";
		let vcsRepoFileUrlList = vcsRepoFileUrlListAsString.split(RegExp("\\s"));

		// Check if PR overrides/adds supplemental repos files
		const vcsReposOverride = dep.getReposFilesOverride(github.context.payload);
		const vcsReposSupplemental = dep.getReposFilesSupplemental(
			github.context.payload
		);
		await core.group(
			"Repos files: override" +
				(vcsReposOverride.length === 0 ? " - none" : ""),
			() => {
				for (const vcsRepos of vcsReposOverride) {
					core.info("\t" + vcsRepos);
				}
				return Promise.resolve();
			}
		);
		if (vcsReposOverride.length > 0) {
			vcsRepoFileUrlList = vcsReposOverride;
		}
		await core.group(
			"Repos files: supplemental" +
				(vcsReposSupplemental.length === 0 ? " - none" : ""),
			() => {
				for (const vcsRepos of vcsReposSupplemental) {
					core.info("\t" + vcsRepos);
				}
				return Promise.resolve();
			}
		);
		vcsRepoFileUrlList = vcsRepoFileUrlList.concat(vcsReposSupplemental);

		const vcsRepoFileUrlListNonEmpty = vcsRepoFileUrlList.filter(
			(x) => x != ""
		);
		const vcsRepoFileUrlListResolved = vcsRepoFileUrlListNonEmpty.map((x) =>
			resolveVcsRepoFileUrl(x)
		);

		const coverageIgnorePattern = core.getInput("coverage-ignore-pattern");

		if (!validateDistros(targetRos1Distro, targetRos2Distro)) {
			return;
		}

		// rosdep does not reliably work on Windows, see
		// ros-infrastructure/rosdep#610 for instance. So, we do not run it.
		if (!isWindows) {
			await retry(
				async () => {
					await execBashCommand("rosdep update --include-eol-distros");
				},
				{
					retries: 3,
				}
			);
		}

		// Reset colcon configuration.
		await io.rmRF(path.join(os.homedir(), ".colcon"));

		// Wipe out the workspace directory to ensure the workspace is always
		// identical.
		await io.rmRF(rosWorkspaceDir);

		// Checkout ROS 2 from source and install ROS 2 system dependencies
		await io.mkdirP(rosWorkspaceDir + "/src");

		if (importToken !== "") {
			const config = `
[url "https://${importToken}@github.com"]
	insteadOf = https://github.com
[url "http://${importToken}@github.com"]
	insteadOf = http://github.com`;
			fs.appendFileSync(path.join(os.homedir(), ".gitconfig"), config);
		}

		const options = {
			cwd: rosWorkspaceDir,
		};

		const curlFlags = curlFlagsArray.join(" ");
		for (const vcsRepoFileUrl of vcsRepoFileUrlListResolved) {
			await execBashCommand(
				`curl ${curlFlags} '${vcsRepoFileUrl}' | vcs import --force --recursive src/`,
				undefined,
				options
			);
		}

		// If the package under tests is part of ros.repos, remove it first.
		// We do not want to allow the "default" head state of the package to
		// to be present in the workspace, and colcon will fail stating it found twice
		// a package with an identical name.
		const posixRosWorkspaceDir = isWindows
			? rosWorkspaceDir.replace(/\\/g, "/")
			: rosWorkspaceDir;
		await execBashCommand(
			`vcs diff -s --repos ${posixRosWorkspaceDir} | cut -d ' ' -f 1 | grep "${repo["repo"]}$" | xargs rm -rf`
		);

		// The repo file for the repository needs to be generated on-the-fly to
		// incorporate the custom repository URL and branch name, when a PR is
		// being built.
		let repoFullName = process.env.GITHUB_REPOSITORY as string;
		if (github.context.payload.pull_request) {
			repoFullName = github.context.payload.pull_request.head.repo.full_name;
		}
		const headRef = process.env.GITHUB_HEAD_REF as string;
		const commitRef = headRef || github.context.sha;
		const repoFilePath = path.join(rosWorkspaceDir, "package.repo");
		// Add a random string prefix to avoid naming collisions when checking out the test repository
		const randomStringPrefix = Math.random().toString(36).substring(2, 15);
		const repoFileContent = `repositories:
  ${randomStringPrefix}/${repo["repo"]}:
    type: git
    url: 'https://github.com/${repoFullName}.git'
    version: '${commitRef}'`;
		fs.writeFileSync(repoFilePath, repoFileContent);
		await execBashCommand(
			"vcs import --force --recursive src/ < package.repo",
			undefined,
			options
		);

		// Print HEAD commits of all repos
		await execBashCommand("vcs log -l1 src/", undefined, options);

		await installRosdeps(
			packageNames,
			rosWorkspaceDir,
			targetRos1Distro,
			targetRos2Distro
		);

		if (colconMixinName !== "" && colconMixinRepo !== "") {
			await execBashCommand(`colcon mixin add default '${colconMixinRepo}'`);
			await execBashCommand("colcon mixin update default");
		}

		let extra_options: string[] = [];
		if (colconMixinName !== "") {
			extra_options = extra_options.concat(["--mixin", colconMixinName]);
		}
		if (colconExtraArgs !== "") {
			extra_options = extra_options.concat(colconExtraArgs);
		}

		// Add the future install bin directory to PATH.
		// This enables cmake find_package to find packages installed in the
		// colcon install directory, even if local_setup.sh has not been sourced.
		//
		// From the find_package doc:
		// https://cmake.org/cmake/help/latest/command/find_package.html
		//   5. Search the standard system environment variables.
		//   Path entries ending in /bin or /sbin are automatically converted to
		//   their parent directories:
		//   PATH
		//
		// ament_cmake should handle this automatically, but we are seeing cases
		// where this does not happen. See issue #26 for relevant CI logs.
		core.addPath(path.join(rosWorkspaceDir, "install", "bin"));

		// Source any installed ROS distributions if they are present
		let colconCommandPrefix = "";
		if (isLinux) {
			if (targetRos1Distro) {
				const ros1SetupPath = `/opt/ros/${targetRos1Distro}/setup.sh`;
				if (fs.existsSync(ros1SetupPath)) {
					colconCommandPrefix += `source ${ros1SetupPath} && `;
				}
			}
			if (targetRos2Distro) {
				const ros2SetupPath = `/opt/ros/${targetRos2Distro}/setup.sh`;
				if (fs.existsSync(ros2SetupPath)) {
					colconCommandPrefix += `source ${ros2SetupPath} && `;
				}
			}
		} else if (isWindows) {
			// Windows only supports ROS2
			if (targetRos2Distro) {
				const ros2SetupPath = `c:/dev/${targetRos2Distro}/ros2-windows/setup.bat`;
				if (fs.existsSync(ros2SetupPath)) {
					colconCommandPrefix += `${ros2SetupPath} && `;
				}
			}
		}

		let colconBuildCmd = [
			`colcon build`,
			`--event-handlers console_cohesion+`,
			`--packages-up-to ${packageNames}`,
			`${extra_options.join(" ")}`,
			`--cmake-args ${extraCmakeArgs}`,
		].join(" ");
		if (!isWindows) {
			colconBuildCmd = colconBuildCmd.concat(" --symlink-install");
		}
		await execBashCommand(colconBuildCmd, colconCommandPrefix, options);

		// ignoreReturnCode is set to true to avoid having a lack of coverage
		// data fail the build.
		const colconLcovInitialCmd = "colcon lcov-result --initial";
		await execBashCommand(colconLcovInitialCmd, colconCommandPrefix, {
			cwd: rosWorkspaceDir,
			ignoreReturnCode: true,
		});

		const colconTestCmd = [
			`colcon test`,
			`--event-handlers console_cohesion+`,
			`--pytest-with-coverage`,
			`--return-code-on-test-failure`,
			`--packages-select ${packageNames}`,
			`${extra_options.join(" ")}`,
		].join(" ");
		await execBashCommand(colconTestCmd, colconCommandPrefix, options);

		// ignoreReturnCode, check comment above in --initial
		const colconLcovResultCmd = [
			`colcon lcov-result`,
			`--filter ${coverageIgnorePattern}`,
			`--packages-select ${packageNames}`,
		].join(" ");
		await execBashCommand(colconLcovResultCmd, colconCommandPrefix, {
			cwd: rosWorkspaceDir,
			ignoreReturnCode: true,
		});

		const colconCoveragepyResultCmd = [
			`colcon coveragepy-result`,
			`--packages-select ${packageNames}`,
		].join(" ");
		await execBashCommand(
			colconCoveragepyResultCmd,
			colconCommandPrefix,
			options
		);

		core.setOutput("ros-workspace-directory-name", rosWorkspaceName);
	} catch (error) {
		core.setFailed(error.message);
	}
}

run();
