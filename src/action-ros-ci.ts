import * as core from "@actions/core";
import * as github from "@actions/github";
import * as im from "@actions/exec/lib/interfaces"; // eslint-disable-line no-unused-vars
import * as tr from "@actions/exec/lib/toolrunner";
import * as io from "@actions/io";
import * as os from "os";
import * as path from "path";
import fs from "fs";

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
	"--location"
];

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
	const message = log_message || `Invoking "bash -c '${bashScript}'`;

	let toolRunnerCommandLine = "";
	let toolRunnerCommandLineArgs: string[] = [];
	if (process.platform == "win32") {
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
			bashScript
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

async function run() {
	try {
		const repo = github.context.repo;
		const workspace = process.env.GITHUB_WORKSPACE as string;

		const colconMixinName = core.getInput("colcon-mixin-name");
		const colconMixinRepo = core.getInput("colcon-mixin-repository");
		const extraCmakeArgs = core.getInput("extra-cmake-args");
		const packageName = core.getInput("package-name", { required: true });
		const packageNameList = packageName.split(RegExp("\\s"));
		const rosWorkspaceName = "ros_ws";
		const rosWorkspaceDir = path.join(workspace, rosWorkspaceName);
		const sourceRosBinaryInstallation = core.getInput(
			"source-ros-binary-installation"
		);
		const sourceRosBinaryInstallationList = sourceRosBinaryInstallation
			? sourceRosBinaryInstallation.split(RegExp("\\s"))
			: [];

		const vcsRepoFileUrlListAsString = core.getInput("vcs-repo-file-url");
		const vcsRepoFileUrlList = vcsRepoFileUrlListAsString.split(RegExp("\\s"));
		const vcsRepoFileUrlListNonEmpty = vcsRepoFileUrlList.filter(x => x != "");
		const vcsRepoFileUrlListResolved = vcsRepoFileUrlListNonEmpty.map(x =>
			resolveVcsRepoFileUrl(x)
		);

		const coverageIgnorePattern = core.getInput("coverage-ignore-pattern");

		let commandPrefix = "";
		if (sourceRosBinaryInstallation) {
			if (process.platform !== "linux") {
				core.setFailed(
					"sourcing binary installation is only available on Linux"
				);
				return;
			}
			for (let rosDistribution of sourceRosBinaryInstallationList) {
				commandPrefix += `source /opt/ros/${rosDistribution}/setup.sh && `;
			}
		}

		// rosdep on Windows does not reliably work on Windows, see
		// ros-infrastructure/rosdep#610 for instance. So, we do not run it.
		if (process.platform != "win32") {
			await execBashCommand("rosdep update", commandPrefix);
		}

		// Reset colcon configuration.
		await io.rmRF(path.join(os.homedir(), ".colcon"));

		// Wipe out the workspace directory to ensure the workspace is always
		// identical.
		await io.rmRF(rosWorkspaceDir);

		// Checkout ROS 2 from source and install ROS 2 system dependencies
		await io.mkdirP(rosWorkspaceDir + "/src");

		const options = {
			cwd: rosWorkspaceDir
		};

		const curlFlags = curlFlagsArray.join(" ");
		for (let vcsRepoFileUrl of vcsRepoFileUrlListResolved) {
			await execBashCommand(
				`curl ${curlFlags} '${vcsRepoFileUrl}' | vcs import --force src/`,
				commandPrefix,
				options
			);
		}

		// If the package under tests is part of ros.repos, remove it first.
		// We do not want to allow the "default" head state of the package to
		// to be present in the workspace, and colcon will fail stating it found twice
		// a package with an identical name.
		const posixRosWorkspaceDir =
			process.platform === "win32"
				? rosWorkspaceDir.replace(/\\/g, "/")
				: rosWorkspaceDir;
		await execBashCommand(
			`find "${posixRosWorkspaceDir}" -type d -and -name "${repo["repo"]}" | xargs rm -rf`,
			commandPrefix
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
		const repoFileContent = `repositories:
  ${repo["repo"]}:
    type: git
    url: 'https://github.com/${repoFullName}.git'
    version: '${commitRef}'`;
		fs.writeFileSync(repoFilePath, repoFileContent);
		await execBashCommand(
			"vcs import --force --recursive src/ < package.repo",
			commandPrefix,
			options
		);

		// Remove all repositories the package under test does not depend on, to
		// avoid having rosdep installing unrequired dependencies.
		await execBashCommand(
			`diff --new-line-format="" --unchanged-line-format="" <(colcon list -p) <(colcon list --packages-up-to ${packageNameList.join(
				" "
			)} -p) | xargs rm -rf`,
			commandPrefix,
			options
		);

		// Install ROS dependencies for each distribution being sourced
		for (let rosDistribution of sourceRosBinaryInstallationList) {
			// For "latest" builds, rosdep often misses some keys, adding "|| true", to
			// ignore those failures, as it is often non-critical.
			await execBashCommand(
				`DEBIAN_FRONTEND=noninteractive RTI_NC_LICENSE_ACCEPTED=yes rosdep install -r --from-paths src --ignore-src --rosdistro ${rosDistribution} -y || true`,
				commandPrefix,
				options
			);
		}

		// If no distribution is being sourced, then install dependencies for the latest release
		if (!sourceRosBinaryInstallation) {
			// For "latest" builds, rosdep often misses some keys, adding "|| true", to
			// ignore those failures, as it is often non-critical.
			await execBashCommand(
				`DEBIAN_FRONTEND=noninteractive RTI_NC_LICENSE_ACCEPTED=yes rosdep install -r --from-paths src --ignore-src --rosdistro eloquent -y || true`,
				commandPrefix,
				options
			);
		}

		if (colconMixinName !== "" && colconMixinRepo !== "") {
			await execBashCommand(
				`colcon mixin add default '${colconMixinRepo}'`,
				commandPrefix
			);
			await execBashCommand("colcon mixin update default", commandPrefix);
		}

		let extra_options: string[] = [];
		if (colconMixinName !== "") {
			extra_options = extra_options.concat(["--mixin", colconMixinName]);
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

		let colconBuildCmd = `colcon build --event-handlers console_cohesion+ --symlink-install \
			--packages-up-to ${packageNameList.join(" ")} \
			${extra_options.join(" ")} \
			--cmake-args ${extraCmakeArgs}`;
		await execBashCommand(colconBuildCmd, commandPrefix, options);

		// ignoreReturnCode is set to true to avoid having a lack of coverage
		// data fail the build.
		const colconLcovInitialCmd = "colcon lcov-result --initial";
		await execBashCommand(colconLcovInitialCmd, commandPrefix, {
			cwd: rosWorkspaceDir,
			ignoreReturnCode: true
		});

		const colconTestCmd = `colcon test --event-handlers console_cohesion+ \
			--pytest-args --cov=. --cov-report=xml --return-code-on-test-failure \
			--packages-select ${packageNameList.join(" ")} \
			${extra_options.join(" ")}`;
		await execBashCommand(colconTestCmd, commandPrefix, options);

		// ignoreReturnCode, check comment above in --initial
		const colconLcovResultCmd = `colcon lcov-result \
	             --filter ${coverageIgnorePattern} \
	             --packages-select ${packageNameList.join(" ")}`;
		await execBashCommand(colconLcovResultCmd, commandPrefix, {
			cwd: rosWorkspaceDir,
			ignoreReturnCode: true
		});

		core.setOutput("ros-workspace-directory-name", rosWorkspaceName);
	} catch (error) {
		core.setFailed(error.message);
	}
}

run();
