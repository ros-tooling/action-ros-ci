import * as core from "@actions/core";
import * as github from "@actions/github";
import * as im from "@actions/exec/lib/interfaces"; // eslint-disable-line no-unused-vars
import * as tr from "@actions/exec/lib/toolrunner";
import * as io from "@actions/io";
import * as os from "os";
import * as path from "path";
import fs from "fs";

const validROS1Distros: string[] = ["kinetic", "lunar", "melodic", "noetic"];
const validROS2Distros: string[] = ["dashing", "eloquent", "foxy", "rolling"];
const targetROS1DistroInput: string = "target-ros1-distro";
const targetROS2DistroInput: string = "target-ros2-distro";
const isLinux: boolean = process.platform == "linux";
const isWindows: boolean = process.platform == "win32";

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

async function run() {
	try {
		const repo = github.context.repo;
		const workspace = process.env.GITHUB_WORKSPACE as string;

		const colconMixinName = core.getInput("colcon-mixin-name");
		const colconMixinRepo = core.getInput("colcon-mixin-repository");
		const extraCmakeArgs = core.getInput("extra-cmake-args");
		const colconExtraArgs = core.getInput("colcon-extra-args");
		const importToken = core.getInput("import-token");
		const packageName = core.getInput("package-name", { required: true });
		const packageNameList = packageName.split(RegExp("\\s"));
		const rosWorkspaceName = "ros_ws";
		core.setOutput("ros-workspace-directory-name", rosWorkspaceName);
		const rosWorkspaceDir = path.join(workspace, rosWorkspaceName);
		const targetRos1Distro = core.getInput(targetROS1DistroInput);
		const targetRos2Distro = core.getInput(targetROS2DistroInput);
		const vcsRepoFileUrlListAsString = core.getInput("vcs-repo-file-url") || "";
		const vcsRepoFileUrlList = vcsRepoFileUrlListAsString.split(RegExp("\\s"));
		const vcsRepoFileUrlListNonEmpty = vcsRepoFileUrlList.filter(
			(x) => x != ""
		);

		const coverageIgnorePattern = core.getInput("coverage-ignore-pattern");

		if (!validateDistros(targetRos1Distro, targetRos2Distro)) {
			return;
		}

		// rosdep does not reliably work on Windows, see
		// ros-infrastructure/rosdep#610 for instance. So, we do not run it.
		if (!isWindows) {
			await execBashCommand("rosdep update");
		}

		// Reset colcon configuration.
		await io.rmRF(path.join(os.homedir(), ".colcon"));

		// Wipe out the workspace directory to ensure the workspace is always
		// identical.
		await io.rmRF(rosWorkspaceDir);

		// Checkout ROS 2 from source and install ROS 2 system dependencies
		await io.mkdirP(rosWorkspaceDir + "/src");

		const options = {
			cwd: rosWorkspaceDir,
		};

		for (let vcsRepoFileUrl of vcsRepoFileUrlListNonEmpty) {
			await execBashCommand(
				`vcs import --force --recursive src/ --input '${vcsRepoFileUrl}'`,
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
			`find "${posixRosWorkspaceDir}" -type d -and -name "${repo["repo"]}" | xargs rm -rf`
		);

		// The repo file for the repository needs to be generated on-the-fly to
		// incorporate the custom repository URL and branch name, when a PR is
		// being built.
		let repoFullName = process.env.GITHUB_REPOSITORY as string;
		if (github.context.payload.pull_request) {
			repoFullName = github.context.payload.pull_request.head.repo.full_name;
		}
		let tokenAuth = importToken;
		if (tokenAuth !== "") {
			tokenAuth = tokenAuth.concat("@");
		}
		const headRef = process.env.GITHUB_HEAD_REF as string;
		const commitRef = headRef || github.context.sha;
		const repoFilePath = path.join(rosWorkspaceDir, "package.repo");
		const repoFileContent = `repositories:
  ${repo["repo"]}:
    type: git
    url: 'https://${tokenAuth}github.com/${repoFullName}.git'
    version: '${commitRef}'`;
		fs.writeFileSync(repoFilePath, repoFileContent);
		await execBashCommand(
			"vcs import --force --recursive src/ --input package.repo",
			undefined,
			options
		);

		// Remove all repositories the package under test does not depend on, to
		// avoid having rosdep installing unrequired dependencies.
		await execBashCommand(
			`diff --new-line-format="" --unchanged-line-format="" <(colcon list -p) <(colcon list --packages-up-to ${packageNameList.join(
				" "
			)} -p) | xargs rm -rf`,
			undefined,
			options
		);

		// Install ROS dependencies for each distribution being sourced
		if (targetRos1Distro) {
			await execBashCommand(
				`DEBIAN_FRONTEND=noninteractive RTI_NC_LICENSE_ACCEPTED=yes rosdep install -r --from-paths src --ignore-src --rosdistro ${targetRos1Distro} -y || true`,
				undefined,
				options
			);
		}
		if (targetRos2Distro) {
			await execBashCommand(
				`DEBIAN_FRONTEND=noninteractive RTI_NC_LICENSE_ACCEPTED=yes rosdep install -r --from-paths src --ignore-src --rosdistro ${targetRos2Distro} -y || true`,
				undefined,
				options
			);
		}

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
			`--packages-up-to ${packageNameList.join(" ")}`,
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
			`--packages-select ${packageNameList.join(" ")}`,
			`${extra_options.join(" ")}`,
		].join(" ");
		await execBashCommand(colconTestCmd, colconCommandPrefix, options);

		// ignoreReturnCode, check comment above in --initial
		const colconLcovResultCmd = [
			`colcon lcov-result`,
			`--filter ${coverageIgnorePattern}`,
			`--packages-select ${packageNameList.join(" ")}`,
		].join(" ");
		await execBashCommand(colconLcovResultCmd, colconCommandPrefix, {
			cwd: rosWorkspaceDir,
			ignoreReturnCode: true,
		});

		const colconCoveragepyResultCmd = [
			`colcon coveragepy-result`,
			`--packages-select ${packageNameList.join(" ")}`,
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
