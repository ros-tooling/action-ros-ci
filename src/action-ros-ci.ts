import * as core from "@actions/core";
import * as github from "@actions/github";
import * as im from "@actions/exec/lib/interfaces"; // eslint-disable-line no-unused-vars
import * as tr from "@actions/exec/lib/toolrunner";
import * as io from "@actions/io";
import * as os from "os";
import * as path from "path";
import * as url from "url";
import fs from "fs";
import retry from "async-retry";
import * as dep from "./dependencies";

const validROS1Distros: string[] = ["kinetic", "lunar", "melodic", "noetic"];
const validROS2Distros: string[] = [
	"dashing",
	"eloquent",
	"foxy",
	"galactic",
	"humble",
	"rolling",
];
const targetROS1DistroInput: string = "target-ros1-distro";
const targetROS2DistroInput: string = "target-ros2-distro";
const isLinux: boolean = process.platform == "linux";
const isWindows: boolean = process.platform == "win32";
const useMergeInstall: boolean = isWindows;

/**
 * Join string array using a single space and make sure to filter out empty elements.
 *
 * @param values the string values array
 * @returns the joined string
 */
export function filterNonEmptyJoin(values: string[]): string {
	return values.filter((v) => v.length > 0).join(" ");
}

/**
 * Check if a string is a valid JSON string.
 *
 * @param str the string to validate
 * @returns `true` if valid, `false` otherwise
 */
function isValidJson(str: string): boolean {
	try {
		JSON.parse(str);
	} catch (e) {
		return false;
	}
	return true;
}

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
		return url.pathToFileURL(path.resolve(vcsRepoFileUrl)).href;
	} else {
		return vcsRepoFileUrl;
	}
}

/**
 * Execute a shell command and wrap the output in a log group.
 *
 * @param   command         command to execute w/ any params
 * @param   force_bash      force running in bash shell, instead of os default. defaults to true.
 * @param   options         optional exec options.  See ExecOptions
 * @param   log_message     log group title.
 * @returns Promise<number> exit code
 */
export async function execShellCommand(
	command: string[],
	options?: im.ExecOptions,
	force_bash: boolean = true,
	log_message?: string
): Promise<number> {
	const use_bash = !isWindows || force_bash;
	if (use_bash) {
		// Bash commands needs to be flattened into a single string when passed to bash with "-c" switch
		command = [filterNonEmptyJoin(command)];
	}

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
			"%programfiles(x86)%\\Microsoft Visual Studio\\2019\\Enterprise\\VC\\Auxiliary\\Build\\vcvars64.bat",
			"&&",
			...(use_bash ? [`C:\\Program Files\\Git\\bin\\bash.exe`, `-c`] : []),
			...command,
		];
	} else {
		toolRunnerCommandLine = "bash";
		toolRunnerCommandLineArgs = ["-c", ...command];
	}
	const message =
		log_message ||
		`Invoking: ${toolRunnerCommandLine} ${toolRunnerCommandLineArgs}`;
	const runner: tr.ToolRunner = new tr.ToolRunner(
		toolRunnerCommandLine,
		toolRunnerCommandLineArgs,
		options
	);
	if (options && options.silent) {
		return runner.exec();
	}
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
	packageSelection: string[],
	skipKeys: string[],
	workspaceDir: string,
	options: im.ExecOptions,
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
	package_paths=$(colcon list --paths-only ${filterNonEmptyJoin(
		packageSelection
	)})
	# suppress errors from unresolved install keys to preserve backwards compatibility
	# due to difficulty reading names of some non-catkin dependencies in the ros2 core
	# see https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/#install-dependencies-using-rosdep
	rosdep install -r --from-paths $package_paths --ignore-src --skip-keys "rti-connext-dds-5.3.1 ${filterNonEmptyJoin(
		skipKeys
	)}" --rosdistro $DISTRO -y || true`;
	fs.writeFileSync(scriptPath, scriptContent, { mode: 0o766 });

	let exitCode = 0;
	if (ros1Distro) {
		exitCode += await execShellCommand(
			[`./${scriptName} ${ros1Distro}`],
			options
		);
	}
	if (ros2Distro) {
		exitCode += await execShellCommand(
			[`./${scriptName} ${ros2Distro}`],
			options
		);
	}
	return exitCode;
}

/**
 * Run tests and process & aggregate coverage results.
 *
 * @param colconCommandPrefix the prefix to use before colcon commands
 * @param options the exec options
 * @param testPackageSelection the package selection option
 * @param colconExtraArgs the extra args for 'colcon test'
 * @param coverageIgnorePattern the coverage filter pattern to use for lcov
 */
async function runTests(
	colconCommandPrefix: string[],
	options: im.ExecOptions,
	testPackageSelection: string[],
	colconExtraArgs: string[],
	coverageIgnorePattern: string[]
): Promise<void> {
	const doLcovResult = !isWindows; // lcov-result not supported in Windows
	const doTests = !isWindows; // Temporarily disable colcon test on Windows to unblock Windows CI builds: https://github.com/ros-tooling/action-ros-ci/pull/712#issuecomment-969495087

	if (doLcovResult) {
		// ignoreReturnCode is set to true to avoid having a lack of coverage
		// data fail the build.
		const colconLcovInitialCmd = [`colcon`, `lcov-result`, `--initial`];
		await execShellCommand(
			[...colconCommandPrefix, ...colconLcovInitialCmd],
			{
				...options,
				ignoreReturnCode: true,
			},
			false
		);
	}

	let colconTestCmd = [
		`colcon`,
		`test`,
		`--event-handlers=console_cohesion+`,
		...testPackageSelection,
		...colconExtraArgs,
	];
	if (useMergeInstall) {
		colconTestCmd = [...colconTestCmd, `--merge-install`];
	}

	if (doTests) {
		await execShellCommand(
			[...colconCommandPrefix, ...colconTestCmd],
			options,
			false
		);

		/**
		 * Display all test results first and ignore the return code. Then, display only failing
		 * tests with their output and let non-zero return code fail the job.
		 */
		const colconTestResultCmd = ["colcon", "test-result"];
		const colconTestResultAllCmd = [...colconTestResultCmd, "--all"];
		const colconTestResultVerboseCmd = [...colconTestResultCmd, "--verbose"];
		await execShellCommand(
			[...colconCommandPrefix, ...colconTestResultAllCmd],
			{
				...options,
				ignoreReturnCode: true,
			},
			false
		);
		await execShellCommand(
			[...colconCommandPrefix, ...colconTestResultVerboseCmd],
			options,
			false
		);
	}

	if (doLcovResult) {
		// ignoreReturnCode, check comment above in --initial
		const colconLcovResultCmd = [
			`colcon`,
			`lcov-result`,
			...testPackageSelection,
			...coverageIgnorePattern,
			`--verbose`,
		];
		await execShellCommand(
			[...colconCommandPrefix, ...colconLcovResultCmd],
			{
				...options,
				ignoreReturnCode: true,
			},
			false
		);
	}

	const colconCoveragepyResultCmd = [
		`colcon`,
		`coveragepy-result`,
		...testPackageSelection,
		`--verbose`,
		`--coverage-report-args`,
		`-m`,
	];
	await execShellCommand(
		[...colconCommandPrefix, ...colconCoveragepyResultCmd],
		options,
		false
	);
}

async function run_throw(): Promise<void> {
	const repo = github.context.repo;
	const workspace = process.env.GITHUB_WORKSPACE as string;

	const colconDefaults = core.getInput("colcon-defaults");
	const colconMixinRepo = core.getInput("colcon-mixin-repository");

	const extraCmakeArgsInput = core.getInput("extra-cmake-args");
	const extraCmakeArgs = extraCmakeArgsInput
		? ["--cmake-args", extraCmakeArgsInput]
		: [];

	const coverageIgnorePatternInput = core.getInput("coverage-ignore-pattern");
	const coverageIgnorePattern = coverageIgnorePatternInput
		? ["--filter", coverageIgnorePatternInput]
		: [];

	const colconExtraArgsInput = core.getInput("colcon-extra-args");
	const colconExtraArgs = colconExtraArgsInput ? [colconExtraArgsInput] : [];

	const importToken = core.getInput("import-token");

	const packageNameInput = core.getInput("package-name");
	const packageNames = packageNameInput
		? packageNameInput.split(RegExp("\\s"))
		: undefined;
	const buildPackageSelection = packageNames
		? ["--packages-up-to", ...packageNames]
		: [];
	const testPackageSelection = packageNames
		? ["--packages-select", ...packageNames]
		: [];

	const rosdepSkipKeysInput = core.getInput("rosdep-skip-keys");
	const rosdepSkipKeys = rosdepSkipKeysInput
		? rosdepSkipKeysInput.split(RegExp("\\s"))
		: undefined;
	const rosdepSkipKeysSelection = rosdepSkipKeys ? [...rosdepSkipKeys] : [];

	const rosWorkspaceName = "ros_ws";
	core.setOutput("ros-workspace-directory-name", rosWorkspaceName);
	const rosWorkspaceDir = path.join(workspace, rosWorkspaceName);
	const targetRos1Distro = core.getInput(targetROS1DistroInput);
	const targetRos2Distro = core.getInput(targetROS2DistroInput);
	const vcsRepoFileUrlListAsString = core.getInput("vcs-repo-file-url") || "";
	let vcsRepoFileUrlList = vcsRepoFileUrlListAsString.split(RegExp("\\s"));
	const skipTests = core.getInput("skip-tests") === "true";

	// Check if PR overrides/adds supplemental repos files
	const vcsReposOverride = dep.getReposFilesOverride(github.context.payload);
	const vcsReposSupplemental = dep.getReposFilesSupplemental(
		github.context.payload
	);
	await core.group(
		"Repos files: override" + (vcsReposOverride.length === 0 ? " - none" : ""),
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

	const vcsRepoFileUrlListNonEmpty = vcsRepoFileUrlList.filter((x) => x != "");

	if (!validateDistros(targetRos1Distro, targetRos2Distro)) {
		return;
	}

	// rosdep does not reliably work on Windows, see
	// ros-infrastructure/rosdep#610 for instance. So, we do not run it.
	if (!isWindows) {
		await retry(
			async () => {
				await execShellCommand(["rosdep update --include-eol-distros"]);
			},
			{
				retries: 3,
			}
		);
	}

	// Reset colcon configuration and create defaults file if one was provided.
	await io.rmRF(path.join(os.homedir(), ".colcon"));
	let colconDefaultsFile = "";
	if (colconDefaults.length > 0) {
		if (!isValidJson(colconDefaults)) {
			core.setFailed(
				`colcon-defaults value is not a valid JSON string:\n${colconDefaults}`
			);
			return;
		}
		colconDefaultsFile = path.join(
			fs.mkdtempSync(path.join(os.tmpdir(), "colcon-defaults-")),
			"defaults.yaml"
		);
		fs.writeFileSync(colconDefaultsFile, colconDefaults);
	}

	// Wipe out the workspace directory to ensure the workspace is always
	// identical.
	await io.rmRF(rosWorkspaceDir);

	// Checkout ROS 2 from source and install ROS 2 system dependencies
	await io.mkdirP(rosWorkspaceDir + "/src");

	const options: im.ExecOptions = {
		cwd: rosWorkspaceDir,
		env: {
			...process.env,
			ROS_VERSION: targetRos2Distro ? "2" : "1",
			ROS_PYTHON_VERSION:
				targetRos2Distro || (targetRos1Distro && targetRos1Distro == "noetic")
					? "3"
					: "2",
		},
	};
	if (colconDefaultsFile !== "") {
		options.env = {
			...options.env,
			COLCON_DEFAULTS_FILE: colconDefaultsFile,
		};
	}
	if (isLinux) {
		options.env = {
			...options.env,
			DEBIAN_FRONTEND: "noninteractive",
		};
	}

	if (importToken !== "") {
		// Unset all local extraheader config entries possibly set by actions/checkout,
		// because local settings take precedence and the default token used by
		// actions/checkout might not have the right permissions for any/all repos
		await execShellCommand(
			[
				`/usr/bin/git config --local --unset-all http.https://github.com/.extraheader || true`,
			],
			options
		);
		await execShellCommand(
			[
				String.raw`/usr/bin/git submodule foreach --recursive git config --local --name-only --get-regexp 'http\.https\:\/\/github\.com\/\.extraheader'` +
					` && git config --local --unset-all 'http.https://github.com/.extraheader' || true`,
			],
			options
		);
		// Use a global insteadof entry because local configs aren't observed by git clone
		await execShellCommand(
			[
				`/usr/bin/git config --global url.https://x-access-token:${importToken}@github.com.insteadof 'https://github.com'`,
			],
			options
		);
		// same as last three comands but for ssh urls
		await execShellCommand(
			[
				`/usr/bin/git config --local --unset-all git@github.com:.extraheader || true`,
			],
			options
		);
		await execShellCommand(
			[
				String.raw`/usr/bin/git submodule foreach --recursive git config --local --name-only --get-regexp 'git@github\.com:.extraheader'` +
					` && git config --local --unset-all 'git@github.com:.extraheader' || true`,
			],
			options
		);
		// Use a global insteadof entry because local configs aren't observed by git clone (ssh)
		await execShellCommand(
			[
				`/usr/bin/git config --global url.https://x-access-token:${importToken}@github.com/.insteadof 'git@github.com:'`,
			],
			options
		);
		if (core.isDebug()) {
			await execShellCommand(
				[`/usr/bin/git config --list --show-origin || true`],
				options
			);
		}
	}

	// Make sure to delete root .colcon directory if it exists
	// This is because, for some reason, using Docker, commands might get run as root
	await execShellCommand(
		[`rm -rf ${path.join(path.sep, "root", ".colcon")} || true`],
		{ ...options, silent: true }
	);

	for (const vcsRepoFileUrl of vcsRepoFileUrlListNonEmpty) {
		const resolvedUrl = resolveVcsRepoFileUrl(vcsRepoFileUrl);
		await execShellCommand(
			[`vcs import --force --recursive src/ --input ${resolvedUrl}`],
			options
		);
	}

	// If the package under tests is part of ros.repos, remove it first.
	// We do not want to allow the "default" head state of the package to
	// to be present in the workspace, and colcon will fail stating it found twice
	// a package with an identical name.
	let posixPathScriptPath = "";
	if (isWindows) {
		// vcs might output paths with a mix of forward slashes and backslashes on Windows
		posixPathScriptPath = path.join(rosWorkspaceDir, "slash_converter.sh");
		const scriptContent = String.raw`#!/bin/bash
while IFS= read -r line; do
	echo "$line" | sed 's/\\/\//g'
done`;
		fs.writeFileSync(posixPathScriptPath, scriptContent, {
			mode: 0o766,
		});
		posixPathScriptPath = posixPathScriptPath.replace(/\\/g, "/");
	}
	const posixRosWorkspaceDir = isWindows
		? rosWorkspaceDir.replace(/\\/g, "/")
		: rosWorkspaceDir;
	await execShellCommand(
		[
			`vcs diff -s --repos ${posixRosWorkspaceDir} | cut -d ' ' -f 1 | grep "${repo["repo"]}$"` +
				(isWindows ? ` | ${posixPathScriptPath}` : "") +
				` | xargs rm -rf`,
		],
		undefined
	);

	// The repo file for the repository needs to be generated on-the-fly to
	// incorporate the custom repository URL and branch name, when a PR is
	// being built.
	let repoFullName = process.env.GITHUB_REPOSITORY as string;
	if (github.context.payload.pull_request) {
		repoFullName = github.context.payload.pull_request.base.repo.full_name;
	}
	const headRef = process.env.GITHUB_HEAD_REF as string;
	let commitRef = headRef || github.context.sha;
	if (github.context.payload.pull_request) {
		commitRef = github.context.sha;
	}
	const repoFilePath = path.join(rosWorkspaceDir, "package.repo");
	// Add a random string prefix to avoid naming collisions when checking out the test repository
	const randomStringPrefix = Math.random().toString(36).substring(2, 15);
	const repoFileContent = `repositories:
  ${randomStringPrefix}/${repo["repo"]}:
    type: git
    url: 'https://github.com/${repoFullName}.git'
    version: '${commitRef}'`;
	fs.writeFileSync(repoFilePath, repoFileContent);
	await execShellCommand(
		["vcs import --force --recursive src/ < package.repo"],
		options
	);

	// Print HEAD commits of all repos
	await execShellCommand(["vcs log -l1 src/"], options);

	if (isLinux) {
		// Always update APT before installing packages on Ubuntu
		await execShellCommand(["sudo apt-get update"]);
	}
	// rosdep does not really work on Windows, so do not use it
	// See: https://github.com/ros-infrastructure/rosdep/issues/610
	if (!isWindows) {
		await installRosdeps(
			buildPackageSelection,
			rosdepSkipKeysSelection,
			rosWorkspaceDir,
			options,
			targetRos1Distro,
			targetRos2Distro
		);
	}

	if (colconDefaults.includes(`"mixin"`) && colconMixinRepo !== "") {
		await execShellCommand(
			[`colcon`, `mixin`, `add`, `default`, `${colconMixinRepo}`],
			options,
			false
		);
		await execShellCommand(
			[`colcon`, `mixin`, `update`, `default`],
			options,
			false
		);
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
	let colconCommandPrefix: string[] = [];
	if (isLinux) {
		if (targetRos1Distro) {
			const ros1SetupPath = `/opt/ros/${targetRos1Distro}/setup.sh`;
			if (fs.existsSync(ros1SetupPath)) {
				colconCommandPrefix = [
					...colconCommandPrefix,
					`source ${ros1SetupPath}`,
					`&&`,
				];
			}
		}
		if (targetRos2Distro) {
			const ros2SetupPath = `/opt/ros/${targetRos2Distro}/setup.sh`;
			if (fs.existsSync(ros2SetupPath)) {
				colconCommandPrefix = [
					...colconCommandPrefix,
					`source ${ros2SetupPath}`,
					`&&`,
				];
			}
		}
	} else if (isWindows) {
		// Windows only supports ROS2
		if (targetRos2Distro) {
			const ros2SetupPath = `c:/dev/${targetRos2Distro}/ros2-windows/setup.bat`;
			if (fs.existsSync(ros2SetupPath)) {
				colconCommandPrefix = [
					...colconCommandPrefix,
					`${ros2SetupPath}`,
					`&&`,
				];
			}
		}
	}

	let colconBuildCmd = [
		`colcon`,
		`build`,
		`--symlink-install`,
		...buildPackageSelection,
		...colconExtraArgs,
		...extraCmakeArgs,
		`--event-handlers=console_cohesion+`,
	];
	if (useMergeInstall) {
		colconBuildCmd = [...colconBuildCmd, `--merge-install`];
	}
	await execShellCommand(
		[...colconCommandPrefix, ...colconBuildCmd],
		options,
		false
	);

	if (!skipTests) {
		await runTests(
			colconCommandPrefix,
			options,
			testPackageSelection,
			colconExtraArgs,
			coverageIgnorePattern
		);
	} else {
		core.info("Skipping tests");
	}

	if (importToken !== "") {
		// Unset config so that it doesn't leak to other actions
		await execShellCommand(
			[
				`/usr/bin/git config --global --unset-all url.https://x-access-token:${importToken}@github.com.insteadof`,
			],
			options
		);
	}
}

async function run(): Promise<void> {
	try {
		await run_throw();
	} catch (error) {
		let errorMessage = "Unknown error";
		if (error instanceof Error) {
			errorMessage = error.message;
		}
		core.setFailed(errorMessage);
	}
}

run();
