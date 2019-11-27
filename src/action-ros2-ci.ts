import * as core from "@actions/core";
import * as exec from "@actions/exec";
import * as github from "@actions/github";
import * as io from "@actions/io";
import * as path from "path";

async function run() {
	try {
		const repo = github.context.repo;
		const workspace = process.env.GITHUB_WORKSPACE as string;

		const colconMixinName = core.getInput("colcon-mixin-name");
		const colconMixinRepo = core.getInput("colcon-mixin-repository");
		const packageName = core.getInput("package-name");
		const ros2WorkspaceDir = path.join(workspace, "ros2_ws");
		await exec.exec("rosdep", ["update"]);

		// Checkout ROS 2 from source and install ROS 2 system dependencies
		await io.mkdirP(ros2WorkspaceDir + "/src");

		const options = {
			cwd: ros2WorkspaceDir
		};
		await exec.exec(
			"bash",
			[
				"-c",
				"curl https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos | vcs import src/"
			],
			options
		);

		// If the package under tests is part of ros2.repos, remove it first.
		// We do not want to allow the "default" head state of the package to
		// to be present in the workspace, and colcon will fail stating it found twice
		// a package with an identical name.
		await exec.exec("bash", [
			"-c",
			`find "${ros2WorkspaceDir}" -type d -and -name "${repo["repo"]}" | xargs rm -rf`
		]);

		// The repo file for the repository needs to be generated on-the-fly to
		// incorporate the custom repository URL and branch name, when a PR is
		// being built.
		let repoFullName = process.env.GITHUB_REPOSITORY as string;
		if (github.context.payload.pull_request) {
			repoFullName = github.context.payload.pull_request.head.repo.full_name;
		}
		const headRef = process.env.GITHUB_HEAD_REF as string;
		const commitRef = headRef || github.context.sha;
		await exec.exec(
			"bash",
			[
				"-c",
				`vcs import src/ << EOF
repositories:
  ${repo["repo"]}:
    type: git
    url: "https://github.com/${repoFullName}.git"
    version: "${commitRef}"
EOF`
			],
			options
		);

		// Remove all repositories the package under test does not depend on, to
		// avoid having rosdep installing unrequired dependencies.
		await exec.exec(
			"bash",
			[
				"-c",
				`diff --new-line-format="" --unchanged-line-format="" <(colcon list -p) <(colcon list --packages-up-to ${packageName} -p) | xargs rm -rf`
			],
			options
		);

		// For "latest" builds, rosdep often misses some keys, adding "|| true", to
		// ignore those failures, as it is often non-critical.
		await exec.exec(
			"bash",
			[
				"-c",
				"DEBIAN_FRONTEND=noninteractive RTI_NC_LICENSE_ACCEPTED=yes rosdep install -r --from-paths src --ignore-src --rosdistro eloquent -y || true"
			],
			options
		);

		if (colconMixinName !== "" && colconMixinRepo !== "") {
			await exec.exec("colcon", ["mixin", "add", "default", colconMixinRepo]);
			await exec.exec("colcon", ["mixin", "update", "default"]);
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
		core.addPath(path.join(ros2WorkspaceDir, "install", "bin"));

		await exec.exec(
			"colcon",
			[
				"build",
				"--event-handlers",
				"console_cohesion+",
				"--packages-up-to",
				packageName,
				"--symlink-install"
			].concat(extra_options),
			options
		);
		await exec.exec(
			"colcon",
			[
				"test",
				"--event-handlers",
				"console_cohesion+",
				"--pytest-args",
				"'--cov=.'",
				"'--cov-report=xml'",
				"--packages-select",
				packageName,
				"--return-code-on-test-failure"
			].concat(extra_options),
			options
		);

		// ignoreReturnCode is set to true to avoid  having a lack of coverage
		// data fail the build.
		await exec.exec(
			"colcon",
			["lcov-result", "--packages-select", packageName],
			{ ignoreReturnCode: true }
		);
	} catch (error) {
		core.setFailed(error.message);
	}
}

run();
