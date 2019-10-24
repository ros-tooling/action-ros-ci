import * as core from "@actions/core";
import * as exec from "@actions/exec";
import * as github from "@actions/github";
import * as io from "@actions/io";
import * as path from 'path';

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
        ["-c",
        "curl https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos | vcs import src/"],
        options);

    // If the package under tests is part of ros2.repos, remove it first.
    // We do not want to allow the "default" head state of the package to
    // to be present in the workspace, and colcon will fail stating it found twice
    // a package with an identical name.
    await exec.exec(
        "bash",
        ["-c",
        `find "${ros2WorkspaceDir}" -type d -and -name "${repo["repo"]}" | xargs rm -rf`]);

    // The repo file for the repository needs to be generated on-the-fly to
    // incorporate the custom repository URL and branch name, when a PR is
    // being built.
    const headRef = process.env.GITHUB_HEAD_REF as string;
    const commitRef = headRef || github.context.sha;
    await exec.exec(
        "bash",
        ["-c", `vcs import src/ << EOF
repositories:
  ${repo["repo"]}:
    type: git
    url: "https://github.com/${repo["owner"]}/${repo["repo"]}.git"
    version: "${commitRef}"
EOF`], options);

    // Remove all repositories the package under test does not depend on, to
    // avoid having rosdep installing unrequired dependencies.
    await exec.exec(
        "bash",
        ["-c",
         `diff --new-line-format="" --unchanged-line-format="" <(colcon list -p) <(colcon list --packages-up-to ${packageName} -p) | xargs rm -rf`],
        options);

    // For "latest" builds, rosdep often misses some keys, adding "|| true", to
    // ignore those failures, as it is often non-critical.
    await exec.exec(
        "bash",
        ["-c", "DEBIAN_FRONTEND=noninteractive RTI_NC_LICENSE_ACCEPTED=yes rosdep install -r --from-paths src --ignore-src --rosdistro eloquent -y || true"],
        options);

     if (colconMixinName !== "" && colconMixinRepo !== "") {
       await exec.exec("colcon", ["mixin", "add", "default", colconMixinRepo]);
       await exec.exec("colcon", ["mixin", "update", "default"]);
     }

    let extra_options: string[] = [];
    if (colconMixinName !== "") {
      extra_options.concat(["--mixin", colconMixinName]);
    }
    await exec.exec(
        "colcon",
        ["build", "--event-handlers", "console_cohesion+", "--packages-up-to",
        packageName, "--symlink-install"].concat(extra_options), options);
    await exec.exec(
        "colcon",
        ["test", "--event-handlers", "console_cohesion+", "--packages-select",
        packageName, "--return-code-on-test-failure"].concat(extra_options), options);
  } catch (error) {
    core.setFailed(error.message);
  }
}

run();
