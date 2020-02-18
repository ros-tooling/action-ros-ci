# `action-ros-ci`

[![GitHub Action Status](https://github.com/ros-tooling/action-ros-ci/workflows/Test%20action-ros-ci/badge.svg)](https://github.com/ros-tooling/action-ros-ci) [![Greenkeeper badge](https://badges.greenkeeper.io/ros-tooling/action-ros-ci.svg)](https://greenkeeper.io/)

This action builds a [ROS, or ROS 2](https://index.ros.org/doc/ros/) workspace from source, and run colon-test on the package under test.

## Requirements

This action requires ROS development tools (`colcon`, `rosdep`, `vcs`) to be installed on the CI worker instance.

On Linux, the setup can be done through [`ros-tooling/setup-ros`](https://github.com/ros-tooling/setup-ros), or by running the action in a Docker image containing the appropriate binaries.

## Overview

The action first assembles a workspace, then run `colcon build`, and `colcon test` in it.

The workspace is built by running:
* `vcs import` on the repo file specified through the `vcs-repo-file-url` argument (defaults to `https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos`).
* checkout the code under test in the workspace
* run `colcon build` for all packages specified in `package-name`
* run `colcon test` for all packages specified in `package-name`

## Usage

See [action.yml](action.yml)

```yaml
steps:
- uses: ros-tooling/setup-ros@master
- uses: ros-tooling/action-ros-ci@master
  with:
    package-name: ament_copyright
```

## License

The scripts and documentation in this project are released under the [Apache 2](LICENSE)
