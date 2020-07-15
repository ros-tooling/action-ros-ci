# `action-ros-ci`

[![GitHub Action Status](https://github.com/ros-tooling/action-ros-ci/workflows/Test%20action-ros-ci/badge.svg)](https://github.com/ros-tooling/action-ros-ci)
[![Dependabot Status](https://api.dependabot.com/badges/status?host=github&repo=ros-tooling/action-ros-ci)](https://dependabot.com)

This action builds a [ROS, or ROS 2](https://index.ros.org/doc/ros/) workspace from source, and run colon-test on the package under test.

## Requirements

This action requires the following ROS development tools to be installed (and initialized if applicable) on the CI worker instance:

```
curl
colcon-common-extensions
colcon-lcov-result  # Optional
colcon-coveragepy-result
colcon-mixin
rosdep
vcstool
```

On Linux, the setup can be done through [`ros-tooling/setup-ros`](https://github.com/ros-tooling/setup-ros), or by running the action in a Docker image containing the appropriate binaries.

## Overview

The action first assembles a workspace, then runs `colcon build`, and `colcon test` in it.

The workspace is built by running:
* `vcs import` on the repo file specified through the `vcs-repo-file-url` argument (defaults to `https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos`).
* checkout the code under test in the workspace using `vcs`
* run `colcon build` for all packages specified in `package-name`
* run `colcon test` for all packages specified in `package-name`

## Action Output

This action defines [an output variable](https://help.github.com/en/actions/building-actions/metadata-syntax-for-github-actions#outputs) `ros-workspace-directory-name`.
It contains the path to the root of the ROS workspace assembled by the action.

The variable value should be used to retrieve logs, binaries, etc. after the action completes.

## Usage

See [action.yml](action.yml) to get the list of flags supported by this action.

[action-ros-ci-template](https://github.com/ros-tooling/action-ros-ci-template) offers a template for using `action-ros-ci`.

### Build and run `ament_copyright` tests

```yaml
steps:
- uses: ros-tooling/setup-ros@0.0.25
- uses: ros-tooling/action-ros-ci@0.0.15
  with:
    package-name: ament_copyright
```

### Build with a custom `repos` or `rosinstall` file

You can specify your own repos file using the `vcs-repo-file-url` input.
You can also automatically generate your package's dependencies using the following workflow:

```yaml
steps:
- uses: actions/checkout@v2
- uses: ros-tooling/setup-ros@0.0.25
# Run the generator and output the results to a file.
- run: |
    rosinstall_generator <package-name> --rosdistro <target-distro> \
    --deps-only --deps --upstream-development > /tmp/deps.repos
# Pass the file to the action
- uses: ros-tooling/action-ros-ci@0.0.15
  with:
    package-name: my_package
    vcs-repo-file-url: /tmp/deps.repos
```

### Enable Address Sanitizer to automatically report memory issues

[ASan][AddressSanitizer] is an open-source tool developed to automatically report
memory corruption bugs.

```yaml
    steps:
    - uses: ros-tooling/setup-ros@0.0.25
    - uses: ros-tooling/action-ros-ci@0.0.15
      with:
        colcon-mixin-name: asan
        colcon-mixin-repository: https://raw.githubusercontent.com/colcon/colcon-mixin-repository/3e627e0fa30db85aea05a50e2c61a9832664d236/index.yaml
        package-name: my_package
```

To look for detected memory errors, check the build logs for entries containing `ERROR: AddressSanitizer`. Example:

```
==9442== ERROR: AddressSanitizer heap-use-after-free on address 0x7f7ddab8c084 at pc 0x403c8c bp 0x7fff87fb82d0 sp 0x7fff87fb82c8
```

ASan is analyzing memory issues at runtime. ASan diagnostic messages will be emitted by the package tests when they run.

### Generate and process code coverage data

#### Generate code coverage information using `lcov` and `colcon-lcov-result`

If the compiler is invoked with the appropriate flags, `action-ros-ci` will use
[`colcon-lcov-result`](https://github.com/colcon/colcon-lcov-result) to generate
coverage information.

Flags can be passed manually using, for instance, `extra-cmake-args`, but it is
preferable to use a `colcon` mixin to pass the appropriate flags automatically.

```yaml
    steps:
    - uses: ros-tooling/setup-ros@0.0.25
    - uses: ros-tooling/action-ros-ci@0.0.15
      with:
        package-name: my_package
        colcon-mixin-name: coverage-gcc
        # If possible, pin the repository in the workflow to a specific commit to avoid
        # changes in colcon-mixin-repository from breaking your tests.
        colcon-mixin-repository: https://raw.githubusercontent.com/colcon/colcon-mixin-repository/5c45b95018788deff62202aaa831ad4c20ebe2c6/index.yaml
```

#### Generate code coverage information using `coveragepy` and `colcon-coveragepy-result`

If `colcon` is invoked with the `coverage-pytest` mixin, `action-ros-ci` will use
[`colcon-coveragepy-result`](https://github.com/colcon/colcon-coveragepy-result) to generate
coverage information.

Flags can be passed manually using, for instance, `extra-cmake-args`, but it is
preferable to use a `colcon` mixin to pass the appropriate flags automatically.

```yaml
    steps:
    - uses: ros-tooling/setup-ros@0.0.25
    - uses: ros-tooling/action-ros-ci@0.0.15
      with:
        package-name: my_package
        colcon-mixin-name: coverage-pytest
        # If possible, pin the repository in the workflow to a specific commit to avoid
        # changes in colcon-mixin-repository from breaking your tests.
        colcon-mixin-repository: https://raw.githubusercontent.com/colcon/colcon-mixin-repository/5c45b95018788deff62202aaa831ad4c20ebe2c6/index.yaml
```

#### Integrate `action-ros-ci` with `codecov`

The generated code coverage information can be uploaded to [codecov.io](https://codecov.io/).
In this case, you will need to setup a secret `CODECOV_TOKEN` in [your repository settings][creating-encrypted-secrets].

See [action/codecov-action](https://github.com/codecov/codecov-action) documentation for more information about how to setup the action.

```yaml
    steps:
    - uses: ros-tooling/setup-ros@0.0.25
    - uses: ros-tooling/action-ros-ci@0.0.15
      with:
        package-name: my_package
        colcon-mixin-name: coverage-gcc
        # If possible, pin the repository in the workflow to a specific commit to avoid
        # changes in colcon-mixin-repository from breaking your tests.
        colcon-mixin-repository: https://raw.githubusercontent.com/colcon/colcon-mixin-repository/5c45b95018788deff62202aaa831ad4c20ebe2c6/index.yaml
    - uses: codecov/codecov-action@v1.0.7
      with:
        token: ${{ secrets.CODECOV_TOKEN }}
        file: ros_ws/lcov/total_coverage.info
        flags: unittests
        name: codecov-umbrella
```

You will also need to add a `codecov.yaml` configuration file (at the root of your repo):

```yaml
fixes:
  - "ros_ws/src/my_package/::"
```

The configuration file is required to let codecov map the workspace directory structure, to the Git repository structure, and setup the links between codecov and GitHub properly.

### Store `colcon` logs as build artifacts

GitHub workflows can persist data generated in workers during the build using [artifacts](persisting-workflow-data-using-artifacts). `action-ros-ci` generated colcon logs can be saved as follows:

```yaml
    - uses: ros-tooling/action-ros-ci@0.0.15
      id: action_ros_ci_step
      with:
        package-name: ament_copyright
    - uses: actions/upload-artifact@v1
      with:
        name: colcon-logs
        path: ${{ steps.action_ros_ci_step.outputs.ros-workspace-directory-name }}/log
    - if: always()  # upload the logs even when the build fails
```


## License

The scripts and documentation in this project are released under the [Apache 2](LICENSE) license.

[creating-encrypted-secrets]: https://help.github.com/en/actions/configuring-and-managing-workflows/creating-and-storing-encrypted-secrets#creating-encrypted-secrets
[persisting-workflow-data-using-artifacts]: https://help.github.com/en/actions/configuring-and-managing-workflows/persisting-workflow-data-using-artifacts
[AddressSanitizer]: https://en.wikipedia.org/wiki/AddressSanitizer
