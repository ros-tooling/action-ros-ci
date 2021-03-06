# `action-ros-ci`

[![GitHub Action Status](https://github.com/ros-tooling/action-ros-ci/workflows/Test%20action-ros-ci/badge.svg)](https://github.com/ros-tooling/action-ros-ci)
[![Dependabot Status](https://api.dependabot.com/badges/status?host=github&repo=ros-tooling/action-ros-ci)](https://dependabot.com)
[![codecov](https://codecov.io/gh/ros-tooling/action-ros-ci/branch/master/graph/badge.svg)](https://codecov.io/gh/ros-tooling/action-ros-ci)

This action builds and tests a [ROS](http://wiki.ros.org/) or [ROS 2](https://docs.ros.org/en/rolling/) workspace from source.

1. [Requirements](#Requirements)
1. [Overview](#Overview)
1. [Action Output](#Action-Output)
1. [Usage](#Usage)
   1. [Build and run tests for your ROS 2 package](#Build-and-run-tests-for-your-ROS-2-package)
   1. [Build with a custom `repos` or `rosinstall` file](#Build-with-a-custom-repos-or-rosinstall-file)
   1. [Build a ROS 1 workspace](#Build-a-ROS-1-workspace)
   1. [Use a `colcon` `defaults.yaml` file](#Use-a-colcon-defaultsyaml-file)
   1. [Enable Address Sanitizer to automatically report memory issues](#Enable-Address-Sanitizer-to-automatically-report-memory-issues)
   1. [Generate and process code coverage data](#Generate-and-process-code-coverage-data)
   1. [Store `colcon` logs as build artifacts](#Store-colcon-logs-as-build-artifacts)
   1. [Use with private repos](#Use-with-private-repos)
   1. [Interdependent pull requests or merge requests](#Interdependent-pull-requests-or-merge-requests)
1. [Developing](#Developing)
1. [License](#License)

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

- `vcs import` on the repo file(s) specified through the `vcs-repo-file-url` argument, if any (defaults to none)
- checkout the code under test in the workspace using `vcs`
- `rosdep install` for the workspace, to get its dependencies
- run `colcon build` for all packages specified in `package-name`
- run `colcon test` for all packages specified in `package-name`

This action requires targeting a ROS or ROS 2 distribution explicitly.
This is provided via the `target-ros1-distro` or `target-ros2-distro` inputs, respectively.
Either or both may be specified, if neither is provided an error will be raised.
This input is used to `source setup.sh` for any installed ROS binaries (e.g. installed using [`ros-tooling/setup-ros`](https://github.com/ros-tooling/setup-ros)), as well as used as an argument to `rosdep install`.

## Action Output

This action defines [an output variable](https://help.github.com/en/actions/building-actions/metadata-syntax-for-github-actions#outputs): `ros-workspace-directory-name`.
It contains the path to the root of the ROS workspace assembled by the action.

The variable value should be used to retrieve logs, binaries, etc. after the action completes.

## Usage

See [`action.yml`](action.yml) to get the list of inputs supported by this action.

[action-ros-ci-template](https://github.com/ros-tooling/action-ros-ci-template) offers a template for using `action-ros-ci`.

### Build and run tests for your ROS 2 package

Here are the two simplest use-cases.

#### Using dependencies from binaries

In this case, `action-ros-ci` will rely on `setup-ros` for installing ROS 2 binaries.

```yaml
steps:
  - uses: ros-tooling/setup-ros@v0.1
    with:
      required-ros-distributions: foxy
  - uses: ros-tooling/action-ros-ci@v0.1
    with:
      package-name: my_package
      target-ros2-distro: foxy
```

#### Building ROS 2 dependencies from source

In this case, `action-ros-ci` will build all necessary ROS 2 dependencies of `my_package` from source.

```yaml
steps:
  - uses: ros-tooling/setup-ros@v0.1
  - uses: ros-tooling/action-ros-ci@v0.1
    with:
      package-name: my_package
      target-ros2-distro: foxy
      vcs-repo-file-url: https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
```

### Build with a custom `repos` or `rosinstall` file

You can specify your own repos file using the `vcs-repo-file-url` input.
You can also automatically generate your package's dependencies using the following workflow:

```yaml
steps:
  - uses: actions/checkout@v2
  - uses: ros-tooling/setup-ros@v0.1
  # Run the generator and output the results to a file.
  - run: |
      rosinstall_generator <package-name> --rosdistro <target-distro> \
      --deps-only --deps --upstream-development > /tmp/deps.repos
  # Pass the file to the action
  - uses: ros-tooling/action-ros-ci@v0.1
    with:
      package-name: my_package
      target-ros2-distro: foxy
      vcs-repo-file-url: /tmp/deps.repos
```

Note that the `actions/checkout` step is required when using a custom repos file from your repository.

### Build a ROS 1 workspace

Building a ROS 1 workspace works the same way.
Simply use `target-ros1-distro` instead of `target-ros2-distro`.

```yaml
steps:
  - uses: ros-tooling/setup-ros@v0.1
    with:
      required-ros-distributions: melodic
  - uses: ros-tooling/action-ros-ci@v0.1
    with:
      package-name: my_package
      target-ros1-distro: melodic
```

### Use a `colcon` `defaults.yaml` file

To use a [`colcon` `defaults.yaml` file](https://colcon.readthedocs.io/en/released/user/configuration.html#defaults-yaml), provide a valid JSON string through the `colcon-defaults` input.
This allows using a `colcon` option/argument that is not exposed by this action's inputs.

```yaml
steps:
  - uses: ros-tooling/setup-ros@v0.1
    with:
      required-ros-distributions: foxy
  - uses: ros-tooling/action-ros-ci@v0.1
    with:
      package-name: my_package
      target-ros2-distro: foxy
      colcon-defaults: |
        {
          "build": {
            "cmake-args": [
                "-DMY_CUSTOM_OPTION=ON"
            ]
          }
        }
```

### Enable Address Sanitizer to automatically report memory issues

[ASan][addresssanitizer] is an open-source tool developed to automatically report
memory corruption bugs.

```yaml
steps:
  - uses: ros-tooling/setup-ros@v0.1
    with:
      required-ros-distributions: foxy
  - uses: ros-tooling/action-ros-ci@v0.1
    with:
      colcon-mixin-name: asan
      colcon-mixin-repository: https://raw.githubusercontent.com/colcon/colcon-mixin-repository/3e627e0fa30db85aea05a50e2c61a9832664d236/index.yaml
      package-name: my_package
      target-ros2-distro: foxy
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
  - uses: ros-tooling/setup-ros@v0.1
    with:
      required-ros-distributions: foxy
  - uses: ros-tooling/action-ros-ci@v0.1
    with:
      package-name: my_package
      target-ros2-distro: foxy
      colcon-mixin-name: coverage-gcc
      # If possible, pin the repository in the workflow to a specific commit to avoid
      # changes in colcon-mixin-repository from breaking your tests.
      colcon-mixin-repository: https://raw.githubusercontent.com/colcon/colcon-mixin-repository/1ddb69bedfd1f04c2f000e95452f7c24a4d6176b/index.yaml
```

#### Generate code coverage information using `coveragepy` and `colcon-coveragepy-result`

If `colcon` is invoked with the `coverage-pytest` mixin, `action-ros-ci` will use
[`colcon-coveragepy-result`](https://github.com/colcon/colcon-coveragepy-result) to generate
coverage information.

Flags can be passed manually using, for instance, `extra-cmake-args`, but it is
preferable to use a `colcon` mixin to pass the appropriate flags automatically.

```yaml
steps:
  - uses: ros-tooling/setup-ros@v0.1
    with:
      required-ros-distributions: foxy
  - uses: ros-tooling/action-ros-ci@v0.1
    with:
      package-name: my_package
      target-ros2-distro: foxy
      colcon-mixin-name: coverage-pytest
      # If possible, pin the repository in the workflow to a specific commit to avoid
      # changes in colcon-mixin-repository from breaking your tests.
      colcon-mixin-repository: https://raw.githubusercontent.com/colcon/colcon-mixin-repository/1ddb69bedfd1f04c2f000e95452f7c24a4d6176b/index.yaml
```

#### Integrate `action-ros-ci` with `codecov`

The generated code coverage information can be uploaded to [codecov.io](https://codecov.io/).

For a private repo, you will need to setup a secret `CODECOV_TOKEN` in [your repository settings][creating-encrypted-secrets].
See [`codecov/codecov-action`](https://github.com/codecov/codecov-action) documentation for more information about how to setup the action.

```yaml
steps:
  - uses: actions/checkout@v2
  - uses: ros-tooling/setup-ros@v0.1
    with:
      required-ros-distributions: foxy
  - uses: ros-tooling/action-ros-ci@v0.1
    with:
      package-name: my_package
      target-ros2-distro: foxy
      colcon-mixin-name: coverage-gcc coverage-pytest
      # If possible, pin the repository in the workflow to a specific commit to avoid
      # changes in colcon-mixin-repository from breaking your tests.
      colcon-mixin-repository: https://raw.githubusercontent.com/colcon/colcon-mixin-repository/1ddb69bedfd1f04c2f000e95452f7c24a4d6176b/index.yaml
  - uses: codecov/codecov-action@v1.2.1
    with:
      token: ${{ secrets.CODECOV_TOKEN }}  # only needed for private repos
      files: ros_ws/lcov/total_coverage.info,ros_ws/coveragepy/.coverage
      flags: unittests
      name: codecov-umbrella
```

You will also need to add a [`codecov.yml` configuration file](https://docs.codecov.io/docs/codecovyml-reference) (at the root of your repo):

```yaml
fixes:
  # For each package in your repo
  - "ros_ws/src/*/my_repo/my_package/::"
```

The configuration file is required to let codecov map the workspace directory structure to the Git repository structure, and setup the links between codecov and GitHub properly.
Note here that `actions/checkout` is required because `codecov/codecov-action` needs the `codecov.yml` file.

### Store `colcon` logs as build artifacts

GitHub workflows can persist data generated in workers during the build using [artifacts](persisting-workflow-data-using-artifacts). `action-ros-ci` generated colcon logs can be saved as follows:

```yaml
steps:
  # ...
  - uses: ros-tooling/action-ros-ci@v0.1
    id: action_ros_ci_step
    with:
      package-name: ament_copyright
      target-ros2-distro: foxy
  - uses: actions/upload-artifact@v1
    with:
      name: colcon-logs
      path: ${{ steps.action_ros_ci_step.outputs.ros-workspace-directory-name }}/log
    if: always() # upload the logs even when the build fails
```

### Use with private repos

`action-ros-ci` needs a personal access token to be able to clone private repositories.
This may include the repository that is being tested as well as other repositories included in repos files provided through `vcs-repo-file-url`.
Generate a [personal access token](https://github.com/settings/tokens) with the "repo" scope and add it to your repo's [secrets][creating-encrypted-secrets].
For example, if your secret is called `REPO_TOKEN`:

```yaml
steps:
  # ...
  - uses: ros-tooling/action-ros-ci@v0.1
    with:
      package-name: my_package
      import-token: ${{ secrets.REPO_TOKEN }}
      # ...
```

### Interdependent pull requests or merge requests

This action allows declaring PR dependencies by providing:
* repos file(s) to override the one(s) defined through the `vcs-repo-file-url` action input
* supplemental repos file(s) to be used along with the rest

For example, this may be useful when your PR depends on PRs/MRs/branches from other repos for it to work or be properly tested.

Include links in your PR's description using the following format:

```
action-ros-ci-repos-override: https://gist.github.com/some-user/some.repos
action-ros-ci-repos-override: https://gist.github.com/some-user/some-other.repos
action-ros-ci-repos-supplemental: https://gist.github.com/some-user/some-supplemental.repos
action-ros-ci-repos-supplemental: file://path/to/some/other/supplemental.repos
```

## Developing

For developing and releasing `action-ros-ci`, see [`DEVELOPING.md`](DEVELOPING.md).

## License

The scripts and documentation in this project are released under the [Apache 2](LICENSE) license.

[creating-encrypted-secrets]: https://help.github.com/en/actions/configuring-and-managing-workflows/creating-and-storing-encrypted-secrets#creating-encrypted-secrets
[persisting-workflow-data-using-artifacts]: https://help.github.com/en/actions/configuring-and-managing-workflows/persisting-workflow-data-using-artifacts
[addresssanitizer]: https://en.wikipedia.org/wiki/AddressSanitizer
