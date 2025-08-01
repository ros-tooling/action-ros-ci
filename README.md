# `action-ros-ci`

[![GitHub Action Status](https://github.com/ros-tooling/action-ros-ci/workflows/Test%20action-ros-ci/badge.svg)](https://github.com/ros-tooling/action-ros-ci)
[![Dependabot Status](https://api.dependabot.com/badges/status?host=github&repo=ros-tooling/action-ros-ci)](https://dependabot.com)
[![codecov](https://codecov.io/gh/ros-tooling/action-ros-ci/branch/master/graph/badge.svg)](https://codecov.io/gh/ros-tooling/action-ros-ci)

This action builds and tests a [ROS](http://wiki.ros.org/) or [ROS 2](https://docs.ros.org/en/rolling/) workspace from source.

1. [Requirements](#Requirements)
1. [Overview](#Overview)
1. [Action Output](#Action-Output)
1. [Usage](#Usage)
   1. [Base Usage Patterns](#Base-Usage-Patterns)
   1. [Building ROS 2 core dependencies from source](#building-ros-2-core-dependencies-from-source)
   1. [Build with a custom `repos` or `rosinstall` file](#Build-with-a-custom-repos-or-rosinstall-file)
   1. [Build a ROS 1 workspace](#Build-a-ROS-1-workspace)
   1. [Skip tests](#Skip-tests)
   1. [Use a `colcon` `defaults.yaml` file](#Use-a-colcon-defaultsyaml-file)
   1. [Do not use `--symlink-install` when building](#Do-not-use---symlink-install-when-building)
   1. [Enable Address Sanitizer to automatically report memory issues](#Enable-Address-Sanitizer-to-automatically-report-memory-issues)
   1. [Generate and process code coverage data](#Generate-and-process-code-coverage-data)
   1. [Store `colcon` logs as build artifacts](#Store-colcon-logs-as-build-artifacts)
   1. [Use with private repos](#Use-with-private-repos)
   1. [Skip `rosdep install` and run `rosdep check`](#Skip-rosdep-install-and-run-rosdep-check)
   1. [Interdependent pull requests or merge requests](#Interdependent-pull-requests-or-merge-requests)
1. [Developing](#Developing)
1. [License](#License)

## Requirements

This action requires ROS development tools to be installed (and initialized if applicable) on the CI worker instance.

For Ubuntu Linux builds, it is recommended to run your CI on an appropriate base container image from [`ros-tooling/setup-ros-docker`](https://github.com/ros-tooling/setup-ros-docker).

If you want to use a different base environment, you can prepare that environment for building a ROS workspace with [`ros-tooling/setup-ros`](https://github.com/ros-tooling/setup-ros).

For reference, these are the minimum tools needed by the action, which are provided automatically by the options mentioned above:

- Base development tools, `cmake` and compilers for the platform
- `vcstool`
- `rosdep`
- `colcon-common-extensions`
- `colcon-lcov-result` (Optional)
- `colcon-coveragepy-result` (Optional)
- `colcon-mixin` (Optional)

**Note**: for Windows, ROS 2 currently only officially supports Visual Studio 2019, but GitHub has dropped support for the `windows-2019` runner image, so we can only use `windows-2022` (Visual Studio 2022) or newer.

## Overview

The action first assembles a workspace with the dependencies needed to build the source, then runs `colcon build` and `colcon test`.

Step by step, that process goes like this:

- `vcs import` the repo file(s) specified through the `vcs-repo-file-url` argument, if any (defaults to none)
- checkout the code under test in the workspace using `vcs`
- `rosdep install` for the workspace, to get its dependencies
- run `colcon build` (optionally limited to packages specified in `package-name`)
- run `colcon test` (optionally limited to packages specified in `package-name`; optionally [skipped](#Skip-tests))

This action requires targeting a ROS or ROS 2 distribution explicitly.
This is provided via the `target-ros1-distro` or `target-ros2-distro` inputs.
Either or both may be specified, if neither is provided an error will be raised.
This input is used to `source setup.sh` for any installed ROS binaries (e.g. installed using [`ros-tooling/setup-ros`](https://github.com/ros-tooling/setup-ros)), as well as used as an argument to `rosdep install`.

## Action Output

This action defines [an output variable](https://help.github.com/en/actions/building-actions/metadata-syntax-for-github-actions#outputs): `ros-workspace-directory-name`.
It contains the path to the root of the ROS workspace assembled by the action.

The variable value can be used to retrieve logs, binaries, etc. after the action completes.

## Usage

See [`action.yml`](action.yml) to get the list of inputs supported by this action.

[action-ros-ci-template](https://github.com/ros-tooling/action-ros-ci-template) offers a template for using `action-ros-ci`.

### Base Usage Patterns

In most cases, you will use one of the following two base patterns to set up your build environment and run your build.

The sections following cover more specific options.

#### Using a prepared base container image from `setup-ros-docker` (Recommended)

```yaml
jobs:
  build_and_test_ros2:
    runs-on: ubuntu-latest
    container:
      image: rostooling/setup-ros-docker:ubuntu-noble-latest
    steps:
      - name: Build and run tests
        uses: ros-tooling/action-ros-ci@v0.4
        with:
          package-name: my_package
          target-ros2-distro: jazzy
```


#### Using `ros-tooling/setup-ros` to prepare the environment

In this case, `action-ros-ci` will rely on `setup-ros` for installing ROS 2 development tools, instead of a base container.

```yaml
jobs:
  build_and_test_ros2:
    runs-on: ubuntu-latest
    steps:
      - uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: jazzy
      - uses: ros-tooling/action-ros-ci@v0.4
        with:
          package-name: my_package
          target-ros2-distro: jazzy
```

### Building ROS 2 core dependencies from source

In this case, `action-ros-ci` will build all necessary ROS 2 dependencies of `my_package` from source.

```yaml
  - name: Build and run tests
    uses: ros-tooling/action-ros-ci@v0.4
    with:
      package-name: my_package
      target-ros2-distro: jazzy
      vcs-repo-file-url: https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos
```

### Scheduling periodic builds

You may want to run a build periodically - for example to reveal flaky tests or reveal problems from updates in upstream dependencies.

Note in this example that we've specified `ref:` in the action, to build a non-default branch.
Without this setting, the repository default branch and most recent commit will be checked out.

```yaml
name: Jazzy Source Build
on:
  schedule:
    # At 00:00 on Sunday.
    - cron '0 0 * * 0'

jobs:
  jazzy_from_source:
    runs-on: ubuntu-latest
    container:
      image: rostooling/setup-ros-docker:ubuntu-noble-latest
    steps:
      - uses: ros-tooling/action-ros-ci@v0.4
        with:
          package-name: my_package
          ref: jazzy
          target-ros2-distro: jazzy
          vcs-repo-file-url: https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos
```

### Build with a custom `repos` or `rosinstall` file

You can specify your own repos file using the `vcs-repo-file-url` input.
You can also automatically generate your package's dependencies using the following workflow:

```yaml
  - uses: actions/checkout@v4
  - run: |
      rosinstall_generator <package-name> --rosdistro <target-distro> \
      --deps-only --deps --upstream-development > /tmp/deps.repos
  # Pass the file to the action
  - uses: ros-tooling/action-ros-ci@v0.4
    with:
      package-name: my_package
      target-ros2-distro: jazzy
      vcs-repo-file-url: /tmp/deps.repos
```

Note that the `actions/checkout` step is required when using a custom repos file from your repository.

### Build a ROS 1 workspace

Building a ROS 1 workspace works the same way.
Simply use `target-ros1-distro` instead of `target-ros2-distro`.

```yaml
jobs:
  build_and_test_ros1:
    runs-on: ubuntu-latest
    container:
      image: rostooling/setup-ros-docker:ubuntu-focal-latest
    steps:
      - uses: ros-tooling/action-ros-ci@v0.4
        with:
          package-name: my_package
          target-ros1-distro: noetic
```

### Skip tests

To skip tests and code coverage data processing, set the `skip-tests` option to `true`.

```yaml
  - uses: ros-tooling/action-ros-ci@v0.4
    with:
      package-name: my_package
      target-ros2-distro: jazzy
      skip-tests: true
```

### Use a `colcon` `defaults.yaml` file

To use a [`colcon` `defaults.yaml` file](https://colcon.readthedocs.io/en/released/user/configuration.html#defaults-yaml), provide a valid JSON string through the `colcon-defaults` input.
This allows using a `colcon` option/argument that is not exposed by this action's inputs.

```yaml
  - uses: ros-tooling/action-ros-ci@v0.4
    with:
      package-name: my_package
      target-ros2-distro: jazzy
      colcon-defaults: |
        {
          "build": {
            "cmake-args": [
                "-DMY_CUSTOM_OPTION=ON"
            ]
          }
        }
```

### Do not use `--symlink-install` when building

By default, [`--symlink-install` is used with `colcon build`](https://colcon.readthedocs.io/en/released/reference/verb/build.html).
To avoid this, set the `no-symlink-install` input to `true`.

```yaml
  - uses: ros-tooling/action-ros-ci@v0.4
    with:
      package-name: my_package
      target-ros2-distro: jazzy
      no-symlink-install: true
```

### Enable Address Sanitizer to automatically report memory issues

[ASan][addresssanitizer] is an open-source tool developed to automatically report
memory corruption bugs.

```yaml
  - uses: ros-tooling/action-ros-ci@v0.4
    with:
      colcon-defaults: |
        {
          "build": {
            "mixin": ["asan-gcc"]
          }
        }
      colcon-mixin-repository: https://raw.githubusercontent.com/colcon/colcon-mixin-repository/b8436aa16c0bdbc01081b12caa253cbf16e0fb82/index.yaml
      package-name: my_package
      target-ros2-distro: jazzy
```

To look for detected memory errors, check the build logs for entries containing `ERROR: AddressSanitizer`. Example:

```
==9442== ERROR: AddressSanitizer heap-use-after-free on address 0x7f7ddab8c084 at pc 0x403c8c bp 0x7fff87fb82d0 sp 0x7fff87fb82c8
```

ASan is analyzing memory issues at runtime. ASan diagnostic messages will be emitted by the package tests when they run.

### Generate and process code coverage data

#### Generate code coverage information using `lcov` and `colcon-lcov-result`

Generate code coverage information for C/C++ files using the appropriate mixins for `gcc`.
`action-ros-ci` uses [`colcon-lcov-result`](https://github.com/colcon/colcon-lcov-result) to aggregate generated coverage information.

Flags can be passed manually using, for instance, `extra-cmake-args`, but it is
preferable to use a `colcon` mixin (through [`colcon-defaults`](#Use-a-colcon-defaultsyaml-file)) to pass the appropriate flags automatically.

```yaml
  - uses: ros-tooling/action-ros-ci@v0.4
    with:
      package-name: my_package
      target-ros2-distro: jazzy
      colcon-defaults: |
        {
          "build": {
            "mixin": ["coverage-gcc"]
          }
        }
      # If possible, pin the repository in the workflow to a specific commit to avoid
      # changes in colcon-mixin-repository from breaking your tests.
      colcon-mixin-repository: https://raw.githubusercontent.com/colcon/colcon-mixin-repository/b8436aa16c0bdbc01081b12caa253cbf16e0fb82/index.yaml
```

#### Generate code coverage information using `coveragepy` and `colcon-coveragepy-result`

Generate code coverage information for Python files using the appropriate mixins.
`action-ros-ci` uses [`colcon-coveragepy-result`](https://github.com/colcon/colcon-coveragepy-result) to aggregate generated coverage information.

```yaml
  - uses: ros-tooling/action-ros-ci@v0.4
    with:
      package-name: my_package
      target-ros2-distro: jazzy
      colcon-defaults: |
        {
          "build": {
            "mixin": ["coverage-pytest"]
          },
          "test": {
            "mixin": ["coverage-pytest"]
          }
        }
      # If possible, pin the repository in the workflow to a specific commit to avoid
      # changes in colcon-mixin-repository from breaking your tests.
      colcon-mixin-repository: https://raw.githubusercontent.com/colcon/colcon-mixin-repository/b8436aa16c0bdbc01081b12caa253cbf16e0fb82/index.yaml
```

#### Integrate `action-ros-ci` with `codecov`

The generated code coverage information can be uploaded to [codecov.io](https://codecov.io/).

For a private repo, you will need to setup a secret `CODECOV_TOKEN` in [your repository settings][creating-encrypted-secrets].
See [`codecov/codecov-action`](https://github.com/codecov/codecov-action) documentation for more information about how to setup the action.

```yaml
  - uses: actions/checkout@v4
  - uses: ros-tooling/action-ros-ci@v0.4
    with:
      package-name: my_package
      target-ros2-distro: jazzy
      colcon-defaults: |
        {
          "build": {
            "mixin": ["coverage-gcc", "coverage-pytest"]
          },
          "test": {
            "mixin": ["coverage-pytest"]
          }
        }
      # If possible, pin the repository in the workflow to a specific commit to avoid
      # changes in colcon-mixin-repository from breaking your tests.
      colcon-mixin-repository: https://raw.githubusercontent.com/colcon/colcon-mixin-repository/b8436aa16c0bdbc01081b12caa253cbf16e0fb82/index.yaml
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

#### Disable code coverage results collection

By default, when tests are run, code coverage data is collected using [`colcon-lcov-result`](#generate-code-coverage-information-using-lcov-and-colcon-lcov-result) and [`colcon-coveragepy-result`](#generate-code-coverage-information-using-coveragepy-and-colcon-coveragepy-result).
To disable it, set `coverage-result` to `false`.

```yaml
  - uses: ros-tooling/action-ros-ci@v0.4
    with:
      # ...
      coverage-result: false
```

### Store `colcon` logs as build artifacts

GitHub workflows can persist data generated in workers during the build using [artifacts](persisting-workflow-data-using-artifacts). `action-ros-ci` generated colcon logs can be saved as follows:

```yaml
  - uses: ros-tooling/action-ros-ci@v0.4
    id: action_ros_ci_step
    with:
      package-name: ament_copyright
      target-ros2-distro: jazzy
  - uses: actions/upload-artifact@v1
    with:
      name: colcon-logs
      path: ${{ steps.action_ros_ci_step.outputs.ros-workspace-directory-name }}/log
    if: always() # upload the logs even when the build fails
```

### Use with private repos

`action-ros-ci` needs a token to be able to clone private repositories.
If the only private repository your workflow needs is the one against which it runs, using the default `GITHUB_TOKEN` will work.
However, if your workflow also clones other private repositories (e.g., repositories included in repos files provided through `vcs-repo-file-url`), you will need to generate a [personal access token](https://github.com/settings/tokens) (PAT) with the "repo" scope and add it to your repo's [secrets][creating-encrypted-secrets].
For example, if this secret is called `REPO_TOKEN`:

```yaml
  - uses: ros-tooling/action-ros-ci@v0.4
    with:
      package-name: my_package
      # If there are no private dependencies, use the default token, no need to create a PAT or add a secret
      import-token: ${{ secrets.GITHUB_TOKEN }}
      # If there are private dependencies (e.g., in a file provided through vcs-repo-file-url), a PAT is required
      import-token: ${{ secrets.REPO_TOKEN }}
```

Note that this currently only works for tokens for the GitHub server this action runs on.
For example, it will not work with a token for a private repo on github.com when the action is running on an enterprise GitHub server.

### Skip `rosdep install` and run `rosdep check`

Include an option to bypass `rosdep install` for workflow that uses specific docker image that is expected to already have all dependencies.
To check for missing dependencies within the workflow's image, user can run with `rosdep-check: true` flag.

```yaml
runs-on: ubuntu-latest
container:
  image: my-prepared-docker-image
steps:
  # ...
  - uses: ros-tooling/action-ros-ci@v0.4
    with:
      target-ros2-distro: jazzy
      package-name: ament_copyright
      vcs-repo-file-url: "https://raw.githubusercontent.com/ros2/ros2/release-jazzy-20240523/ros2.repos"
      skip-rosdep-install: true
      rosdep-check: true
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
