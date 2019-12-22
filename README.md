# action-ros-ci

[![GitHub Action Status](https://github.com/ros-tooling/action-ros-ci/workflows/Test%20action-ros-ci/badge.svg)](https://github.com/ros-tooling/action-ros-ci) [![Greenkeeper badge](https://badges.greenkeeper.io/ros-tooling/action-ros-ci.svg)](https://greenkeeper.io/)

This action compiles [ROS 2](https://index.ros.org/doc/ros/) from source, and run colon-test on the package under test.

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
