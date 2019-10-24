# action-ros2-ci

[![GitHub Action Status](https://github.com/ros-tooling/action-ros2-ci/workflows/Test%20action-ros2-ci/badge.svg)](https://github.com/ros-tooling/action-ros2-ci) [![Greenkeeper badge](https://badges.greenkeeper.io/ros-tooling/action-ros2-ci.svg)](https://greenkeeper.io/)

This action compiles [ROS 2](https://index.ros.org/doc/ros2/) from source, and run colon-test on the package under test.

## Usage

See [action.yml](action.yml)

```yaml
steps:
- uses: ros-tooling/setup-ros2@master
- uses: ros-tooling/action-ros2-ci@master
  with:
    package-name: ament_copyright
```

## License

The scripts and documentation in this project are released under the [Apache 2](LICENSE)
