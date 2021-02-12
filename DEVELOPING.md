# Developing action-ros-ci

## Build and test

```
# install dependencies
npm install
# autoformat sources to meet enforced linter style
npm run fixup
# generate build artifacts
npm run build
```

Make sure to run this every time you update a Pull Request, it is necessary to check in the generated `index.js` from the build.

## Releasing

NOTE: This action is not yet determined stable enough to call 1.0, therefore the following information applies to minor releases.

This repository makes two types of tags for releases:

* static patch-level tags per release
  * `0.MINOR.PATCH`: e.g `0.1.4`
* dynamic API-level tags that move with patch releases so that users can get bugfixes without changing anything
  * `v0.MINOR`: e.g. `v0.1`

Release process
1. Create and push new release tag e.g. `0.1.4`
    * `git tag 0.1.4`
    * `git push origin 0.1.4`
1. Update or create the corresponding minor version tag. DON'T PUSH THIS YET
    * `git tag -f v0.1`
1. Create a new release and publish it to the marketplace via https://github.com/ros-tooling/action-ros-ci/releases/new using the patch-level tag that you created
1. Push the minor version tag now that the release is officially out. Users will get the new version automatically
    * `git push -f origin v0.1`
