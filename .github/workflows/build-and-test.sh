#!/usr/bin/env bash
set -euxo pipefail

npm install -g codecov

# When running inside a Docker container, by default, we're root and all files belond to root.
# However, calling the npm scripts (build, etc.) with 'npm run ...' runs the commands as user
# ID=1001, which means we can't open or write to any files. Therefore, if we're in Docker, chown
# everything under /__w (which is the workspace directory on the host: /home/runner/work) to user
# ID=1001. See:
# * https://github.com/ros-tooling/setup-ros/pull/521
# * https://github.com/npm/cli/issues/4589
docker_workdir="/__w"
if [ -d "${docker_workdir}" ]; then
  chown -R 1001:1001 "${docker_workdir}"
fi

npm ci
npm run build
npm test

# Upload code coverage to CodeCov, but do not fail the CI if CodeCov upload
# fails as the service is sometimes flaky.
codecov -f ./coverage/coverage-final.json || true
