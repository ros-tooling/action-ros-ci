#!/usr/bin/env bash
set -euxo pipefail

npm install -g codecov

npm ci
npm run build
npm test

# Upload code coverage to CodeCov, but do not fail the CI if CodeCov upload
# fails as the service is sometimes flaky.
codecov -f ./coverage/coverage-final.json || true
