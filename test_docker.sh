#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="nautomatica-base:latest"
OVERVIEW_BUILD_SCRIPT="${OVERVIEW_BUILD_SCRIPT:-./build_overview_docker.sh}"

run_docker_tests() {
  local image_name="${IMAGE_NAME}"
  docker run -it --rm \
    -v "$(pwd)/:/root/code" \
    -w /root/code \
    "$image_name" \
    bash -lc 'source /root/code/common_scripts.sh && ct'
}

run_overview_build() {
  if [[ -x "${OVERVIEW_BUILD_SCRIPT}" ]]; then
    "${OVERVIEW_BUILD_SCRIPT}"
  else
    echo "Skipping overview build (missing executable ${OVERVIEW_BUILD_SCRIPT})" >&2
  fi
}

run_overview_build
run_docker_tests
