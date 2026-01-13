#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="nautomatica-base:latest"

run_docker_tests() {
  local image_name="${IMAGE_NAME}"
  docker run -it --rm \
    -v "$(pwd)/:/root/code" \
    -w /root/code \
    "$image_name" \
    bash -lc 'source /root/code/common_scripts.sh && ct'
}

run_docker_tests
