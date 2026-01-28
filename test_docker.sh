#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="nautomatica-base:latest"
OVERVIEW_BUILD_SCRIPT="${OVERVIEW_BUILD_SCRIPT:-./build_overview_docker.sh}"
DEFAULT_TEST_LOG_DIR="/root/code/log/na_test_results"

run_docker_tests() {
  local image_name="${IMAGE_NAME}"
  local test_log_dir="${NA_TEST_LOG_DIR:-$DEFAULT_TEST_LOG_DIR}"
  local tty_flags=()
  if [[ -t 1 ]]; then
    tty_flags+=("-it")
  fi
  docker run "${tty_flags[@]}" --rm \
    -e "NA_TEST_LOG_DIR=${test_log_dir}" \
    -v "$(pwd)/:/root/code" \
    -w /root/code \
    "$image_name" \
    bash -lc 'source /root/code/common_scripts.sh && if [[ $# -eq 0 ]]; then ct; else colcon test "$@" && colcon test-result --verbose; fi' -- "$@"
}

run_overview_build() {
  if [[ -x "${OVERVIEW_BUILD_SCRIPT}" ]]; then
    "${OVERVIEW_BUILD_SCRIPT}"
  else
    echo "Skipping overview build (missing executable ${OVERVIEW_BUILD_SCRIPT})" >&2
  fi
}

run_overview_build
run_docker_tests "$@"
