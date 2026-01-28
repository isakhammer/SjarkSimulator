#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-sjarksimulator-overview:latest}"
SOURCE_PDF="latex/main.pdf"

docker build -f latex/Dockerfile -t "${IMAGE_NAME}" .
docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "${PWD}:/workspace" \
  -w /workspace \
  "${IMAGE_NAME}"

if [[ ! -f "${SOURCE_PDF}" ]]; then
  echo "Build failed: ${SOURCE_PDF} not found." >&2
  exit 1
fi

echo "Built: ${SOURCE_PDF}"
