#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-nautomatic-overview:latest}"

docker build -f latex/Dockerfile -t "${IMAGE_NAME}" .
docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "${PWD}:/workspace" \
  -w /workspace \
  "${IMAGE_NAME}"

echo "Built: latex/build/overview.pdf"

