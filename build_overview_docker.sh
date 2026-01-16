#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-nautomatic-overview:latest}"
SOURCE_PDF="latex/build/main.pdf"
OUTPUT_DIR="latex/output"
OUTPUT_PDF="${OUTPUT_DIR}/main.pdf"

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

mkdir -p "${OUTPUT_DIR}"
cp "${SOURCE_PDF}" "${OUTPUT_PDF}"
echo "Built: ${OUTPUT_PDF}"
