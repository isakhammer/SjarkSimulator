#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="${IMAGE_NAME:-nautomatic-overview:latest}"
SOURCE_PDF="latex/build/main.pdf"
OUTPUT_DIR="latex/output"
OUTPUT_PDF="${OUTPUT_DIR}/main.pdf"
ROOT_OUTPUT_PDF="${ROOT_OUTPUT_PDF:-/root/code/overview.pdf}"

docker build -f latex/Dockerfile -t "${IMAGE_NAME}" .
docker run --rm \
  -v "${PWD}:/workspace" \
  -w /workspace \
  --entrypoint bash \
  "${IMAGE_NAME}" \
  -c "mkdir -p latex/build && chown -R $(id -u):$(id -g) latex/build"
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
root_output_dir="$(dirname "${ROOT_OUTPUT_PDF}")"
if [[ -d "${root_output_dir}" ]]; then
  cp "${OUTPUT_PDF}" "${ROOT_OUTPUT_PDF}"
  echo "Built: ${ROOT_OUTPUT_PDF}"
else
  echo "Skipping copy: ${root_output_dir} does not exist (set ROOT_OUTPUT_PDF to override)." >&2
fi
