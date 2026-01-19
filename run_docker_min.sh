#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="nautomatica-base:latest"
XSOCK=/tmp/.X11-unix

xhost +local:root

docker run -i --rm \
  -e DISPLAY="$DISPLAY" \
  --privileged \
  -v "$(pwd)/:/root/code" \
  -v "$XSOCK:$XSOCK" \
  -v "$HOME/.ssh:/root/.ssh" \
  -v "$HOME/.Xauthority:/root/.Xauthority" \
  "$IMAGE_NAME" "$@"
