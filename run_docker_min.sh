#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="nautomatica-base:latest"
XSOCK=/tmp/.X11-unix

USE_X11=0
if [[ -n "${DISPLAY:-}" && -d "$XSOCK" ]]; then
  USE_X11=1
  if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null 2>&1 || true
  fi
fi

docker_args=(
  -i
  --rm
  --privileged
  -v "$(pwd)/:/root/code"
  -v "$HOME/.ssh:/root/.ssh"
)

if [[ "$USE_X11" -eq 1 ]]; then
  docker_args+=(
    -e DISPLAY="$DISPLAY"
    -v "$XSOCK:$XSOCK"
  )
  if [[ -f "$HOME/.Xauthority" ]]; then
    docker_args+=(-v "$HOME/.Xauthority:/root/.Xauthority")
  fi
fi

docker run "${docker_args[@]}" "$IMAGE_NAME" "$@"
