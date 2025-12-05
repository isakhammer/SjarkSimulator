
IMAGE_NAME="ros2-kilted-base:latest"

function run_docker()  {
  image_name=$IMAGE_NAME
  xhost +local:root
  XSOCK=/tmp/.X11-unix

  docker run -it --rm \
     -e DISPLAY=$DISPLAY \
     --privileged \
     -v $(pwd)/:/root/code \
     -v $XSOCK:$XSOCK \
     -v $HOME/.ssh:/root/.ssh \
     -v $HOME/.Xauthority:/root/.Xauthority \
     $image_name "$@"
}

run_docker

# docker run -it --rm -v "$(pwd)":/ws ros2-kilted-base
