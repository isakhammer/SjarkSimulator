# ROS 2 Kilted Kaiju on Ubuntu Noble (24.04)
FROM ubuntu:24.04

# Use bash for RUN so we can use source, export, etc.
SHELL ["/bin/bash", "-c"]

# Set noninteractive front-end and locale env vars
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# 1) Locale + basic tools + enable universe
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        locales \
        software-properties-common \
        curl \
        tmux \
        vim \
        ca-certificates && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    rm -rf /var/lib/apt/lists/*

# 2) Install ros2-apt-source (adds ROS 2 Kilted repository)
RUN apt-get update && \
    apt-get install -y --no-install-recommends curl && \
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    rm -rf /var/lib/apt/lists/*

# 3) Install dev tools and ROS 2 Kilted (desktop variant)
#    For bare-bones instead of desktop, replace ros-kilted-desktop with ros-kilted-ros-base
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        ros-dev-tools \
        ros-kilted-desktop && \
    rm -rf /var/lib/apt/lists/*


RUN apt install python3-colcon-common-extensions

RUN apt-get update && apt-get install -y ros-kilted-turtlebot3 ros-kilted-turtlebot3-simulations
# ros-kilted-gazebo-ros-pkgs ros-kilted-gazebo-ros-control

RUN rosdep init && \
    rosdep update

# 4) Make every interactive shell source ROS and your workspace helpers
RUN echo "source /root/code/common_scripts.sh" >> /root/.bashrc
RUN . /root/.bashrc

WORKDIR /root/code
COPY . .

CMD ["bash"]
