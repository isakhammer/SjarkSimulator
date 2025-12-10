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
        gnupg \
        xterm \
        lsb-release \
        ca-certificates && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    rm -rf /var/lib/apt/lists/*

ENV TERM=xterm-256color
ENV COLORTERM=truecolor

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
# Set ROS 2 distro
ARG ROS_DISTRO=jazzy
ENV ROS_DISTRO=${ROS_DISTRO}

# 3) Install dev tools, ROS 2 Jazzy desktop, colcon, and TurtleBot3
#    For bare-bones instead of desktop, replace ros-${ROS_DISTRO}-desktop with ros-${ROS_DISTRO}-ros-base
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y --no-install-recommends \
        ros-dev-tools \
        ros-${ROS_DISTRO}-desktop \
        python3-colcon-common-extensions \
        ros-${ROS_DISTRO}-turtlebot3 \
        ros-${ROS_DISTRO}-turtlebot3-simulations && \
    rm -rf /var/lib/apt/lists/*



# Add Gazebo (gz-harmonic) repository and install
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) \
        signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
        https://packages.osrfoundation.org/gazebo/ubuntu-stable \
        $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
        > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends gz-harmonic && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
    rosdep update


# 4) Make every interactive shell source ROS and your workspace helpers
RUN echo "source /root/code/common_scripts.sh" >> /root/.bashrc
RUN . /root/.bashrc

WORKDIR /root/code
COPY . .

CMD ["bash"]
