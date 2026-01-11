FROM ros:humble AS builder

RUN apt-get update && \
    apt-get -y install git cmake build-essential wget file qtbase5-dev libqt5svg5-dev \
                       ros-humble-rviz2 libqtav-dev \
                       libqt5websockets5-dev libqt5opengl5-dev libqt5x11extras5-dev \
                       libprotoc-dev libzmq3-dev liblz4-dev libzstd-dev libmosquittopp-dev


WORKDIR /opt/ws_plotjuggler/src
RUN git clone https://github.com/PlotJuggler/plotjuggler_msgs.git
RUN git clone https://github.com/facontidavide/PlotJuggler.git
RUN git clone https://github.com/PlotJuggler/plotjuggler-ros-plugins.git

WORKDIR /opt/ws_plotjuggler
RUN rosdep update && rosdep install -y --from-paths src --ignore-src


SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && colcon build

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /opt/ws_plotjuggler/install/setup.sh" >> /root/.bashrc
RUN echo "source /root/code/common_scripts.sh" >> /root/.bashrc
WORKDIR /root/code
COPY . .




CMD ["bash"]
