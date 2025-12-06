source /opt/ros/kilted/setup.bash
source /root/code/src/install/setup.bash

alias so='source /root/.bashrc'
alias colb='colcon build --symlink-install'
alias cb='colcon build'
alias cbb='cd /root/code/src/ && colcon build && so'

alias colc='colcon build --packages-select cpp_pubsub'
alias install_setup='. install/setup.bash' # do research when to do this.

alias makepkg='ros2 pkg create --build-type ament_cmake' # see https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

alias dep='rosdep install -i --from-path src --rosdistro kilted -y' # checking dependencies

alias t='tmux -f /root/code/tmux.conf'
