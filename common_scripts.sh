
source /opt/ros/kilted/setup.bash
source /root/code/install/setup.bash

alias so='source /root/.bashrc'
alias colb='colcon build --symlink-install'
# alias cb='colcon build'
alias pyfast='pytest src --import-mode=importlib --ignore=src/na_launch/test --ignore=src/na_sim/test --ignore=src/na_planner/test --ignore=src/na_viz/test'
alias cbb='cd /root/code/ && pyfast && colcon build && so'


alias colc='colcon build --packages-select cpp_pubsub'
alias install_setup='. install/setup.bash' # do research when to do this.

alias lc='ros2 launch na_launch controller_launch.py'
alias lsc='ros2 launch na_launch sim_controller_launch.py'

alias makepkg='ros2 pkg create --build-type ament_cmake' # see https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

alias dep='rosdep install -i --from-path src --rosdistro kilted -y' # checking dependencies

alias t='tmux -f /root/code/tmux.conf'

# # Enable color prompt if possible
# if [ -x /usr/bin/tput ] && tput setaf 1 >/dev/null 2>&1; then
#     PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
# fi
