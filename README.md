# Repository for auto gas  sensing by robots

## Setup

```
source /opt/ros/${ROS_DISTRO}/setup.bash # please replace ${ROS_DISTRO} with your specific env variable, e.g., noetic
mkdir ~/ros/agv_uav_explore_ws -p
cd ~/ro/agv_uav_explore
sudo rosdep init # if necessary
rosdep update # if necessary
wstool init src
wstool set -u -t src agv_uav_explore https://github.com/ut-dragon-lab/agv_uav_explore.git --git
wstool merge -t src src/agv_uav_explore/myagv.rosinstall
wstool set -u -t src jsk_aerial_robot http://github.com/jsk-ros-pkg/jsk_aerial_robot --git # if necessary
wstool merge -t src src/jsk_aerial_robot/aerial_robot_${ROS_DISTRO}.rosinstall # if necessary
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin b
```
