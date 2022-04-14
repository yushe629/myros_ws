# Repository for auto gas  sensing by robots

## Pakcages

- turtlebot3_explore

## Build

```
mkdir ~/ros/gas_sensing_ws -p
cd ~/ros/gas_sensing_ws
wstool init src
wstool set -u -t src gas_sensing https://github.com/ut-hnl-lab/gas_sensing.git --git
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin b
```
