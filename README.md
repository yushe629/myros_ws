# Repository for auto gap sensing by robots

## Pakcages

- turtlebot3_explore

## Build

```
mkdir ~/ros/gap_sensing_ws -p
cd ~/ros/gap_sensing_ws
wstool init src
wstool set -u -t src gap_sensing https://github.com/tongtybj/gap_sensing.git --git
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin b
```
