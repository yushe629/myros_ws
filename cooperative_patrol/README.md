# Demo of exploration using turtlebot3


## Usage

### Simulation

### SLAM demo

```bash
roslaunch cooperative_patrol bringup.launch use_teleop:=true
```

Use keyboard to control turtlebot3

### AMCL demo

```bash
roslaunch cooperative_patrol bringup.launch use_slam:=false use_teleop:=true initial_pose_x:=-2 initial_pose_y:=-0.5
```

Use keyboard to control turtlebot3




### Simple explore demo

```bash
roslaunch cooperative_patrol bringup.launch # bringup gazebo env
```

In another terminal:

```bash
rosrun cooperative_patrol simple_explore.py
```


### Gazebo

```bash
 roslaunch cooperative_patrol aibot_bringup.launch simulation:=true map_file:=/home/chou/ros/agv_ws/src/gas_sensing/cooperative_patrol/map/turtlebot3/turtlebot3_gazebo_world_map.yaml
```