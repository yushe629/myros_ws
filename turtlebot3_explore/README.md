# Demo of exploration using turtlebot3


## Usage

### Simulation

### SLAM demo

```bash
roslaunch turtlebot3_explore bringup.launch use_teleop:=true
```

Use keyboard to control turtlebot3

### AMCL demo

```bash
roslaunch turtlebot3_explore bringup.launch use_slam:=false use_teleop:=true initial_pose_x:=-2 initial_pose_y:=-0.5
```

Use keyboard to control turtlebot3




### Simple explore demo

```bash
roslaunch turtlebot3_explore bringup.launch # bringup gazebo env
```

In another terminal:

```bash
rosrun turtlebot3_explore simple_explore.py
```

