
## Rviz

```bash
$ rviz -d `rospack find cooperative_patrol`/config/patrol.rviz
```

## Record

```bash
$ rosrun cooperative_patrol rosbag_record.sh
```

## Joy for tele-operation

```bash
$ roslaunch aerial_robot_base joy_stick.launch robot_name:=quadrotor
```

## Start Patrol

```bash
$ rostopic pub -1 /task_start std_msgs/Empty "{}" # or you can click the "PublihsTopic" in rviz
```
