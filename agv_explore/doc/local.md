
## Rviz
$ rviz -d `rospack find agv_explore`/config/patrol.rviz

## Record
$ rosrun agv_explore rosbag_record.sh

## Joy for tele-operation

$ roslaunch aerial_robot_base joy_stick.launch robot_name:=quadrotor

## Start Patrol

$ rostopic pub -1 /task_start std_msgs/Empty "{}" # or you can click the "PublihsTopic" in rviz
