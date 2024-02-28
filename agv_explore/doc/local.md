
## Rviz
$ rviz -d `rospack find agv_explore`/config/patrol.rviz

## Record
$ rosrun agv_explore rosbag_record.sh

## Start Patrol

$ rostopic pub -1 /task_start std_msgs/Empty "{}"
