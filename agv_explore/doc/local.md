
## Rviz
$ rviz -d `rospack find agv_explore`/config/patrol.rviz

## Start Patrol

$ rostopic pub -1 /task_start std_msgs/Empty "{}"
