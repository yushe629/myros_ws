## Localization & Navigation

### @ myagv (set uav as master )
$ roslaunch agv_explore aibot_bringup.launch livox_hw:=true with_uav:=true


### @ local
$ roslaunch agv_explore rviz_display.launch


## [Additional] Generate Existing Path Mapg

### @ myagv
$ rosrun agv_explore existing_path_generator
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py # keyboard
$ rosrun map_server map_saver -f xxx-existing-path-map map:=/path_map

## Patrol Demo

$ roslaunch agv_explore gas_patrol.launch plan_file_short_hand:=::.yaml # auto waypoint mission
