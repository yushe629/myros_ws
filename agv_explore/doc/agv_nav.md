## Localization & Navigation

### @ myagv (set uav as master )

$ roslaunch agv_explore aibot_bringup.launch livox_hw:=true with_uav:=true
$ roslaunch agv_explore patrol.launch waypoint_file:=::.yaml ## Patrol 


### [Additional] Generate Existing Path Map

$ rosrun agv_explore existing_path_generator
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py # keyboard
$ rosrun map_server map_saver -f xxx-existing-path-map map:=/path_map

