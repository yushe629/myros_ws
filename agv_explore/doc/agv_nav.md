## Localization & Navigation

### repository:

#### Base: 
- dyamixel-workbench:  git@github.com:tongtybj/dynamixel-workbench.git develop/current_based_position_control
- gas_sensing: git@github.com:tongtybj/gas_sensing.git develop/refactor

#### AGV

- aibot:
  -- aibot_hw: https://github.com/xmhuaway/aibot_description master
  -- aibot_description: https://github.com/xmhuaway/aibot_description master

### @ myagv (set uav as master )

$ roslaunch agv_explore aibot_bringup.launch livox_hw:=true with_uav:=true
$ roslaunch agv_explore patrol.launch waypoint_file:=::.yaml ## Patrol 


### [Additional] Generate Existing Path Map

$ rosrun agv_explore existing_path_generator
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py # keyboard
$ rosrun map_server map_saver -f xxx-existing-path-map map:=/path_map

