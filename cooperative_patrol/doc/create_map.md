## Preparation

### branchs:

- livox_ros_driver: git@github.com:tongtybj/livox_ros_driver2 develop/pub_topic
- fast_lio: git@github.com:tongtybj/FAST_LIO develop/convert_map

#### myagv

####  uav or livox vim4


## Create Map Point Cloud

### step1 @ myagv 

```bash
$ roslaunch cooperative_patrol myagv_livox_hw.launch
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### step2 @ uav or livox vim4 

```bash
$ roslaunch fast_lio mapping_mid360.launch rviz:=false livox_driver:=true
```

### step3 @ myagv 

```bash
$ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

- use keyobard to move the agv and crate the 3D map


### step4 @ local machine

- close the launch will generate .pcd file under `fast_lio/PCD`, please rename the .pcd file for your usage

- transform the origin frame of pointcloud:

```bash
$ rosrun tf static_transform_publisher 0 0 0.18 0 0.005 -0.01 world map 1 # correct the origin frame
$ rosrun fast_lio save_pointcloud input:=/cloud_pcd _target_frame:=world # save the pointcloud to .pcd file with the target frame
```

## step5 @ local machine or vim4

- generate 2D Map for navigation

```bash
$ roslaunch fast_lio pcd2map.launch file_name:=`rospack find fast_lio`/PCD/8-326_scans.pcd min_height:=0.05 max_height:=0.5 map_name:=map-name width:=10.0 height:=10.0
```

*note1*: `width` and `height` effect the size of the existing path map, so please set to a sufficient size, and the values should be identical right now
*note2*: `min_height` and `max_height` is calculated from the ground level


## step6 @ myagv

- copy 2D map file under to {TODO}


