#!/bin/bash

rosbag record /patrol_smach_server/smach/container_init /patrol_smach_server/smach/container_status /patrol_smach_server/smach/container_structure /tf  /tf_static /map /livox/scan /move_base/DWAPlannerROS/global_plan /move_base/DWAPlannerROS/local_plan /move_base/GlobalPlanner/plan /move_base/global_costmap/costmap /move_base/local_costmap/costmap /quadrotor/target_pose /camera/image_raw/compressed /move_base/DWAPlannerROS/global_plan /move_base/DWAPlannerROS/local_plan /move_base/GlobalPlanner/plan /move_base/current_goal /move_base/goal /move_base/result
