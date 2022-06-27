#!/usr/bin/env python
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32

# gas_visual_map = OccupancyGrid()
# gas_visual_map.info.resolution = 0.05
# gas_scale = 1.0
# gas_offset = 0.0
# gas_origin = [0, 0, 0]
# origin = [-10.0, -10.0, 0.0]
# max_vel = 100
# # setting rosTime
# start_time = 0
# time_rate_off = 1.0
# time_rate = 1.0
# dist_rate = 10.0

def GasGlobalMapping():
    rospy.init_node("gas_global_mapping")

    # gas source coordinate
    gas_origin = rospy.get_param("~gas_origin", [2.0,0.5,0.0])
    gas_origin = np.array( gas_origin)
    # setting constant
    max_val = rospy.get_param("~gap_max_val", 100)
    # define publisher and subscriber
    gas_map_pub = rospy.Publisher("/true_gas_map", OccupancyGrid, queue_size=10)
    # gas map for visualization
    gas_visual_map.header.frame_id = "map"
    gas_scale = rospy.get_param("~gas_visual_scale", 1.0)
    gas_offset = rospy.get_param("~gas_visual_offset", 0.0)
    gas_visual_map = OccupancyGrid()
    gas_visual_map.info.resolution = rospy.get_param("~gas_visual_map_resolution", 0.05) # 0.05 m/pixel
    origin = rospy.get_param("~gas_visual_map_origin", [-10.0, -10.0, 0.0])
    print(origin)
    gas_visual_map.info.width = int(math.fabs(origin[0]) /  gas_visual_map.info.resolution * 2)
    gas_visual_map.info.height = int(math.fabs(origin[1]) /  gas_visual_map.info.resolution * 2)
    gas_visual_map.info.origin.position.x = origin[0]
    gas_visual_map.info.origin.position.y = origin[1]
    gas_visual_map.info.origin.position.z = origin[2]
    gas_visual_map.info.origin.orientation.w = 1
    gas_visual_map.data = [0] *  gas_visual_map.info.width *  gas_visual_map.info.height
    start_time = rospy.get_time()

    def calc_gas_value(pos):
        # pos is 3d_array
        dist = np.linalg.norm(gas_origin - pos)
        del_time = rospy.get_time() -  start_time
        val =  max_val*math.exp(-dist**2)*math.exp(- time_rate/(del_time +  time_rate_off))
        return val

    def map_gen():
        width =  gas_visual_map.info.width
        height =  gas_visual_map.info.height
        for i in range(width*height):
            x = i % width *  gas_visual_map.info.resolution +  gas_visual_map.info.origin.position.x
            y = i / width *  gas_visual_map.info.resolution +  gas_visual_map.info.origin.position.y
            half =  gas_visual_map.info.resolution / 2
            value =  calc_gas_value([x + half, y + half, 0])
            gas_visual_map.data[i] = int(value* gas_scale +  gas_offset)

    while not rospy.is_shutdown():
        map_gen()
        gas_map_pub.publish(gas_visual_map)
        rospy.Rate(10).sleep


if __name__ == "__main__":
    try:
        GasGlobalMapping()
    except rospy.ROSInterruptException: pass




















