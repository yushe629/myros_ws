#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import message_filters
import math
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np

class gas_mapping:
    def __init__(self):

        rospy.init_node("gas_mapping")
        self.gas_map_pub= rospy.Publisher("/estimated_gas_map", OccupancyGrid, queue_size=1)
        self.gas_value_sub = rospy.Subscriber("/gas", Float32, self.gas_callback)
        # use tf
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        # 10 Hz
        self.rate = rospy.Rate(10.0)
        
        self.gas_value = None

        # gap map for visualization
        self.gas_visual_map = OccupancyGrid()
        self.gas_visual_map.header.frame_id = "map"
        self.gas_scale = rospy.get_param("~gas_visual_scale", 1.0)
        self.gas_offset = rospy.get_param("~gas_visual_offset", 0.0)
        self.gas_visual_map.info.resolution = rospy.get_param("~gas_visual_map_resolution", 0.05) # 0.05 m/pixel
        origin = rospy.get_param("~gas_visual_map_origin", [-10.0, -10.0, 0.0])
        print(origin)
        self.gas_visual_map.info.width = int(math.fabs(origin[0]) / self.gas_visual_map.info.resolution * 2)
        self.gas_visual_map.info.height = int(math.fabs(origin[1]) / self.gas_visual_map.info.resolution * 2)
        self.gas_visual_map.info.origin.position.x = origin[0]
        self.gas_visual_map.info.origin.position.y = origin[1]
        self.gas_visual_map.info.origin.position.z = origin[2]
        self.gas_visual_map.info.origin.orientation.w = 1
        self.gas_visual_map.data = [0] * self.gas_visual_map.info.width * self.gas_visual_map.info.height
        self.gas_map_pub_time = rospy.get_time()
        rospy.sleep(1.0)

        rospy.spin()
        # TODO How to use two or three subscribers

    def gas_callback(self,msg):
        # set Odometry.pose.pose
        # self.robot_pose = msg.pose.pose
        self.gas_value = msg.data
        try:
            t = self.tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
            #t = tfBuffer.lookup_transform('mug', 'base_link', rospy.Time())
            self.odom_callback(t.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                self.rate.sleep()
        
    def odom_callback(self, msg):
        # Add gas map date to gas_map_array
        if self.gas_value != None:
            # robot_pose = msg.pose.pose
            translation = msg.translation
            x = translation.x
            y = translation.y
            # visualize the gas map
            pixel_x = int((x - self.gas_visual_map.info.origin.position.x) / self.gas_visual_map.info.resolution)
            pixel_y = int((y - self.gas_visual_map.info.origin.position.y) / self.gas_visual_map.info.resolution)
            rospy.loginfo("x:%s, y:%s, px: %s, py: %s", x, y, pixel_x, pixel_y)
            self.gas_visual_map.data[pixel_y * self.gas_visual_map.info.width + pixel_x] = int(self.gas_value * self.gas_scale + self.gas_offset)
            if rospy.get_time() - self.gas_map_pub_time > 1.0: # publish in 1 Hz
                self.gas_map_pub.publish(self.gas_visual_map)
                self.gas_map_pub_time= rospy.get_time()

if __name__ == "__main__":
    try:
        gas_mapping_action = gas_mapping()
    except rospy.ROSInterruptException: pass
