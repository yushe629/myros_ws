#!/usr/bin/env python
import imp
import rospy
import message_filters
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from turtlebot3_explore.msg import PositionAndGas
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid
import numpy as np

class gas_roughly_search:
    def __init__(self):
        
        rospy.init_node("gas_roughly_search")
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.estimated_gas_map_sub = rospy.Subscriber("estimated_gas_map", self.callback)
        self.gas_visual_map.info.resolution = rospy.get_param("~gas_visual_map_resolution", 0.05) # 0.05 m/pixel
        origin = rospy.get_param("~gas_visual_map_origin", [-10.0, -10.0, 0.0])
        self.gas_visual_map.info.width = int(math.fabs(origin[0]) / self.gas_visual_map.info.resolnution * 2)
        self.gas_visual_map.info.height = int(math.fabs(origin[1]) / self.gas_visual_map.info.resolution * 2)
        self.gas_visual_map.info.origin.position.x = origin[0]
        self.gas_visual_map.info.origin.position.y = origin[1]
        self.gas_visual_map.info.origin.position.z = origin[2]
        self.gas_visual_map.info.origin.orientation.w = 1
        self.gas_visual_map.data = [0] * self.gas_visual_map.info.width * self.gas_visual_map.info.height

        
        self.execute = false

        
        rospy.spin()


    def callback(self, msg):
        if not self.execute:
            return
        mapdata = msg.data
        mapindex = mapdata.index(max(mapdata))
        target_x_pixel = mapindex % self.gas_visual_map.info.width
        target_y_pixel = mapindex / self.gas_visual_map.info.width
        target_x = target_x_pixel * self.gas_visual_map.info.resolution + self.gas_visual_map.info.origin.position.x
        target_y = target_y_pixel * self.gas_visual_map.info.resolution + self.gas_visual_map.info.origin.position.y
        target = PoseStamped()
        target.header.frame_id = "map"
        target.header.seq = target.header.seq + 1
        target.header.stamp = rospy.Time.now()
        target.pose.position.x = target_x
        target.pose.position.y = target_y
        target.pose.position.z = 0.0
        target.pose.orientation.x = 0.0
        target.pose.orientation.y = 0.0
        target.pose.orientation.z = 0.0
        target.pose.orientation.w = 1.0
        self.goal_pub.publish(target)

        
if __name__ == "__main__":
    try:
        gas_roughly_search_action = gas_roughly_search()
    except rospy.ROSInterruptException: pass
