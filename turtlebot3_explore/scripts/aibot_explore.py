#!/usr/bin/env python

import sys
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

inf_distance = 5.0
# for aibot changing param
territory_radius = 1.0
half_of_scan_size = 30

class Roomba:
    def __init__(self):

        rospy.init_node("roomba")
        
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.callback)

        rospy.spin()

    def callback(self, msg):
        size = len(msg.ranges)
        rospy.loginfo_throttle(1.0, "laser number: %d", size)
        
        
        #front_dist = msg.ranges[0]
        front_dists = msg.ranges[:half_of_scan_size]+ msg.ranges[(size - half_of_scan_size):]
        front_dist = min(list(map(lambda x: inf_distance if (x == float('inf') or x == 0.0
) else x, front_dists)))
        rospy.loginfo_throttle(1.0, "minimum front distance: %f", front_dist)
        
        # 180 (back):  msg.ranges[len(msg.ranges)/2]

        ## TODO: please implement roomba action
        if (front_dist < territory_radius):
            cmd_x = 0.0
            cmd_yaw = 0.5
        else:
            cmd_x = 0.3
            cmd_yaw = 0.0

        
        cmd_msg = Twist()
        cmd_msg.linear.x = cmd_x
        cmd_msg.angular.z = cmd_yaw
        self.vel_pub.publish(cmd_msg)

if __name__ == "__main__":
    try:
        roomba_action = Roomba()
    except rospy.ROSInterruptException:
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        rospy.Publisher("cmd_vel", Twist, queue_size=10).publish(stop_msg)

