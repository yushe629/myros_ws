#!/usr/bin/env python

import sys
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Roomba:
    def __init__(self):

        rospy.init_node("roomba")

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.callback)

        rospy.spin()

    def callback(self, msg):
        rospy.loginfo_throttle(1.0, "laser number: %d", len(msg.ranges))
        

        front_dist = msg.ranges[0]
        
        rospy.loginfo_throttle(1.0, "front distance: %f", front_dist)
        
        # 180 (back):  msg.ranges[len(msg.ranges)/2]

        ## TODO: please implement roomba action
        if (front_dist < 1.0):
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
    except rospy.ROSInterruptException: pass

