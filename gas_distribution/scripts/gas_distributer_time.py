#!/usr/bin/env python

import sys
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class GasDistributer:
    def __init__(self):

        rospy.init_node("gas_distributer_time")

        # gas source coordinate
        self.gas_origin = rospy.get_param("gas_origin", [0,0,0])
        self.gas_origin = np.array(self.gas_origin)
        # setting constant
        self.alpha = rospy.get_param("~alpha", 1.0)
        self.max_val = rospy.get_param("~gap_max_val", 100)
        # define publisher and subscriber
        self.gas_value_pub = rospy.Publisher("/gas", Float32, queue_size=10)
        self.robot_pose_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # setting rosTime
        self.start_time = rospy.Time.now()
        self.time_rate_off = 1.0
        self.time_rate = 1.0
        rospy.spin()

    def odom_callback(self, msg):
        rospy.logdebug_throttle(1.0, "robot odom")

        pos = msg.pose.pose.position
        pos = np.array([pos.x, pos.y, pos.z])
        # distribution rule:
        dist = np.linalg.norm(self.gas_origin - pos)
        val = self.max_val*math.exp(-dist**2) * (math.exp(- self.time_rate * (rospy.Time.now() - self.start_time).to_sec()) + self.time_rate_off)
        # val = self.max_val - self.alpha *  dist**2
        self.gas_value_pub.publish(Float32(val))


if __name__ == "__main__":
    try:
        gas_distributor = GasDistributer()
    except rospy.ROSInterruptException: pass
