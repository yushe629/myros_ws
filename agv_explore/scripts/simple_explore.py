#!/usr/bin/env python

import sys
import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler

inf_distance = 5.0
territory_radius = 0.6

class Roomba:
    def __init__(self):

        rospy.init_node("roomba")
        
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber("scan", LaserScan, self.callback)

        # set initial pose for AMCL
        # self.init_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size = 1)
        # sleep(1.0)
        # init_pose = PoseWithCovarianceStamped()
        # init_pose.header.stamp = rospy.Time.now()
        # init_pose.header.frame_id = "/map"
        # init_pose.pose.pose.x = rospy.get_param("initial_pose/x", 0)
        # init_pose.pose.pose.y = rospy.get_param("initial_pose/y", 0)
        # # RPY to convert to Quaternion
        # init_pose.pose.orientation = Quaternion(quaternion_from_euler(0, 0, rospy.get_param("initial_pose/theta", 0)))
        # self.init_pose_pub.publish(init_pose)
        

        rospy.spin()

    def callback(self, msg):
        rospy.loginfo_throttle(1.0, "laser number: %d", len(msg.ranges))
        

        #front_dist = msg.ranges[0]
        front_dists = msg.ranges[:20]+ msg.ranges[340:]
        front_dist = min(list(map(lambda x: inf_distance if x == float('inf') else x, front_dists)))
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
    except rospy.ROSInterruptException: pass

