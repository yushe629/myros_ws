#!/usr/bin/env python
import rospy
import message_filters
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from turtlebot3_explore.msg import PositionAndGas

# if scan_val is Inf, inf_distance is assigned.
inf_distance = 5.0
# the radius of the robot explore area
territory_radius = 0.6
# the velocity of exploring and the type of explore state
explore_vel = 0.3
explore_time = 0.3
explore_state = ['front', 'left', 'rear', 'right']

class gas_explore:
    def __init__(self):

        rospy.init_node("gas_explore")
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback, callback_args=0)
        # self.gas_value_sub = rospy.Subscriber("/gas", Float32, self.callback, callback_args=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.gas_value_sub = rospy.Subscriber("/gas", Float32, self.gas_callback)
        # self.mf = message_filters.ApproximateTimeSynchronizer([self.scan_sub, self.gas_value_sub], queue_size=10, deley =1/100. * 0.5)
        # self.mf.registerCallback(self.callback)
        # self.maxConcentrate = 0
        self.before_gas_value = 0.0
        self.gas_value = 0.0
        self.cmd_x = 0.0
        self.cmd_y = 0.0
        self.cmd_yaw = 0.0
        self.explore_state = 'front'
        self.iscorrect_path = True
        rospy.spin()
        # TODO How to use two or three subscribers

    def gas_callback(self, msg):
        self.before_gas_value = self.gas_value
        self.gas_value = msg

    def explore(self):
        if self.iscorrect_path:
            if self.explore_state == 'front':
                    self.cmd_x = explore_vel
            elif self.explore_state == 'left':
                    self.cmd_y = -1*explore_vel
            elif self.explore_state == 'rear':
                    self.cmd_x= -1*explore_vel
            elif self.explore_state == 'right':
                    self.cmd_y = explore_vel
            self.iscorrect_path = False
        else:
            rospy.sleep(explore_time)
            if self.explore_state == 'front':
                    self.cmd_x = -1*explore_vel
                    self.explore_state='left'
            elif self.explore_state == 'left':
                    self.cmd_y = explore_vel
                    self.explore_state='rear'
            elif self.explore_state == 'rear':
                    self.cmd_x= explore_vel
                    self.explore_state='right'
            elif self.explore_state == 'right':
                    self.cmd_y = -1*explore_vel
                    self.explore_state='front'

    def callback(self, msg):
        rospy.loginfo_throttle(1.0, "laser number: %d", len(msg.ranges))

        #front_dist = msg.ranges[0]
        front_dists = msg.ranges[:20]+ msg.ranges[340:]
        front_dist = min(list(map(lambda x: inf_distance if x == float('inf') else x, front_dists)))
        rospy.loginfo_throttle(1.0, "minimum front distance: %f", front_dist)

        if self.before_gas_value < self.gas_value:
            self.iscorrect_path = True
            self.explore_state = 'front'

        # 180 (back):  msg.ranges[len(msg.ranges)/2]

        ## TODO: please implement gas_explore action
        if (front_dist < territory_radius and self.iscorrect_path):
            self.cmd_x = 0.0
            self.cmd_yaw = 0.5
        else:
            self.explore()


        cmd_msg = Twist()
        cmd_msg.linear.x = self.cmd_x
        cmd_msg.linear.y = self.cmd_y
        cmd_msg.angular.z =self.cmd_yaw
        self.vel_pub.publish(cmd_msg)

if __name__ == "__main__":
    try:
        gas_explore_action = gas_explore()
    except rospy.ROSInterruptException: pass

