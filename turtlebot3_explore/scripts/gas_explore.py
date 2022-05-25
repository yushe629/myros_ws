#!/usr/bin/env python
import rospy
import message_filters
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from turtlebot3_explore.msg import PositionAndGas
from nav_msgs.msg import Odometry

# if scan_val is Inf, inf_distance is assigned.
inf_distance = 5.0
# the radius of the robot explore area
territory_radius = 0.6
# the velocity of exploring and the type of explore state
explore_vel = 0.3
explore_time = 0.3
explore_yaw_vel = 1.0
explore_yaw_time = 0.3
explore_state = ['front', 'back', 'turn', 'after_turn', 'explored']
limit_time = 30

class gas_explore:
    def __init__(self):

        rospy.init_node("gas_explore")
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        # self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback, callback_args=0)
        # self.gas_value_sub = rospy.Subscriber("/gas", Float32, self.callback, callback_args=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.gas_value_sub = rospy.Subscriber("/gas", Float32, self.gas_callback)
        self.robot_pose_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # self.mf = message_filters.ApproximateTimeSynchronizer([self.scan_sub, self.gas_value_sub], queue_size=10, deley =1/100. * 0.5)
        # self.mf.registerCallback(self.callback)
        # self.maxConcentrate = 0
        self.before_gas_value = 0.0
        self.gas_value = 0.0
        self.robot_position
        self.cmd_x = 0.0
        self.cmd_yaw = 0.0
        self.explore_state = 'front'
        self.iscorrect_path = True
        self.maxValue = 0.0
        self.setMaxValueTime = rospy.Time.now()
        self.setMaxValueOdom
        rospy.spin()
        # TODO How to use two or three subscribers

    def odom_callback(self,msg):
        # set Odometry.pose.pose.position
        self.robot_position = msg.pose.pose.position
        
    def gas_callback(self, msg):
        self.before_gas_value = self.gas_value
        self.gas_value = msg
        if msg > self.maxValue:
            self.maxValue = msg
            self.setMaxValueTime = rospy.Time.now()
            self.maxvalueOdom = self.robot_position
            
    def explore(self):
        if self.explore_state =='front':
            self.cmd_x = explore_vel
            self.cmd_yaw = 0.0
            self.explore_state = 'back'
        elif self.explore_state == 'back':
            rospy.sleep(explore_time)
            self.cmd_x = -1.0*explore_vel
            self.explore_state = 'turn'
        elif self.explore_state == 'turn':
            rospy.sleep(explore_time)
            self.cmd_x = 0.0
            self.cmd_yaw = explore_yaw_vel
            self.explore_state = 'after_turn'
        elif self.explore_state == 'after_turn':
            rospy.sleep(explore_yaw_time)
            self.cmd_x = 0.0
            self.cmd_yaw = 0.0
            self.explore_state == 'front'

    def callback(self, msg):
        rospy.loginfo_throttle(1.0, "laser number: %d", len(msg.ranges))
        if ((self.setMaxValueTime - rospy.Time.now()).to_sec() > limit_time):
            self.cmd_x = 0.0
            self.cmd_yaw = 0.0
            # TODO: set goal as self.maxValueOdom
            # use diff or move_base
        #front_dist = msg.ranges[0]
        front_dists = msg.ranges[:20]+ msg.ranges[340:]
        front_dist = min(list(map(lambda x: inf_distance if x == float('inf') else x, front_dists)))
        rospy.loginfo_throttle(1.0, "minimum front distance: %f", front_dist)
        if self.before_gas_value < self.gas_value:
            self.explore_state = 'front'

        # 180 (back):  msg.ranges[len(msg.ranges)/2]

        ## TODO: please implement gas_explore action
        if (front_dist < territory_radius):
            self.cmd_x = 0.0
            self.cmd_yaw = 0.5
            self.explore_state = 'front'
            rospy.loginfo_throttle(1.0, 'state: avoiding obstacle',)
        else:
            self.cmd_yaw = 0.0
            self.cmd_x = 0.0
            self.explore()
            rospy.loginfo_throttle(1.0, 'state: %s', self.explore_state)


        cmd_msg = Twist()
        cmd_msg.linear.x = self.cmd_x
        cmd_msg.angular.z =self.cmd_yaw
        rospy.loginfo_throttle(1.0, "x: %f", self.cmd_x)
        rospy.loginfo_throttle(1.0, "yaw: %f", self.cmd_yaw)
        self.vel_pub.publish(cmd_msg)

if __name__ == "__main__":
    try:
        gas_explore_action = gas_explore()
    except rospy.ROSInterruptException: pass

