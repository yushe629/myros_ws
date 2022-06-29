#!/usr/bin/env python

import rospy
import message_filters
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid
import numpy as np

# if scan_val is Inf, inf_distance is assigned.
inf_distance = 5.0
# the radius of the robot explore area
territory_radius = 1.0
half_of_scan_size = 30
# the velocity of exploring and the type of explore state
explore_vel = 0.3
explore_time = 0.5
explore_yaw_vel = 2.0
explore_yaw_time = 1.0
explore_state = ['front', 'back', 'turn', 'after_turn', 'explored']
limit_time = 30

class gas_explore:
    def __init__(self):

        rospy.init_node("gas_explore")
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.gas_map_pub= rospy.Publisher("/estimated_gas_map", OccupancyGrid, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.gas_value_sub = rospy.Subscriber("/gas", Float32, self.gas_callback)
        self.robot_pose_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        # self.mf = message_filters.ApproximateTimeSynchronizer([self.scan_sub, self.gas_value_sub], queue_size=10, deley =1/100. * 0.5)
        # self.mf.registerCallback(self.callback)
        self.before_gas_value = 0.0
        self.gas_value = 0.0
        self.max_gas_value = 0.0
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.robot_pose = None
        self.cmd_x = 0.0
        self.cmd_yaw = 0.0
        self.explore_state = 'front'
        self.iscorrect_path = True
        self.max_gas_value_time = rospy.Time()
        self.begin_time = rospy.Time.now()
        self.max_gas_value_pose = None

        # gas map for exploreation rule
        self.gas_map_array = np.empty((0,3), np.float32)
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

        rospy.spin()
        # TODO How to use two or three subscribers

    def time_now_in_node(self):
        t = rospy.Time.now()
        return t - self.begin_time

    def odom_callback(self,msg):
        # set Odometry.pose.pose
        self.robot_pose = msg.pose.pose

    def gas_callback(self, msg):
        self.before_gas_value = self.gas_value
        self.gas_value = msg.data
        rospy.loginfo("value: %s", msg)
        # Add gas map date to gas_map_array
        if self.robot_pose != None:
            self.gas_map_array = np.append(self.gas_map_array, [[self.robot_pose.position.x, self.robot_pose.position.y, msg.data]], axis=0)

            # visualize the gas map
            pixel_x = int((self.robot_pose.position.x - self.gas_visual_map.info.origin.position.x) / self.gas_visual_map.info.resolution)
            pixel_y = int((self.robot_pose.position.y - self.gas_visual_map.info.origin.position.y) / self.gas_visual_map.info.resolution)
            self.gas_visual_map.data[pixel_y * self.gas_visual_map.info.width + pixel_x] = int(msg.data * self.gas_scale + self.gas_offset)

            if rospy.get_time() - self.gas_map_pub_time > 1.0: # publish in 1 Hz
                self.gas_map_pub.publish(self.gas_visual_map)
                self.gas_map_pub_time= rospy.get_time()

        if msg.data > self.max_gas_value:
            self.max_gas_value = msg.data
            self.max_gas_value_time = self.time_now_in_node()
            self.max_gas_value_pose = self.robot_pose

    def explore(self):
        if self.explore_state == 'front':
            rospy.sleep(explore_time)
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
            self.explore_state = 'front'

    def callback(self, msg):

        if self.explore_state == "explored":
            return

        # rospy.loginfo_throttle(1.0, "laser number: %d", len(msg.ranges))
        last_sec = (self.time_now_in_node().to_sec() - self.max_gas_value_time.to_sec())
        rospy.loginfo_throttle(1.0,"last_time: %f",last_sec)
        if last_sec  > limit_time:
            rospy.logwarn_once("Robot discovered goal!")
            self.explore_state = "explored"
            self.cmd_x = 0.0
            self.cmd_yaw = 0.0
            self.goal_pose.header.seq = self.goal_pose.header.seq + 1
            self.goal_pose.header.stamp = rospy.Time.now()
            self.goal_pose.pose = self.robot_pose
            self.goal_pub.publish(self.goal_pose)
            return

        size = len(msg.ranges)
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

        # rospy.loginfo_throttle(1.0, "minimum front distance: %f", front_dist)
        if self.before_gas_value < self.gas_value:
            self.explore_state = 'front'

        # 180 (back):  msg.ranges[len(msg.ranges)/2]

        if (front_dist < territory_radius):
            self.cmd_x = 0.0
            self.cmd_yaw = 0.5
            self.explore_state = 'front'
            # rospy.loginfo_throttle(1.0, 'state: avoiding obstacle')
        else:
            self.cmd_yaw = 0.0
            self.cmd_x = 0.0
            # rospy.loginfo_throttle(1.0, 'state: %s', self.explore_state)
            self.explore()


        cmd_msg = Twist()
        cmd_msg.linear.x = self.cmd_x
        cmd_msg.angular.z =self.cmd_yaw
        self.vel_pub.publish(cmd_msg)

if __name__ == "__main__":
    try:
        gas_explore_action = gas_explore()
    except rospy.ROSInterruptException: pass
