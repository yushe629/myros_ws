#!/usr/bin/env python
import imp
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
territory_radius = 0.6
# the velocity of exploring and the type of explore state
explore_vel = 0.3
explore_time = 0.3
explore_yaw_vel = 1.0
explore_yaw_time = 0.3
explore_state = ['front', 'back', 'turn', 'after_turn', 'explored']
limit_time = 30.0

class gas_mapping:
    def __init__(self):

        rospy.init_node("gas_mapping")
        self.gas_map_pub= rospy.Publisher("/estimated_gas_map", OccupancyGrid, queue_size=1)
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

        rospy.sleep(1.0)
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

if __name__ == "__main__":
    try:
        gas_mapping_action = gas_mapping()
    except rospy.ROSInterruptException: pass
