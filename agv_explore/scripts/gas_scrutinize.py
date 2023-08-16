#!/usr/bin/env python
import rospy
import message_filters
import math
import tf
import tf2_ros
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid

# if scan_val is Inf, inf_distance is assigned.
inf_distance = 5.0
# the radius of the robot explore area
territory_radius = 0.4
half_of_scan_size = 30
# the velocity of exploring and the type of explore state
explore_vel = 0.20
explore_time = 0.5
explore_yaw_vel = 2.0
explore_yaw_time = 1.0
explore_state = ['front', 'back', 'turn', 'after_turn', 'explored']

class gas_scrutinize:
    def __init__(self):

        rospy.init_node("gas_scrutinize")

        self.execute = False
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.gas_value_sub = rospy.Subscriber("/gas", Float32, self.gas_callback)
        # use tf
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(10.0)
        
        # self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        self.is_finish_search_sub = rospy.Subscriber("/is_finish_search", Bool, self.search_callback)

        
        # self.estimated_gas_map_sub = rospy.Subscriber("estimated_gas_map", self.map_callback)

        self.timeout_sec = rospy.get_param("~timeout_sec", 30.0)

        self.start_time = 0
        self.max_gas_value = 0.0
        self.final_robot_pose = Pose()
        self.robot_pose = Pose()
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = "map"
        self.cmd_x = 0.0
        self.cmd_yaw = 0.0
        self.explore_state = 'front'
        self.iscorrect_path = True
        self.max_gas_value_time = rospy.get_time()
        self.before_gas_value = 0
        self.gas_value = 0

        rospy.spin()

    def search_callback(self, msg):
        self.execute = msg.data
        self.start_time = rospy.get_time()
        # for debug
        self.max_gas_value_time = rospy.get_time()
        rospy.loginfo("execute: at %s", rospy.get_time())

    # def odom_callback(self, msg):
    #     if not self.execute:
    #         return
    #     self.robot_pose = msg.pose.pose

    def gas_callback(self,msg):
        if not self.execute:
            return
        self.before_gas_value = self.gas_value
        self.gas_value = msg.data
        if msg.data > self.max_gas_value:
            try:
                self.max_gas_value = msg.data
                self.max_gas_value_time = rospy.get_time()
                t = self.tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
                transform = t.transform
                self.final_robot_pose.position.x = transform.translation.x
                self.final_robot_pose.position.y = transform.translation.y
                self.final_robot_pose.position.z = transform.translation.z
                self.final_robot_pose.orientation.x = transform.rotation.x
                self.final_robot_pose.orientation.y = transform.rotation.y
                self.final_robot_pose.orientation.z = transform.rotation.z
                self.final_robot_pose.orientation.w = transform.rotation.w
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                self.rate.sleep()
            

    def explore(self):
        if self.explore_state == 'front':
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
        if not self.execute:
            return

        if self.start_time == 0:
            self.start_time = rospy.get_time()
            self.max_gas_value_time = rospy.get_time()
            rospy.loginfo("In callback set start_time at %s", rospy.get_time)
            return

        if self.explore_state == "explored":
            return

        last_sec = (rospy.get_time() - self.max_gas_value_time)
        rospy.loginfo_throttle(1.0, "last_sec: %f", last_sec)
        if last_sec  > self.timeout_sec:
            rospy.logwarn_once("Robot discovered goal!")
            self.explore_state = "explored"
            self.cmd_x = 0.0
            self.cmd_yaw = 0.0
            cmd_msg = Twist()
            cmd_msg.linear.x = self.cmd_x
            cmd_msg.angular.z =self.cmd_yaw
            self.vel_pub.publish(cmd_msg)
            
            self.goal_pose.header.seq = self.goal_pose.header.seq + 1
            self.goal_pose.header.stamp = rospy.Time.now()
            self.goal_pose.pose = self.final_robot_pose
            rospy.loginfo("goal_pose: %s", self.goal_pose)
            rospy.loginfo("final_robot_pose %s", self.final_robot_pose)
            self.goal_pub.publish(self.goal_pose)
            self.execute = False
            return

        #front_dist = msg.ranges[0]
        # front_dists = msg.ranges[:20]+ msg.ranges[340:]
        # front_dist = min(list(map(lambda x: inf_distance if x == float('inf') else x, front_dists)))
        size = len(msg.ranges)
        front_dists = msg.ranges[:half_of_scan_size]+ msg.ranges[(size - half_of_scan_size):]
        front_dist = min(list(map(lambda x: inf_distance if (x == float('inf') or x == 0.0
        ) else x, front_dists)))
        # rospy.loginfo_throttle(1.0, "minimum front distance: %f", front_dist)

        if self.before_gas_value < self.gas_value:
            self.explore_state = 'front'

        if (front_dist < territory_radius):
            self.cmd_x = 0.0
            self.cmd_yaw = 0.5
            self.explore_state = 'front'
            # rospy.loginfo_throttle(1.0, 'state: avoiding obstacle',)
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
        gas_scrutinize_action = gas_scrutinize()
    except rospy.ROSInterruptException: pass
