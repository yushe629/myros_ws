#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32

class GasDistributer:
    def __init__(self):

        rospy.init_node("gas_distributer_time")

        # gas source coordinate
        self.gas_origin = rospy.get_param("~gas_origin", [2.0,0.5,0.0])
        self.gas_origin = np.array(self.gas_origin)
        # setting constant
        # self.alpha = rospy.get_param("~alpha", 1.0)
        self.max_val = rospy.get_param("~gap_max_val", 100)
        # define publisher and subscriber
        self.gas_value_pub = rospy.Publisher("/gas", Float32, queue_size=10)
        # self.robot_pose_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.gas_map_pub = rospy.Publisher("/true_gas_map", OccupancyGrid, queue_size=1)
        # setting rosTime
        self.start_time = 0
        self.time_rate_off = 1.0
        self.time_rate = 1.0
        self.dist_rate = 10.0
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(10.0)
        
        while not rospy.is_shutdown():
            try:
                t = self.tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0))
                #t = tfBuffer.lookup_transform('mug', 'base_link', rospy.Time())
                self.odom_callback(t.transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                self.rate.sleep()
            

        rospy.spin()

    def calc_gas_value(self, pos):
        if self.start_time == 0:
            self.start_time = rospy.get_time()
            return

        # pos is 3d_array
        dist = np.linalg.norm(self.gas_origin - pos)
        del_time = rospy.get_time() - self.start_time
        val = self.max_val*math.exp(-dist**2)*math.exp(-self.time_rate/(del_time + self.time_rate_off))
        return val

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        pos = np.array([pos.x, pos.y, pos.z])
        val = self.calc_gas_value(pos)
        # rospy.loginfo("pos: %s", pos)
        value = Float32()
        value.data = val
        self.gas_value_pub.publish(value)

if __name__ == "__main__":
    try:
        gas_distributor = GasDistributer()
    except rospy.ROSInterruptException: pass
