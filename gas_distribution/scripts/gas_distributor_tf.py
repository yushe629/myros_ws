#!/usr/bin/env python
import rospy
import tf
import tf2_ros
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32


map_width = 6.0
map_grid = 100

class GasDistributer:
    def __init__(self):

        rospy.init_node("gas_distributer")

        self.gas_origin = rospy.get_param("~gas_origin", [2.0,0.5,0.0])
        self.gas_origin = np.array(self.gas_origin)
        # self.alpha = rospy.get_param("~alpha", 1.0)
        self.max_val = rospy.get_param("~gap_max_val", 100)
        self.gas_value_pub = rospy.Publisher("/gas", Float32, queue_size=10)
        self.gas_map_pub = rospy.Publisher("/true_gas_map", OccupancyGrid, queue_size=1)
        # self.robot_pose_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.rate = rospy.Rate(10.0)
        
        # gas map for visualization
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

        self.map_gen()
        rospy.loginfo_once("gas_origin: %s", self.gas_origin)
        
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
        # pos is 3d_array
        dist = np.linalg.norm(self.gas_origin - pos)
        # val = self.max_val - self.alpha * dist**2
        val = self.max_val*math.exp(-dist**2)
        return val

    def odom_callback(self, msg):
        pos = msg.translation
        pos = np.array([pos.x, pos.y, pos.z])
        val = self.calc_gas_value(pos)
        rospy.loginfo("pos: %s", pos)
        value = Float32()
        value.data = val
        self.gas_value_pub.publish(value)
        self.gas_map_pub.publish(self.gas_visual_map)

    def map_gen(self):
        width = self.gas_visual_map.info.width
        height = self.gas_visual_map.info.height
        for i in range(width*height):
            x = i % width * self.gas_visual_map.info.resolution + self.gas_visual_map.info.origin.position.x
            y = i / width * self.gas_visual_map.info.resolution + self.gas_visual_map.info.origin.position.y
            half = self.gas_visual_map.info.resolution / 2
            value = self.calc_gas_value([x + half, y + half, 0])
            self.gas_visual_map.data[i] = int(value*self.gas_scale + self.gas_offset)

if __name__ == "__main__":
    try:
        gas_distributor = GasDistributer()
    except rospy.ROSInterruptException: pass
