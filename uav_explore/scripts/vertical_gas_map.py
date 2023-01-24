#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import quaternion_from_euler
from std_msgs.msg import UInt16, UInt16MultiArray

# Todo: create vertical gas gridmap.

class vertical_gas_map:
    def __init__(self):
        rospy.init_node("vertical_gas_map")
        
        self.mocap_sub = rospy.Subscriber("/quadrotor/mocap/pose", PoseStamped, self.mocap_callback)
        self.tvoc_sub = rospy.Subscriber("/quadrotor/tvoc", UInt16, self.tvoc_callback)
        self.map_pub = rospy.Publisher("/quadrotor/vertical_grid_map", UInt16MultiArray, queue_size=1)

        # get rosparams
        self.initial_height = rospy.get_param("initial_height")
        self.limited_height = rospy.get_param("limited_height")
        self.grid_size = rospy.get_param("grid_size")
        
        self.half_of_grid_size = self.grid_size

        self.gas_map_data = []
        rospy.sleep(1.0)
        rospy.spin()
        self.pos_z = None
        self.gas_value = None
        self.index = None

    def tvoc_callback(self, msg):
        self.gas_value = msg.data
        
    def mocap_callback(self, msg):
        self.pos_z = msg.pose.position.z
        self.index = int(self.pos_z + self.half_of_grid_size - self.initial_height)
        if self.index < 0:
            return
        self.map_publisher()
        rospy.sleep(1.0)
        
    def map_publisher(self):
        if self.gas_value == None:
            return
        self.gas_map_data[self.index] = self.gas_value
        pub_map = UInt16MultiArray()
        pub_map.data = self.gas_map_data
        self.map_pub.publish(pub_map)
        
        
if __name__ == "__main__":
    try:
        vertical_gas_map()
    except rospy.ROSInterruptException: pass
