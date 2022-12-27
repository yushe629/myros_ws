#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import quaternion_from_euler
from std_msgs.msg import UInt16, Bool
from move_as_pos_z import move_as_pose_z

# Constants
INITIAL_HEIGHT = 1.0
MAX_HEIGHT = 1.5
POSE_Z_DIFF = 0.1
# For ignoring noise, using weight.
TVOC_WEIGHT = 0.8

class uav_explore:
    def __init__(self):
        rospy.init_node("uav_explore")
        self.tvoc_sub = rospy.Subscriber("/quadrotor/tvoc", UInt16, self.tvoc_callback)
        self.uav_response_sub = rospy.Subscriber("/quadrotor/move/response", Bool, self.uav_callback)
        self.mocap_sub = rospy.Subscriber("/quadrotor/mocap/pose", PoseStamped, self.mocap_callback)
        self.initinal_height = rospy.get_param("initial_height")
        self.limited_height = rospy.get_param("limited_height")
        self.pose_z_diff = rospy.get_param("pose_z_diff")
        self.tvoc_weight = rospy.get_param("tvoc_weight")
        # Set initial tvoc max value.
        self.tvoc_max_value = 0
        # Set initial quadrotor hovering position of z. (1.0m)
        self.max_value_pos_z = self.initinal_height
        # For setting next goal of pos_z.
        self.next_pose_z = self.initinal_height
        # If self.completed = True, gas source has been localized.
        self.is_completed = False
        # If self.moving is true, quadrotor is rising now.
        self.is_moving = False
        self.mocap_pos = None
        rospy.sleep(1.0)

        while (not self.is_completed):
            if(not self.is_moving):
                self.is_moving = True
                self.next_pose_z = self.next_pose_z + self.pose_z_diff
                if self.next_pose_z > self.limited_height:
                    self.is_completed = True
                    rospy.loginfo("Quadrotor has reached limited_height. Maximum height is %f, max tvoc value is %f", self.max_value_pos_z, self.tvoc_max_value)
                    break
                move_as_pose_z(self.next_pose_z)
        rospy.spin()

    def mocap_callback(self, msg):
        self.mocap_pos = msg.pose

    def tvoc_callback(self, msg):
        if msg.data > self.tvoc_max_value:
            self.tvoc_max_value = msg.data
            self.max_value_pos_z = self.mocap_pos.position.z
        # if msg.data < self.tvoc_max_value * self.tvoc_weight:
        #     rospy.loginfo("Quadrotor has localized maximum point. That is %f", self.max_value_pos_z)
        #     self.is_completed = True

    def uav_callback(self, msg):
        if msg.data == True:
            self.is_moving = False

if __name__ == "__main__":
    try:
        uav_explore()
    except rospy.ROSInterruptException: pass
