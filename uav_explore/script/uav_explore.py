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
MAX_HEIGHT = 2.0
POSE_Z_DIFF = 0.1
# For ignoring noise, using gain.
TVOC_WEIGHT = 0.8

class uav_explore:
    def __init__(self):
        rospy.init_node("uav_explore")
        self.tvoc_sub = rospy.Subscriber("/quadrotor/mocap/tvoc", UInt16, self.tvoc_callback)
        self.uav_response_sub = rospy.Subscriber("/quadrotor/move/response", Bool, self.uav_callback)
        # Set initial tvoc max value.
        self.tvoc_max_value = 0
        # Set initial quadrotor hovering position of z. (1.0m)
        self.max_value_pos_z = INITIAL_HEIGHT
        # For setting next goal of pos_z.
        self.next_pose_z = INITIAL_HEIGHT
        # If self.completed = True, gas source has been localized.
        self.is_completed = False
        # If self.moving is true, quadrotor is rising now.
        self.is_moving = False
        rospy.sleep(1.0)

        while (not self.is_completed) and (not self.is_moving):
            self.is_moving = True
            self.next_pose_z = self.next_pose_z + POSE_Z_DIFF
            if self.next_pose_z > MAX_HEIGHT:
                self.is_completed = True
                break
            move_as_pose_z(self.next_pose_z)
        rospy.spin()

    def tvoc_callback(self, msg):
        if msg.data > self.tvoc_max_value:
            self.tvoc_max_value = msg.data
        if msg.data < self.tvoc_max_value * TVOC_WEIGHT:
            self.is_completed = True

    def nav_callback(self, msg):
        if msg.data == True:
            self.is_moving = False

if __name__ == "__main__":
    try:
        uav_explore()
    except rospy.ROSInterruptException: pass
