#!/usr/bin/env python
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool
from move_as_pos_z import move_as_pose_z

class uav_explore:
    def __init__(self):
        rospy.init_node("uav_explore")
        self.uav_response_sub = rospy.Subscriber("/quadrotor/move/response", Bool, self.uav_callback)

        # get rosparams
        self.initial_height = rospy.get_param("initial_height")
        self.limited_height = rospy.get_param("limited_height")
        self.pose_z_diff = rospy.get_param("pose_z_diff")
        self.tvoc_weight = rospy.get_param("tvoc_weight")

        # Set initial tvoc max value.
        self.tvoc_max_value = 0
        # Set initial quadrotor hovering position of z. (1.0m)
        self.max_value_pos_z = self.initial_height
        # For setting next goal of pos_z.
        self.next_pose_z = self.initial_height
        # If self.completed = True, gas source has been localized.
        self.is_completed = False
        # If self.moving is true, quadrotor is rising now.
        self.is_moving = False
        rospy.sleep(1.0)

        while (not self.is_completed) and (not self.is_moving):
            self.is_moving = True
            self.next_pose_z = self.next_pose_z + self.pose_z_diff
            if self.next_pose_z > self.limited_height:
                self.is_completed = True
                break
            move_as_pose_z(self.next_pose_z)
        rospy.spin()

    def tvoc_callback(self, msg):
        if msg.data > self.tvoc_max_value:
            self.tvoc_max_value = msg.data
        if msg.data < self.tvoc_max_value * self.tvoc_weight:
            self.is_completed = True

    def nav_callback(self, msg):
        if msg.data == True:
            self.is_moving = False

if __name__ == "__main__":
    try:
        uav_explore()
    except rospy.ROSInterruptException: pass
