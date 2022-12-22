#!/usr/bin/env python
import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

AREA_SIZE = 0.025

class uav_response:
    def __init__(self):
        rospy.init_node("uav_response")
        self.flight_nav_sub = rospy.Subscriber("/quadrotor/uav/nav", FlightNav, self.nav_callback)
        self.mocap_sub = rospy.Subscriber("quadrotor/mocap/pose", Pose, self.mocap_callback)
        self.response_pub = rospy.Publisher("/quadrotor/move/response", Bool, queue_size=1)
        self.target_height = None
        
    def nav_callback(self, msg):
        self.target_height = msg.target_pos_z
        
    def mocap_callback(self, msg):
        z = msg.position.pose.z
        if self.target_height - AREA_SIZE < z < self.target_height + AREA_SIZE:
            pub_msg = Bool()
            pub_msg.data =  True
            self.response_pub.publish(pub_msg)
            rospy.loginfo("Quadrotor has reached target_height.")

if __name__ == "__main__":
    try:
        uav_response()
    except rospy.ROSInterruptException: pass
