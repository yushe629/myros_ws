#!/usr/bin/env python
import rospy
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

# Todo: Selecting Message Type of /quadrotor/move/response. Now, using Bool.

class uav_response:
    def __init__(self):
        rospy.init_node("uav_response")
        self.flight_nav_sub = rospy.Subscriber("/quadrotor/uav/nav", FlightNav, self.nav_callback)
        self.mocap_sub = rospy.Subscriber("/quadrotor/mocap/pose", PoseStamped, self.mocap_callback)
        self.response_pub = rospy.Publisher("/quadrotor/move/response", Bool, queue_size=1)
        self.torelance_z = rospy.get_param("torelance_z")
        self.target_height = None
        
        rospy.sleep(1.0)
        rospy.spin()
        
    def nav_callback(self, msg):
        self.target_height = msg.target_pos_z
        
    def mocap_callback(self, msg):
        if self.target_height == None:
            return
        z = msg.pose.position.z
        if self.target_height - self.torelance_z < z < self.target_height + self.torelance_z:
            pub_msg = Bool()
            pub_msg.data =  True
            self.response_pub.publish(pub_msg)
            rospy.loginfo("Quadrotor has reached target_height.")
            self.target_height = None

if __name__ == "__main__":
    try:
        uav_response()
    except rospy.ROSInterruptException: pass
