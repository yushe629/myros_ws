#!/usr/bin/env python
import rospy
from aerial_robot_msgs.msg import FlightNav

    
def move_as_pose_z(target_pos_z):
    pub = rospy.Publisher("/quadrotor/uav/nav", FlightNav, queue_size=1)

    rospy.sleep(1.0)
    
    msg = FlightNav()
    msg.pos_z_nav_mode = 2
    msg.target_pos_z = target_pos_z
    
    pub.publish(msg)

if __name__ == "__main__":
    try:
        rospy.init_node("uav_move_as_pos_z")
        pos_z = rospy.get_param("~pos_z")
        move_as_pose_z(pos_z)
    except rospy.ROSInterruptException: pass
