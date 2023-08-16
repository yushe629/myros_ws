#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, UInt16


class get_co2:
    def __init__(self):
        
        rospy.init_node("get_co2")
        self.gas_pub = rospy.Publisher("/gas", Float32, queue_size=10)
        self.eco2_sub = rospy.Subscriber("/eco2", UInt16, self.callback)
        # self.norm = rospy.get_param("~norm", 10.0)
        # for test, setting norm = 5.0
        self.norm = 5.0
        self.base = 400.0
        rospy.spin()
        
    def callback(self, msg):
        gas_msg = Float32()
        gas_msg.data = float(msg.data - self.base)/self.norm
        self.gas_pub.publish(gas_msg)

if __name__ == "__main__":
    try:
        get_co2_action = get_co2()
    except rospy.ROSInterruptException: pass
