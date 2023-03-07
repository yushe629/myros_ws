#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, UInt16


class get_co2:
    def __init__(self):
        
        rospy.init_node("get_tvoc")
        self.gas_pub = rospy.Publisher("/gas", Float32, queue_size=10)
        # /tvoc topic publish TVOC value in ppb
        self.eco2_sub = rospy.Subscriber("/tvoc", UInt16, self.callback)
        # self.norm = rospy.get_param("~norm", 10.0)
        # for test, setting norm = 5.0
        # gas map data must be less than 127
        self.norm = 10.0
        rospy.spin()
        
    def callback(self, msg):
        gas_msg = Float32()
        # /tvoc data type is UInt16 and /gas data should be Int8 for OccupancyGrid data
        gas_msg.data = float(msg.data)/600
        # if msg.data > 1000:
        #     gas_msg.data = 100
        # else:
        #     gas_msg.data = float(msg.data)/self.norm
        self.gas_pub.publish(gas_msg)

if __name__ == "__main__":
    try:
        get_co2_action = get_co2()
    except rospy.ROSInterruptException: pass
