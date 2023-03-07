#!/usr/bin/env python
import rospy
import message_filters
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32, Bool
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from nav_msgs.msg import OccupancyGrid
import numpy as np

class gas_roughly_search:
    def __init__(self):
        
        rospy.init_node("gas_roughly_search")
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.estimated_gas_map_sub = rospy.Subscriber("estimated_gas_map",OccupancyGrid,  self.callback)

        # TODO: adjust params
        self.gas_visual_map = OccupancyGrid()
        self.gas_visual_map.header.frame_id = "map"
        self.gas_scale = rospy.get_param("~gas_visual_scale", 1.0)
        self.gas_offset = rospy.get_param("~gas_visual_offset", 0.0)

        
        self.gas_visual_map.info.resolution = rospy.get_param("~gas_visual_map_resolution", 0.05) # 0.05 m/pixel
        origin = rospy.get_param("~gas_visual_map_origin", [-10.0, -10.0, 0.0])
        self.gas_visual_map.info.width = int(math.fabs(origin[0]) / self.gas_visual_map.info.resolution * 2)
        self.gas_visual_map.info.height = int(math.fabs(origin[1]) / self.gas_visual_map.info.resolution * 2)
        self.gas_visual_map.info.origin.position.x = origin[0]
        self.gas_visual_map.info.origin.position.y = origin[1]
        self.gas_visual_map.info.origin.position.z = origin[2]
        self.gas_visual_map.info.origin.orientation.w = 1
        self.gas_visual_map.data = [0] * self.gas_visual_map.info.width * self.gas_visual_map.info.height

        
        self.execute = False
        self.completed = False

        # publish and subscirbe topic 
        self.is_finish_patrol_sub = rospy.Subscriber("/is_finish_patrol", Bool, self.patrol_callback)
        self.is_finish_search_pub = rospy.Publisher("/is_finish_search", Bool, queue_size=1)
        self.move_base_result_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.move_base_callback)
        
        
        rospy.spin()

    def move_base_callback(self, msg):
        if not self.execute:
            return
        if msg.status.status == 3:
            rospy.loginfo("%s", msg.status.text)
            rospy.sleep(1.0)
            new_msg = Bool()
            new_msg.data = True
            self.is_finish_search_pub.publish(new_msg)
            self.execute = False
        else:
            rospy.loginfo("%s",msg.status.text)

    def patrol_callback(self, msg):
        self.execute = msg.data
        if self.execute:
            rospy.loginfo("Searching")
        
    def callback(self, msg):
        if not self.execute:
            return
        if not self.completed:
            mapdata = msg.data
            mapindex = mapdata.index(max(mapdata))
            target_x_pixel = mapindex % self.gas_visual_map.info.width
            target_y_pixel = mapindex / self.gas_visual_map.info.width
            target_x = target_x_pixel * self.gas_visual_map.info.resolution + self.gas_visual_map.info.origin.position.x
            target_y = target_y_pixel * self.gas_visual_map.info.resolution + self.gas_visual_map.info.origin.position.y
            target = PoseStamped()
            target.header.frame_id = "map"
            target.header.seq = target.header.seq + 1
            target.header.stamp = rospy.Time.now()
            target.pose.position.x = target_x
            target.pose.position.y = target_y
            target.pose.position.z = 0.0
            target.pose.orientation.x = 0.0
            target.pose.orientation.y = 0.0
            target.pose.orientation.z = 0.0
            target.pose.orientation.w = 1.0
            self.goal_pub.publish(target)
            self.completed = True
        return
        
if __name__ == "__main__":
    try:
        gas_roughly_search_action = gas_roughly_search()
    except rospy.ROSInterruptException: pass
