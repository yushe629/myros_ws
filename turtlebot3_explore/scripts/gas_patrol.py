#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import quaternion_from_euler
from std_msgs.msg import Bool

PI = math.pi

class gas_patrol:
    def __init__(self):

        rospy.init_node("gas_patrol")

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)
        self.relay_points = rospy.get_param("~relay_points")
        # self.relay_points = [[2.0, -0.5, 0, PI/2.0], [2.0, 0.5,0, PI], [-2.0, 0.5, 0, 3*PI/2.0], [-2.0, -0.5, 0, 0]]
        self.move_base_result_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.callback)
        self.is_moving = False
        self.nth_point = 0
        self.is_goal_published = False
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"

        self.execute = True

        # publish topic for next node
        self.is_finish_patrol_pub = rospy.Publisher("/is_finish_patrol", Bool, queue_size = 1)
        
        rospy.sleep(1.0)
        self.patrol()
        rospy.spin()
        
    def callback(self, msg):
        if not self.execute:
            return
        if msg.status.status == 3:
            rospy.loginfo("%s", msg.status.text)
            # wait 4 sec for more sensing
            rospy.sleep(4.0)
            self.is_moving = False
            self.is_goal_published = False
            if self.nth_point == len(self.relay_points):
                msg = Bool()
                msg.data = True
                self.is_finish_patrol_pub.publish(msg)
                self.execute = False

        else:
            rospy.loginfo("%s",msg.status.text)
            
    def calc_plan(self, point):
        self.goal.header.seq = self.goal.header.seq + 1
        self.goal.header.stamp = rospy.Time.now()
        self.goal.pose.position.x = point[0]
        self.goal.pose.position.y = point[1]
        self.goal.pose.position.z = point[2]
        q = quaternion_from_euler(0,0,point[3])
        self.goal.pose.orientation.x = q[0]
        self.goal.pose.orientation.y = q[1]
        self.goal.pose.orientation.z = q[2]
        self.goal.pose.orientation.w = q[3]
        
    def patrol(self):
        if not self.execute:
            return
        while self.nth_point < len(self.relay_points):
            if not self.is_moving:
                if not self.is_goal_published:
                    self.calc_plan(self.relay_points[self.nth_point])
                    self.nth_point = self.nth_point+1
                    self.goal_pub.publish(self.goal)
                    self.is_goal_published = True
                    self.is_moving = True

if __name__ == "__main__":
    try:
        gas_patrol()
    except rospy.ROSInterruptException: pass
