#!/usr/bin/env python
import rospy
import math
import tf
import time
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Empty

class gas_patrol:
    def __init__(self):

        rospy.init_node("gas_patrol")

        self.relay_points = rospy.get_param("~relay_points")
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)
        self.finish_search = rospy.Publisher("/is_finish_search", Bool, queue_size = 1)
        self.task_start_sub = rospy.Subscriber("/start_task", Empty, self.task_start_callback)
        self.move_base_result_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.waypoint_callback)
        self.estimated_gas_map_sub = rospy.Subscriber("estimated_gas_map", OccupancyGrid, self.gas_map_callback)

        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"

        self.gas_map = None

        rospy.sleep(1.0)

        self.execute = False
        self.nth_point = 0
        self.reach_waypoint = True # for first waypoint
        self.final_approach = False
        self.patrol()


        rospy.spin()

    def task_start_callback(self, msg):
        self.execute = True

    def gas_map_callback(self, msg):
        self.gas_map = msg

    def waypoint_callback(self, msg):

        rospy.logdebug("move base action status: %s", msg.status.text)

        if msg.status.status == GoalStatus.SUCCEEDED:

            if self.final_approach:
                rospy.loginfo("reach the rought search point, switch to refine search phase")
                self.finish_search.publish(Bool(True))
                return

            rospy.loginfo("reach the waypoint{}: {}".format(self.nth_point, self.relay_points[self.nth_point-1][:2]))

            # wait 4 sec for more sensing
            rospy.sleep(4.0)
            self.reach_waypoint = True

            # shift to rough search phase
            if self.nth_point == len(self.relay_points):

                if self.gas_map is None:
                    rospy.warn("no estimated gas map received yet !!!")
                    return

                mapdata = self.gas_map.data
                mapindex = mapdata.index(max(mapdata))
                target_x_pixel = mapindex % self.gas_map.info.width
                target_y_pixel = mapindex / self.gas_map.info.width
                target_x = target_x_pixel * self.gas_map.info.resolution + self.gas_map.info.origin.position.x
                target_y = target_y_pixel * self.gas_map.info.resolution + self.gas_map.info.origin.position.y
                target = PoseStamped()
                target.header.frame_id = "map"
                target.header.seq = target.header.seq + 1
                target.header.stamp = rospy.Time.now()
                self.calc_plan([target_x, target_y, 0, 0])
                self.goal_pub.publish(self.goal)
                rospy.loginfo("approach to the rough serach point: {}".format([target_x, target_y]))

                self.execute = False

                self.final_approach = True

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

        while self.nth_point < len(self.relay_points) and not rospy.is_shutdown():

            if not self.execute:
                rospy.sleep(0.01)
                continue

            if not self.reach_waypoint:
                rospy.sleep(0.01)
                continue

            # move to the next waypoint
            self.calc_plan(self.relay_points[self.nth_point])
            self.nth_point += 1
            self.reach_waypoint = False
            self.goal_pub.publish(self.goal)


if __name__ == "__main__":
    try:
        gas_patrol()
    except rospy.ROSInterruptException: pass
