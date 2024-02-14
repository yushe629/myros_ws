#!/usr/bin/env python

import rospy
import smach
import smach_ros
import numpy as np
import tf.transformations as tft
import tf2_ros
import ros_numpy as ros_np
import copy
from jsk_rviz_plugins.msg import OverlayText

from std_msgs.msg import Empty, UInt8
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalStatus
from aerial_robot_msgs.msg import PoseControlPid
from nav_msgs.msg import Odometry

from sensor_msgs.msg import Image 
from cv_bridge import CvBridge
import cv2

class Start(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['succeeded', 'preempted'],
                           io_keys=['agv_init_position'])

        self.task_start = False
        self.task_start_sub = rospy.Subscriber('~task_start', Empty, self.taskStartCallback)
        self.agv_frame = rospy.get_param("~agv/frame", "base_footprint")
        self.map_frame = rospy.get_param("~map_frame", "/map")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def taskStartCallback(self, msg):
        self.task_start = True

    def execute(self, userdata):

        while not self.task_start:
            rospy.sleep(0.1)
            rospy.logdebug_throttle(1.0, "wait to start task")

            if rospy.is_shutdown():
                return 'preempted'

            pass

        while not rospy.is_shutdown():

            try:
                trans = self.tf_buffer.lookup_transform(self.map_frame, self.agv_frame, rospy.Time.now(), rospy.Duration(1.0))
                trans = ros_np.numpify(trans)

                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue


        # get the initial position of AGV
        userdata.agv_init_position = tft.translation_from_matrix(trans)

        return 'succeeded'


# AGV
class AgvMotion(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['flight', 'idle', 'failed'],
                           io_keys=['waypoint_info', 'cnt'])

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)
        self.move_base_result_sub = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.waypointCallback)

        self.reach = None


    def waypointCallback(self, msg):

        rospy.logdebug("move base action status: %s", msg.status.text)

        if msg.status.status == GoalStatus.SUCCEEDED:
            self.reach = True
        else:
            self.reach = False

    def execute(self, userdata):

        # send the target position to AGV
        waypoint = userdata.waypoint[userdata.cnt]

        if not (len(waypoint) == 2 or len(waypoint) == 4):
            # invalid waypoint
            rospy.logerr("the waypoint{} is invalid: {}".format(userdata.cnt, waypoint))
            return 'failed'

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.seq = userdata.cnt + 1
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = waypoint[0]
        goal.pose.position.y = waypoint[1]
        goal.pose.orientation.w = 1
        self.goal_pub.publish(goal)

        # check the convergence

        while not rospy.is_shutdown():

            if self.reach is not None:

                if self.reach:

                    if len(waypoint) == 2:
                        # only position
                        return 'idle'
                    else:
                        # flight
                        return 'flight'
                else:
                    return 'failed'

                break

            rospy.sleep(0.1)


class Idle(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['continue', 'finish'],
                           io_keys=['waypoint_info', 'cnt'])

        self.idle_duration = rospy.get_param('~agv/idle_duration', 2.0)

    def execute(self, userdata):

       rospy.sleep(self.idle_duration)

       userdata.cnt += 1

       if userdata.cnt == len(userdata.waypoint):
           return 'finish'
       else:
           return 'continue'

class Finish(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                             outcomes=['preempted'], io_keys=['agv_init_position'])

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size = 1)

    def execute(self, userdata):

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.seq = userdata.cnt + 1
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = userdata.agv_init_position[0]
        goal.pose.position.y = userdata.agv_init_position[1]
        goal.pose.orientation.w = 1
        self.goal_pub.publish(goal)

        return 'preempted'


## UAV
class UavTakeoff(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['succeeded', 'failed'],
                           io_keys=['waypoint_info', 'cnt', 'before_sensing_target_pose', 'after_sensing_target_pose'])

        self.arm_pub = rospy.Publisher("start", Empty, queue_size = 1)
        self.takeoff_pub = rospy.Publisher("takeoff", Empty, queue_size = 1)
        self.state_sub = rospy.Subscriber("flight_state", UInt8, self.stateCallback)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomCallback)

        self.land_height_offset = rospy.get_param('~uav/land_height_offset', 0.3)
        self.timeout = rospy.get_param('~uav/takeoff_timeout', 30.0)

        self.map_frame = rospy.get_param('~map_frame', '/map')
        self.uav_world_frame = rospy.get_param('~uav/world_frame', '/world')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.flight_state = None
        self.pos = None
        self.yaw = None

    def odomCallback(self, msg):
        self.pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        euler = tft.euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.yaw = euler[2]


    def stateCallback(self, msg):

        self.flight_state = msg.data

    def execute(self, userdata):

        # get waypoint
        waypoint = userdata.waypoint[userdata.cnt]

        # set the uav initial position
        userdata.after_sensing_target_pose = [self.pos.x, self.pos.y, self.pos.z + self.land_height_offset, self.yaw]

        # get offset from /map to /world (/camera_init of fast_lio)
        while not rospy.is_shutdown():

            try:
                trans = self.tf_buffer.lookup_transform(self.map_frame, self.uav_frame, rospy.Time.now(), rospy.Duration(1.0))
                trans = ros_np.numpify(trans)
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue


        # motor arm
        self.arm_pub.publish(Empty())
        rospy.sleep(2.0) # TODO, check the duration
        self.takeoff_pub.publish(Empty())

        # takeoff
        t = rospy.get_time()

        while rospy.get_time() - t < self.timeout:

            if self.flight_state == 5: # hover

                # set the waypoint for after-sensing

                # calculate the true height and yaw for uav from tf map -> world (camera_init)
                target_pose_map = tft.concatenate_matrices(tft.translation_matrix(waypoint[0:3]), tft.euler_matrix(0,0,waypoint[3]))
                target_pose_uav = tft.concatenate_matrices(tft.inverse_matrix(trans), target_pose_map)

                userdata.before_sensing_target_pose = userdata.after_sensing_target_pose
                userdata.before_sensing_target_pose[2] = tft.translation_from_matrix(target_pose_uav)[2] # z
                userdata.after_sensing_target_pose[3] = tft.euler_from_matrix(target_pose_uav)[2] # yaw

                return 'succeeded'

            rospy.sleep(0.1)

        rospy.logerr("[Timeout] UAV cannot hover after takeoff")
        return 'failed'


class UavWayPoint(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['succeeded', 'failed'],
                           io_keys=['uav_target_pose'])

        self.timeout = rospy.get_param('~uav/hover_timeout', 30.0)
        self.nav_pub = rospy.Publisher("~uav/nav", PoseStamped, queue_size = 1)
        self.control_sub = rospy.Subscriber('~control_term', PoseControlPid, self.controlCallback)
        self.pos_converge_thresh = rospy.get_param('~uav/hover_pos_converge_tresh', 0.05)
        self.rot_converge_thresh = rospy.get_param('~uav/hover_rot_converge_tresh', 0.10)
        self.target_pos = None
        self.pos_error = None
        self.rot_error = None

    def controlCallback(self, msg):
        self.target_pos = [msg.x.target_p, msg.y.target_p, msg.z.target_p]
        self.pos_error = [msg.x.err_p, msg.y.err_p, msg.z.err_p]
        self.rot_error = [msg.roll.err_p, msg.pitch.err_p, msg.yaw.err_p]

    def execute(self, userdata):

        # send the waypoint
        waypoint = userdata.target_pose
        msg = PoseStamped()
        # only use z and yaw
        msg.pose.position.x = self.target_pos[0]
        msg.pose.position.y = self.target_pos[1]
        msg.pose.position.z = waypoint[2]
        q = tft.quaternion_from_matrix(tft.euler_matrix(0, 0, waypoint[3]))
        msg.pose.orientation = Quaternion(q)
        self.nav_pub.publish(msg)

        rospy.sleep(0.5)

        # convergent
        t = rospy.get_time()

        while rospy.get_time() - t < self.timeout:

            # only pos z and yaw
            if np.abs(self.pos_error[2]) < self.pos_converge_thresh and \
               np.abs(self.rot_error[2]) < self.rot_converge_thresh: 
                return 'succeeded'


        rospy.logwarn("[UAV] hovering cannot converge")
        return 'failed'


class UavSensing(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['succeeded', 'failed'],
                           io_keys=['waypoint_info', 'cnt'])


    def execute(self, userdata):

        # caputre and save image
        try:
            img_msg = rospy.wait_for_message("/camera/image_raw", Image, timeout = 2.0)
        except ROSException:
            rospy.logerror("Cannot get the camera image data from topic of ...")
            return 'failed'

        rospy.loginfo("Get the camera image data")
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        cv2.imwrite('waypoint' + str(cnt+1) + '.jpeg', img)

        return 'succedded'


class UavLand(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['succeeded', 'failed'],
                           io_keys=[])


        self.land_pub = rospy.Publisher("land", Empty, queue_size = 1)
        self.state_sub = rospy.Subscriber("flight_state", UInt8, self.stateCallback)

        self.timeout = rospy.get_param('~uav/land_timeout', 30.0)
        self.flight_state = None

    def stateCallback(self, msg):

        self.flight_state = msg.data

    def execute(self, userdata):

        # land
        self.land_pub.publish(Empty())


        # takeoff
        t = rospy.get_time()

        while rospy.get_time() - t < self.timeout:

            if self.flight_state == 0: # stop
                return 'succeeded'

            rospy.sleep(0.1)


        return 'failed'
