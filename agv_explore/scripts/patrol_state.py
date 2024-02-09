#!/usr/bin/env python

import rospy
import smach
import smach_ros
import numpy as np
import tf.transformations as tft
import ros_numpy as ros_np
from std_msgs.msg import Empty
import copy
import tf2_ros
from jsk_rviz_plugins.msg import OverlayText

class Start(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['succeeded', 'preempted'],
                           io_keys=['agv_init_position'])

        self.task_start = False
        self.task_start_sub = rospy.Subscriber('~task_start', Empty, self.taskStartCallback)

    def taskStartCallback(self, msg):
        self.task_start = True

    def execute(self, userdata):

        while not self.task_start:
            rospy.sleep(0.1)
            rospy.logdebug_throttle(1.0, "wait to start task")

            if rospy.is_shutdown():
                return 'preempted'

            pass


        # get the initial position of AGV
        TODO
        userdata.agv_init_position = self.agv_position

        return 'succeeded'


# AGV
class AgvMotion(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['flight', 'idle', 'failed'],
                           io_keys=['waypoint_info', 'cnt'])


    def execute(self, userdata):

        TODO

        # send the target position to AGV

        # check the convergence

        while reach:

            break



        waypoint = userdata.waypoint[userdata.cnt]
        if len(waypoint) < 2:

            # only position
            return 'idle'

        elif len(waypoint) == 4:

            # flight
            return 'flight'

        else:

            # invalid waypoint
            return 'failed'


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


## UAV
class UavTakeoff(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['succeeded', 'failed'],
                           io_keys=['init_position'])


    def execute(self, userdata):


        # arm and takeoff
        TODO

        # takeoff

        while timeout:

            if converge:
                return 'succeeded'


        return 'failed'


class UavWayPoint(smach.State):
    def __init__(self, name, use_waypoint):

        smach.State.__init__(self,
                           outcomes=['succeeded', 'failed'],
                           io_keys=['waypoint_info', 'cnt', 'target_position'])


        self.use_waypoint = use_waypoint
        self.offset = rospy.get_param('~uav/' + name + '/waypoint_offset', 0.3)


    def execute(self, userdata):


        # send target position from
        if self.use_waypoint:
            waypoint = userdata.waypoint_info[userdata.cnt]
        else:
            waypoint = userdata.target_position + self.offset

        # convergent
        while timeout:

            if converge:
                return 'succeeded'


        return 'failed'


class UavSensing(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['succeeded', 'failed'],
                           io_keys=['waypoint_info', 'cnt'])



    def execute(self, userdata):


        img = None
        TODO: get image subscriber once, timeout


        if img is None:
            rospy.logerror("Cannot get the camera image data from topic of ...")
            return 'failed'


        rospy.loginfo("Get the camera image data from topic of ...")
        return 'succedded'


class UavLand(smach.State):
    def __init__(self):

        smach.State.__init__(self,
                           outcomes=['succeeded', 'failed'],
                           io_keys=[])


    def execute(self, userdata):


        # land
        TODO: send land


        # convergence
        while True:

            if finish:

                return 'succeeded'

        return 'failed'
