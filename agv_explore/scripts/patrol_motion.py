#!/usr/bin/env python

import rospy
import smach
import smach_ros
import ros_numpy
import copy
from patrol_state import *


def main():
    rospy.init_node('patrol_motion')
    sm_top = smach.StateMachine(outcomes=['preempted'])

    sm_top.userdata.waypoint_info = rospy.get_param('~waypoint_info', [])
    sm_top.userdata.cnt = 0

    sm_top.userdata.agv_init_position = None
    sm_top.userdata.uav_before_sensing_target_pose = None
    sm_top.userdata.uav_after_sensing_target_pose = None


    if len(sm_top.userdata.waypoint_info) == 0:
        rospy.logerr('no valid waypoint info. exit')
        return


    with sm_top:
        smach.StateMachine.add('Start', Start(),
                               transitions = {'succeeded':'AgvMotion'},
                               remapping = {'agv_init_position': 'agv_init_position'})


        smach.StateMachine.add('AgvMotion', AgvMotion(),
                               transitions = {'flight':'UavMotion',
                                            'idle':'Idle',
                                            'failed':'Finish'},
                               remapping = {'waypoint_info': 'waypoint_info',
                                            'cnt': 'cnt'})


        sm_uav = smach.StateMachine(outcomes=['finish', 'preempted'],
                                    input_keys=['waypoint_info', 'cnt',
                                                'agv_init_position',
                                                'uav_before_sensing_target_pose',
                                                'uav_after_sensing_target_pose'],
                                    output_keys=['waypoint_info', 'cnt',
                                                 'agv_init_position',
                                                'uav_before_sensing_target_pose',
                                                'uav_after_sensing_target_pose'])


        with sm_uav:
            smach.StateMachine.add('Takeoff', UavTakeoff(),
                                   transitions = {'succeeded':'WayPointBeforeSensing',
                                                  'failed':'preempted'},
                                   remapping = {'waypoint_info': 'waypoint_info',
                                                'cnt': 'cnt',
                                                'before_sensing_target_pose': 'uav_before_sensing_target_pose',
                                                'after_sensing_target_pose': 'uav_after_sensing_target_pose'})

            smach.StateMachine.add('WayPointBeforeSensing', UavWayPoint(),
                                   transitions = {'succeeded':'Sensing',
                                                  'failed':'preempted'},
                                   remapping = {'target_pose': 'uav_before_sensing_target_pose'})

            smach.StateMachine.add('Sensing', UavSensing(),
                                   transitions = {'succeeded':'WayPointAfterSening',
                                                  'failed':'WayPointAfterSening'},
                                   remapping = {'waypoint_info': 'waypoint_info',
                                                'cnt': 'cnt'})

            smach.StateMachine.add('WayPointAfterSening', UavWayPoint(),
                                   transitions = {'succeeded':'Land',
                                                  'failed':'preempted'},
                                   remapping = {'target_pose': 'uav_after_sensing_target_pose'})

            smach.StateMachine.add('Land', UavLand(),
                                   transitions = {'succeeded':'finish',
                                                  'failed':'preempted'},
                                   remapping = {})


        smach.StateMachine.add('UavMotion', sm_uav,
                               transitions={'finish':'Idle',
                                            'preempted':'preempted'},
                               remapping={'waypoint_info': 'waypoint_info',
                                          'cnt': 'cnt',
                                          'uav_init_position': 'uav_init_position'})

        smach.StateMachine.add('Idle', Idle(),
                               transitions={'continue':'AgvMotion',
                                            'finish':'Finish'},
                               remapping = {'waypoint_info': 'waypoint_info',
                                            'cnt': 'cnt'})


        smach.StateMachine.add('Finish', Finish(),
                               transitions={},
                               remapping={'agv_init_position': 'agv_init_position',
                                          'cnt': 'cnt'})


        sis = smach_ros.IntrospectionServer('patrol_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
