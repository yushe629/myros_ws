#!/usr/bin/env python

import rospy
import smach
import smach_ros
import ros_numpy
import copy

def main():
    rospy.init_node('patrol_motion')
    sm_top = smach.StateMachine(outcomes=['succeeded'])

    sm_top.userdata.waypoint_info = rospy.get_param('~waypoint_count', [])
    sm_top.userdata.cnt = 0

    sm_top.userdata.agv_init_position = None
    sm_top.userdata.uav_init_position = None


    if len(sm_top.userdata.waypoint_info) == 0:
        rospy.logerror('no valid waypoint info. exit')
        return


    with sm_top:
        smach.StateMachine.add('Start', Start(),
                               transitions = {'succeeded':'AgvWayPoint'},
                               remapping = {'agv_init_position': 'agv_init_position'})


        smach.StateMachine.add('AgvMotion', AgvMotion(),
                               transitions = {'flight':'UavMotion',
                                            'idle':'Idle',
                                            'failed':'Finish'},
                               remapping = {'waypoint_info': 'waypoint_info',
                                            'cnt': 'cnt'})


        with sm_uav:
            smach.StateMachine.add('Takeoff', UavTakeoff(),
                                   transitions = {'succeeded':'WayPoint',
                                                  'failed':'Land'},
                                   remapping = {'init_position': 'uav_init_position'})

            smach.StateMachine.add('WayPoint', UavWayPoint('before_sensing', True),
                                   transitions = {'succeeded':'Sensing',
                                                  'failed':'Land'},
                                   remapping = {'waypoint_info': 'waypoint_info',
                                                'cnt': 'cnt',
                                                'target_position': 'uav_init_position'})

            smach.StateMachine.add('Sensing', UavSensing(),
                                   transitions = {'succeeded':'Land',
                                                  'failed':'Land'},
                                   remapping = {'waypoint_info': 'waypoint_info',
                                                'cnt': 'cnt'})

            smach.StateMachine.add('WayPoint', UavWayPoint('after_sensing', False),
                                   transitions = {'succeeded':'Land',
                                                  'failed':'Land'},
                                   remapping = {'waypoint_info': 'waypoint_info',
                                                'cnt': 'cnt',
                                                'target_position': 'uav_init_position'})

            smach.StateMachine.add('Land', UavLand(),
                                   transitions = {'continue':'continue',
                                                  'finish':'finish'
                                                  'failed':'preempted'},
                                   remapping = {})


        smach.StateMachine.add('UavMotion', sm_uav,
                               transitions={'continue':'AgvMotion',
                                            'finish':'Finish'},
                               remapping={'waypoint_info': 'waypoint_info',
                                          'cnt': 'cnt'
                                          'uav_init_position': 'uav_init_position'})

        smach.StateMachine.add('Idle', Idle(),
                               transitions={'continue':'AgvMotion',
                                            'finish':'Finish'},
                               remapping = {'waypoint_info': 'waypoint_info',
                                            'cnt': 'cnt'})


        smach.StateMachine.add('Finish', Finish(),
                               transitions={'preempted':'preempted'},
                               remapping={'agv_init_position': 'agv_init_position'})


        sis = smach_ros.IntrospectionServer('patrol_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
