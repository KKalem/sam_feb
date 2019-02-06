#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8


from __future__ import print_function

import rospy
import actionlib

from std_msgs.msg import Empty, Bool

from bee_tea.msg import BTAction, BTGoal, BTFeedback
from bee_tea.bt_states import SUCCESS, FAILURE, RUNNING
from bee_tea.bt_pieces import ActionNodeLeaf, InstantLeaf, Seq, Fallback, Negate

class SAM:
    def __init__(self):
        """
        create subscribers and such here
        """
        self.is_safe_flag = True
        emergency_stop_sub = rospy.Subscriber("/abort", Empty, self.abort_cb)
        pass

    def abort_cb(self, data):
        self.is_safe_flag = False

    def is_safe(self):
        if self.is_safe_flag:
            return SUCCESS

        return FAILURE

    def continue_command_received(self):
        rospy.loginfo('Continue command received')
        return SUCCESS

    def prepare_system(self):
        rospy.loginfo('System prepared')
        return SUCCESS

    def mission_synchronised(self):
        rospy.loginfo('Mission is already synchronised')
        return SUCCESS

    def synchronise_mission(self):
        rospy.loginfo('Mission is synchronised')
        return SUCCESS

    def mission_complete(self):
        rospy.loginfo('Mission is complete')
        return FAILURE

    def finalised(self):
        rospy.loginfo('Mission is already finalised')
        return SUCCESS

    def finalise(self):
        rospy.loginfo('Finalised mission')
        return SUCCESS

    def idle(self):
        rospy.loginfo('Idling...')
        return RUNNING



if __name__ == '__main__':
    sam = SAM()

    # root of the tree
    root = Seq('Root')

    # actionlib nodes, to be used later
    # TODO make action names rosparams at some point
    rospy.init_node('BT')

    safety_action = ActionNodeLeaf('sam_emergency', goal='No goal, just float up')
    execute_mission = ActionNodeLeaf('sam_sines', goal='3')

    # safety
    safety_check = Fallback('safety_check')
    safety_check.add_child(InstantLeaf('safe?', sam.is_safe))
    safety_check.add_child(safety_action)
    root.add_child(safety_check)

    # system prep
    sys_prep = Fallback('sys_prep')
    sys_prep.add_child(InstantLeaf('continue command?', sam.continue_command_received))
    sys_prep.add_child(InstantLeaf('prepare system', sam.prepare_system))
    root.add_child(sys_prep)

    # mission sync
    mission_sync = Fallback('mission_sync')
    mission_sync.add_child(InstantLeaf('mission synchronised?', sam.mission_synchronised))
    mission_sync.add_child(InstantLeaf('synchronise_mission', sam.synchronise_mission))
    root.add_child(mission_sync)

    # mission exec
    mission_exec = Fallback('mission_exec')
    mission_exec.add_child(InstantLeaf('mission_complete?', sam.mission_complete))
    mission_exec.add_child(execute_mission)
    root.add_child(mission_exec)

    # mission finalise
    mission_finalise = Fallback('mission_finalise')
    mission_finalise.add_child(InstantLeaf('finalised?', sam.finalised))
    mission_finalise.add_child(InstantLeaf('finalse', sam.finalise))
    root.add_child(mission_finalise)

    # idle in the end to prevent the tree from 'succeeding'
    root.add_child(InstantLeaf('IDLE', sam.idle))

    # TREE COMPLETE


    # current return state of the entire tree
    r = FAILURE
    # less print-spam
    prev_s = ''

    # tree ticking freq
    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        # send a tick through the tree.
        # this is recursive
        r = root.tick()
        # s is a string that represents the tree, together with current states
        # and node names, indented.
        s = root.display(0)

        # less spam
        if s != prev_s:
            print(s)
        prev_s = s

        # a SUCCESS or FAILURE at the root node indicates 'done'
        if r == SUCCESS or r == FAILURE:
            print('We are done here')
            break

        # i am not as fast as the CPU, normally not needed
        rate.sleep()
