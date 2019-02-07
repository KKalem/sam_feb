#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8


from __future__ import print_function

import rospy, time
import actionlib

from std_msgs.msg import Empty, Bool, Float64

from bee_tea.msg import BTAction, BTGoal, BTFeedback
from bee_tea.bt_states import SUCCESS, FAILURE, RUNNING
from bee_tea.bt_pieces import ActionNodeLeaf, InstantLeaf, Seq, Fallback, Negate

DEPTH_THRESH = 0.05
PITCH_THRESH = 0.05

class SAM:
    def __init__(self, setpoint_list=None):
        """
        create subscribers and such here
        """

        self.mission_complete_flag = False
        self.safety_action_tried_flag = False

        self.is_safe_flag = True
        emergency_stop_sub = rospy.Subscriber("/abort", Empty, self.abort_cb)

        self.pitch = 0.0
        pitch_sub = rospy.Subscriber("/feedback_pitch", Float64, self.pitch_update_cb)

        self.depth = 0.0
        depth_sub = rospy.Subscriber("/feedback_depth", Float64, self.depth_update_cb)

        if setpoint_list is not None:
            self.setpoint_list = setpoint_list
        else:
            self.setpoint_list = []
        self.current_target = None

    def pitch_update_cb(self, data):
        self.pitch = data.data

    def depth_update_cb(self, data):
        self.depth = data.data


    def is_safety_action_tried(self):
        if self.safety_action_tried_flag:
            return FAILURE

        return SUCCESS

    def set_safety_action_tried(self):
        self.safety_action_tried_flag = True
        return SUCCESS

    def abort_cb(self, data):
        self.is_safe_flag = False

    def is_safe(self):
        if self.is_safe_flag:
            return SUCCESS

        return FAILURE

    def continue_command_received(self):
        rospy.logdebug('Continue command received')
        return SUCCESS

    def prepare_system(self):
        rospy.logdebug('System prepared')
        return SUCCESS

    def mission_synchronised(self):
        rospy.logdebug('Mission is already synchronised')
        return SUCCESS

    def synchronise_mission(self):
        rospy.logdebug('Mission is synchronised')
        return SUCCESS

    def mark_mission_complete(self):
        self.mission_complete_flag = True

    def mission_complete(self):
        if self.mission_complete_flag:
            rospy.logdebug('Mission is complete')
            return SUCCESS

        rospy.logdebug('Mission is not complete')
        return FAILURE

    def finalised(self):
        rospy.logdebug('Mission is already finalised')
        return SUCCESS

    def finalise(self):
        rospy.logdebug('Finalised mission')
        return SUCCESS

    def idle(self):
        rospy.logdebug('Idling...')
        return RUNNING

    def have_target(self):
        if self.current_target is not None:
            return SUCCESS

        return FAILURE

    def pop_target(self):
        if len(self.setpoint_list)>0:
            self.current_target = self.setpoint_list[0]
            self.setpoint_list = self.setpoint_list[1:]
            rospy.loginfo('Current target:'+str(self.current_target))
            rospy.loginfo('Remaining targets:'+str(self.setpoint_list))
            return SUCCESS

        return FAILURE

    def at_target(self):
        if self.current_target is not None:
            pitch, depth = self.current_target
            if abs(depth - self.depth) < DEPTH_THRESH and abs(pitch - self.pitch) < PITCH_THRESH :
                rospy.loginfo('Reached target:'+str(self.current_target))
                return SUCCESS

        return FAILURE


    def reset_target(self):
        self.current_target = None

    def get_target(self):
        return str(self.current_target)



if __name__ == '__main__':
    rospy.init_node('BT')

    # THESE ARE PITCH,DEPTH
    #  sam = SAM(setpoint_list=[(0.5, 1)])
    sam = SAM(setpoint_list=[(0.5, 1), (0.5, 2)])

    # root of the tree
    root = Seq('Root')

    # actionlib nodes, to be used later
    safety_action = ActionNodeLeaf('sam_emergency', goal='-No goal, just float up-')
    execute_mission = ActionNodeLeaf('sam_sines', goal='3')
    # TODO use this somewhere?
    setpoint_action = ActionNodeLeaf('sam_setpoint', goal_fn=sam.get_target)

    # safety
    safety_check = Fallback('safety_check')
    safety_check.add_child(InstantLeaf('safe?', sam.is_safe))

    # catch the time when safety action succeeds, we dont want anything else to run afterwards
    safety_done_idle = Seq('safety_done_idle')
    safety_done_idle.add_child(InstantLeaf('flag check: safety not tried?', sam.is_safety_action_tried))
    safety_done_idle.add_child(safety_action)
    safety_done_idle.add_child(InstantLeaf('flag set: safety_action_tried_flag', sam.set_safety_action_tried))
    safety_done_idle.add_child(InstantLeaf('idle : safety action SUCCESS', sam.idle))

    safety_check.add_child(safety_done_idle)
    safety_done_idle.add_child(InstantLeaf('flag set: safety_action_tried_flag', sam.set_safety_action_tried))
    safety_check.add_child(InstantLeaf('idle : safety_action FAILURE', sam.idle))

    root.add_child(safety_check)

    # system prep
    #  sys_prep = Fallback('sys_prep')
    #  sys_prep.add_child(InstantLeaf('continue command?', sam.continue_command_received))
    #  sys_prep.add_child(InstantLeaf('prepare system', sam.prepare_system))
    #  root.add_child(sys_prep)

    # mission sync
    #  mission_sync = Fallback('mission_sync')
    #  mission_sync.add_child(InstantLeaf('mission synchronised?', sam.mission_synchronised))
    #  mission_sync.add_child(InstantLeaf('synchronise_mission', sam.synchronise_mission))
    #  root.add_child(mission_sync)

    # mission exec
    mission_exec = Fallback('mission_exec')
    mission_exec.add_child(InstantLeaf('flag check: mission_complete?', sam.mission_complete))



    waypoint_follower = Seq('waypoint follower')
    acquire_target = Fallback('acquire target')
    acquire_target.add_child(InstantLeaf('have target', sam.have_target))
    acquire_target.add_child(InstantLeaf('pop target', sam.pop_target))
    waypoint_follower.add_child(acquire_target)

    follow_target = Fallback('follow target')
    follow_target.add_child(InstantLeaf('at target', sam.at_target))
    follow_target.add_child(setpoint_action)
    waypoint_follower.add_child(follow_target)

    waypoint_follower.add_child(InstantLeaf('reset target', sam.reset_target))


    mission_seq = Seq('mission_seq')
    ############## replace with waypoint follower
    # mission_seq.add_child(execute_mission)
    mission_seq.add_child(waypoint_follower)
    mission_seq.add_child(InstantLeaf('flag set: mark mission complete', sam.mark_mission_complete))

    mission_exec.add_child(mission_seq)
    mission_exec.add_child(InstantLeaf('idle : mission action failed', sam.idle))



    root.add_child(mission_exec)

    # mission finalise
    #  mission_finalise = Fallback('mission_finalise')
    #  mission_finalise.add_child(InstantLeaf('finalised?', sam.finalised))
    #  mission_finalise.add_child(InstantLeaf('finalse', sam.finalise))
    #  root.add_child(mission_finalise)

    # idle in the end to prevent the tree from 'succeeding'
    root.add_child(InstantLeaf('idle : all success!', sam.idle))

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
