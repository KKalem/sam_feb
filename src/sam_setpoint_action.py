#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8


from __future__ import print_function

import rospy
from bee_tea.msg import BTAction, BTFeedback, BTResult
from bee_tea.bt_states import SUCCESS, FAILURE, RUNNING
from bee_tea.bt_action_node import BT_ActionNode

import time, math

from std_msgs.msg import Float64


class setpoint_action(BT_ActionNode):
    def __init__(self, action_name):
        BT_ActionNode.__init__(self, name=action_name)

        self.pitch_publisher = rospy.Publisher('/pitch_setpoint',
                                                Float64,
                                                queue_size = 100)

        self.depth_publisher = rospy.Publisher('/depth_setpoint',
                                                Float64,
                                                queue_size = 100)


    def act(self, goal):
        """
        Return True if action is complete otherwise False
        """
        rospy.loginfo("setpoint action received goal: "+str(goal))


        goal = eval(goal)
        if goal is None:
            return False

        pitch, depth = goal
        publish_for = 0.5

        while not rospy.is_shutdown():
            start_time = rospy.get_time()
            elapsed = 0.0
            while elapsed < publish_for:
                if rospy.is_shutdown():
                    break
                elapsed = rospy.get_time() - start_time
                # publish!
                self.pitch_publisher.publish(pitch)
                self.depth_publisher.publish(depth)

        return True


if __name__=='__main__':

    setpoint_action("sam_setpoint")
    rospy.spin()

