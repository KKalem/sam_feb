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


class sine_action(BT_ActionNode):
    def act(self, goal):
        """
        Return True if action is complete otherwise False
        """
        rospy.loginfo("Sine action received goal: "+str(goal))

        # TODO make this useful
        sam_publisher = rospy.Publisher('/pitch_setpoint',
                                        Float64,
                                        queue_size = 100)

        try:
            goal = float(goal)
        except:
            goal = 0.5

        publish_for = 0.5

        while not rospy.is_shutdown():
            start_time = rospy.get_time()
            elapsed = 0.0
            while elapsed < publish_for:
                if rospy.is_shutdown():
                    break
                elapsed = rospy.get_time() - start_time
                # publish!
                sam_publisher.publish(goal)

        return True


if __name__=='__main__':

    sine_action("sam_setpoint")
    rospy.spin()

