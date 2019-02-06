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

        # TODO set to somethings that make sense
        a = 1
        w = 1
        offset = 0

        time_per_sine = 10

        # TODO make this useful
        sam_publisher = rospy.Publisher('/sam_auv_1/joint1_position_controller/command',
                                        Float64)
        # some sub if needed
        # function should take the message and do something with it
        # callback should do stuff like:
            # self.target_set_whatever = data.data.sine_value
        #  sine_sub = rospy.Subscriber('/topic/here', message_name_here, callback_function_here)

        # hack until we have a real tree that sends integers and not 's'es
        if goal == 's':
            goal = '3'
        else:
            goal = '10'
        # </hack>

        num_sines_todo = int(goal)

        while not rospy.is_shutdown() or (num_sines_todo > 0):
            num_sines_todo -= 1
            start_time = rospy.get_time()
            elapsed = 0.0
            while elapsed < time_per_sine:
                elapsed = rospy.get_time() - start_time
                sine_elapsed = a * (math.sin(w * elapsed) + offset)
                # publish!
                sam_publisher.publish(sine_elapsed)
                rospy.loginfo("Sine elapsed:"+str(sine_elapsed)+" sine_todo:"+str(num_sines_todo))
                time.sleep(0.5)

        # return True 
        if num_sines_todo == 0:
            return True




if __name__=='__main__':

    sine_action("test")
    rospy.spin()

