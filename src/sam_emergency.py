#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# vim:fenc=utf-8


from __future__ import print_function

import rospy
from bee_tea.msg import BTAction, BTFeedback, BTResult
from bee_tea.bt_states import SUCCESS, FAILURE, RUNNING
from bee_tea.bt_action_node import BT_ActionNode

import time, math

from std_msgs.msg import Float64, Header, Bool
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped


class emergency_action(BT_ActionNode):

    #def emergency_cb(self, data):
     #   self.emergency_activated = data.data

    def __init__(self, action_name):
        BT_ActionNode.__init__(self, name=action_name)
        self.done_once = False


    def act(self, goal):
        """
        Return True if action is complete otherwise False
        """
        rospy.loginfo("Emergency action received goal: "+str(goal))


        sam_publisher = rospy.Publisher('/sam_auv_1/thrusters/0/input',
                                        FloatStamped,
                                        queue_size = 100)

        #self.emergency_activated = False
        #sam_sub = rospy.Subscriber('/sam_auv_1/emergency_butt', Bool, self.emergency_cb)

        while not rospy.is_shutdown() and not self.done_once:
            time.sleep(0.5)
            #rospy.loginfo('Emergency waiting for butt')
            #if self.emergency_activated:
             #   rospy.loginfo('Butt activated')
            start_time = rospy.get_time()
            elapsed = 0
            rospy.loginfo('Emergency action active...')
            while elapsed < 3:
                if rospy.is_shutdown():
                    break

                elapsed = rospy.get_time() - start_time
                fs = FloatStamped()
                h = Header()
                fs.header = h
                fs.data = 200
                sam_publisher.publish(fs)
                time.sleep(0.1)
            # we are done doing the action succesfully
            self.done_once = True
            rospy.loginfo('Emergency action SUCCESS')
            return True

        # something went wrong, we fucked up
        rospy.loginfo('Emergency action FAILURE '+ str(self.done_once))
        return False



if __name__=='__main__':

    emergency_action("sam_emergency")
    rospy.spin()

