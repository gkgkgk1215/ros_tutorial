#! /usr/bin/env python

import rospy
import actionlib
from actionlib_tutorials.msg import FibonacciFeedback, FibonacciResult, FibonacciAction


# create messages that are used to publish feedback/result
g_feedback = FibonacciFeedback()
g_result = FibonacciResult()

def execute_cb(goal):
    # helper variables
    r = rospy.Rate(3)
    success = True

    # append the seeds for the fibonacci sequence
    g_feedback.sequence = []
    g_feedback.sequence.append(0)
    g_feedback.sequence.append(1)

    # publish info to the console for the user
    rospy.loginfo('Fibonacci Action Server Executing, creating fibonacci sequence of order %i with seeds %i, %i'
                  % (goal.order, g_feedback.sequence[0], g_feedback.sequence[1]))

    # start executing the action
    for i in range(1, goal.order):
        # check that preempt has not been requested by the client
        if _as.is_preempt_requested():
            rospy.loginfo("The goal has been cancelled/preempted")
            _as.set_preempted()
            success = False
            break

        g_feedback.sequence.append(g_feedback.sequence[i] + g_feedback.sequence[i - 1])
        # publish the feedback
        _as.publish_feedback(g_feedback)
        # this step is not necessary, the sequence is computed at 1 Hz for demonstration purposes
        r.sleep()

    if success:
        g_result.sequence = g_feedback.sequence
        rospy.loginfo("Succeeded calculating the Fibonacci")
        _as.set_succeeded(g_result)

rospy.init_node("fibonacci")
_as = actionlib.SimpleActionServer("fibonacci_action_server", FibonacciAction, execute_cb=execute_cb, auto_start=False)
_as.start()

print ("==== Waiting for Client Goal ... ====")

rospy.spin()