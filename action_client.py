#! /usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
from actionlib_tutorials.msg import FibonacciAction, FibonacciGoal

# Action Status (primitive)
PENDING = 0
ACTIVE = 1
PREEMPTED = 2
SUCCEEDED = 3
ABORTED = 4
REJECTED = 5
PREEMPTING = 6
RECALLING = 7
RECALLED = 8
LOST = 9

def fb_callback(feedback):
    print (feedback)

# Initializes a rospy node so that the SimpleActionClient can
# publish and subscribe over ROS.
rospy.init_node('fibonacci_client')
rate = rospy.Rate(5)

# Creates the SimpleActionClient, passing the type of the action
# (FibonacciAction) to the constructor.
client = actionlib.SimpleActionClient('fibonacci_action_server', FibonacciAction)

# Waits until the action server has started up and started
# listening for goals.
client.wait_for_server()

# Creates a goal to send to the action server.
goal = FibonacciGoal()
goal.order = 20

print ("==== Sending Goal to Server ====")

# Sends the goal to the action server.
client.send_goal(goal, feedback_cb =fb_callback)

state_result = client.get_state()

while state_result < PREEMPTED:
    # Doing Stuff while waiting for the Server to give a result ....
    rate.sleep()
    state_result = client.get_state()

if state_result == SUCCEEDED:
    rospy.logwarn("Action Done State Result : " + str(client.get_result()))
else:
    rospy.logerr("Something went wrong, result state : " + str(state_result))