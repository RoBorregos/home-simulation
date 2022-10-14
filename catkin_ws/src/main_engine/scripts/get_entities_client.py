#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import main_engine.msg

def get_entities_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('get_entities_actionserver', main_engine.msg.get_entitiesAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = main_engine.msg.get_entitiesGoal("Go to the living room, grasp the dog_doll and put it under the wagon")

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('get_entitites_client')
        result = get_entities_client()
        print("Result:", ', ',result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)