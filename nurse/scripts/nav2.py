#!/usr/bin/env python

import rospy
from spot_msgs.msg import NavigateToActionGoal, NavigateToGoal, NavigateToAction
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header
import actionlib
import time

# Callbacks definition

def active_cb():
    rospy.loginfo("Goal pose being processed")

def feedback_cb(feedback):
    rospy.loginfo("Current location: "+str(feedback))

def done_cb(status, result):
    if status == 3:
        rospy.loginfo("\n\nGoal reached\n\n")
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")
    

rospy.init_node('send_goal')

navclient = actionlib.SimpleActionClient('/spot/navigate_to', NavigateToAction)
navclient.wait_for_server()

# CREATING THE GOAL

# Creating the NavigateToGoal object
navigate_goal = NavigateToGoal()
navigate_goal.upload_path = "/home/ramji/Desktop/map_folder/downloaded_graph"
navigate_goal.navigate_to = "um"
navigate_goal.initial_localization_fiducial= False
navigate_goal.initial_localization_waypoint = "ur"

# SENDING THE GOAL

navclient.send_goal(navigate_goal, done_cb, active_cb, feedback_cb)
finished = navclient.wait_for_result()

if not finished:
    rospy.logerr("Action server not available!")
else:
    rospy.loginfo (navclient.get_result())