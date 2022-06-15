#!/usr/bin/env python

import rospy
from spot_msgs.msg import NavigateToActionGoal, NavigateToGoal, NavigateToAction
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header
import actionlib
import time


class Spot_Nav():

    def __init__(self):
        rospy.init_node('send_goal')

    # Callbacks definition
    def active_cb(self):
        rospy.loginfo("Goal pose being processed")

    def feedback_cb(self,feedback):
        rospy.loginfo("Current location: "+str(feedback))

    def done_cb(self, status, result):
        if status == 3:
            rospy.loginfo("\n\nGoal reached\n\n")
            return "Done"
        if status == 2 or status == 8:
            rospy.loginfo("Goal cancelled")
        if status == 4:
            rospy.loginfo("Goal aborted")


    def send_to_goal(self):
        # Connecting to the nav client server.
        navclient = actionlib.SimpleActionClient('/spot/navigate_to', NavigateToAction)
        navclient.wait_for_server()

        # Creating the goal object
        navigate_goal = NavigateToGoal()
        navigate_goal.upload_path = "/home/ramji/Desktop/map_folder/downloaded_graph"
        navigate_goal.navigate_to = "hm"
        navigate_goal.initial_localization_fiducial= False
        navigate_goal.initial_localization_waypoint = "tb"

        # SENDING THE GOAL

        navclient.send_goal(navigate_goal, self.done_cb, self.active_cb, self.feedback_cb)
        finished = navclient.wait_for_result()

        if not finished:
            rospy.logerr("Action server not available!")
        else:
            rospy.loginfo (navclient.get_result())


if __name__=="__main__":
    nav = Spot_Nav()
    nav.send_to_goal()