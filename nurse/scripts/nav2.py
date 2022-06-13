#! /usr/bin/env python

# from bosdyn.client.graph_nav import GraphNavClient
import rospy
from spot_msgs.msg import NavigateToActionGoal, NavigateToGoal, NavigateToAction
from actionlib_msgs.msg import GoalID
from std_msgs.msg import Header
import actionlib

class Waypoint():
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/spot/navigate_to', NavigateToAction)
        self.timeout = rospy.get_param("~timeout", 300)
        # self.travel_params = GraphNavClient().generate_travel_params(100,100,0.5)

    def execute(self):
        
        # Ensure move_base action client server is available
        rospy.loginfo('Connecting to navigate_to...')
        timer = self.client.wait_for_server(rospy.Duration(self.timeout))
        if not timer:
            rospy.logerr("Could not connect to navigation server, terminating")
            return
        rospy.loginfo('Connected to navigate_to.')
        
        # CREATING THE GOAL
        # Creating the header object
        header = Header()
        header.frame_id = '/spot/odometry'

        # Creating the GoalID object
        goal_id = GoalID()
        goal_id.id='unique'

        # Creating the NavigateToGoal object
        navigate_goal = NavigateToGoal()
        navigate_goal.upload_path = "/home/ramji/Desktop/map_folder/downloaded_graph"
        navigate_goal.navigate_to = "mb"
        navigate_goal.initial_localization_fiducial= False
        navigate_goal.initial_localization_waypoint = "uc"

        # Creating the object to navigate to the action goal.
        nav_goal = NavigateToActionGoal()
        nav_goal.header = header
        nav_goal.goal_id = goal_id
        nav_goal.goal = NavigateToGoal()
        nav_goal.goal.upload_path = '/home/ramji/Desktop/map_folder/downloaded_graph'
        # SENDING THE GOAL
        self.client.send_goal(navigate_goal)

        # Iterate through waypoints
        done = False
        while not done:
            # Wait for result
            timer = self.client.wait_for_result(rospy.Duration(self.timeout))
            if timer:
                rospy.loginfo("Complete goal - may not have arrived - may be blocked")
                rospy.loginfo("\tContinuing...")
            else :
                rospy.logwarn("Wait for move_base complete timed-out")
                state = self.client.get_state()
                rospy.logwarn("Current move_base state:" + str(state))
                rospy.logwarn("\tContinuing to next waypoint...")
                self.client.cancel_goal()
                done = True
                
        # Execute is complete
        return 'completed'
# Short ROS Node method
try:
    rospy.init_node('rosbot_waypoint', anonymous=True)
    
    wp = Waypoint()
    wp.execute()
except rospy.ROSInterruptException:
    pass
