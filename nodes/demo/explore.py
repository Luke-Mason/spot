#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from datetime import datetime
from spot_msgs.msg import Feedback

# Creating the node
rospy.init_node('move')


# Creating the twist message to move the robot forwards
forward = Twist()
forward.linear.x = 0.5


# Instantiating the rate to sleep
rate = rospy.Rate(200)

motion_list = []

# The callback function for feedback subscriber
def catch_feedback(data):
    global forward, motion_list, time_now
    
    # If there is an obstacle in front, rotate left
    if data.moving == False:
        forward.angular.z = 0.5
        forward.linear.x = 0
        print("Rotating")
        
    # Else keep moving forward
    else:
        forward.angular.z = 0
        forward.linear.x = 0.5
        

    if (datetime.now() - time_now).seconds >3:
        print("Checking after three seconds")
        number_of_false = motion_list.count(1)
        
    try:
        print("False_rate",number_of_false/len(motion_list))
    except:
        print("ZeroDiv")

  
    if number_of_false>3:
        print("Robot stuck")
 
time_now = datetime.now()

# Creating the feedback subscriber
subscriber = rospy.Subscriber('/spot/status/feedback',Feedback,catch_feedback)

# Creating the publisher
publisher = rospy.Publisher('/spot/cmd_vel',Twist,queue_size=1)

while not rospy.is_shutdown():
    
#    if forward.angular.z>0:
#       print("Rotating")
    
    publisher.publish(forward)

    #if (datetime.now() - time_now).seconds >5:
    #print(motion_list)
    

    rate.sleep()


