#! /usr/bin/env python

import rospy
from datetime import datetime
from geometry_msgs.msg import Twist

# Creating the node
rospy.init_node('move1')

# Instantiating the rate to sleep
rate = rospy.Rate(200)


# Creating the twist message
twist = Twist()
twist.linear.x = -0.4


# Creating the publisher
publisher = rospy.Publisher('/spot/cmd_vel',Twist,queue_size=1)


time_now = datetime.now()
while not rospy.is_shutdown():
	
    if (datetime.now() - time_now).seconds <2:
    	publisher.publish(twist)
    else:
	break

    rate.sleep()
     
