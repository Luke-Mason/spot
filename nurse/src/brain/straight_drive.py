#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist

def drive():
    # Drive publisher
    pub_drive = rospy.Publisher('/cmd_vel', Twist)
    cmd = Twist()
    cmd.linear.x = 0
    cmd.linear.y = 0
    cmd.linear.z = 0
    cmd.angular.x = 0
    cmd.angular.y = 0
    cmd.angular.z = 0

    # Paramaters
    speed = rospy.get_param("~speed", 0.1)
    max_duration = rospy.get_param("~duration", 0.1)
    t_start = rospy.Time.now()
    stop = False

    # Periodically publish the drive command to ensure drive continues
    rate = rospy.Rate(10)
    while (not stop) and (not rospy.is_shutdown()):
        cmd.linear.x = speed
        rospy.loginfo(cmd)
        pub_drive.publish(cmd)
        rate.sleep()
        
        # Check time
        t_now = rospy.Time.now()
        duration = t_now - t_start
        if t_now - t_start > rospy.Duration(max_duration):
            stop = True

    # Ensure a stop command is sent
    cmd.linear.x = 0
    pub_drive.publish(cmd)

# Short ROS Node method
if __name__ == '__main__':
    try:
        rospy.init_node('rosbot_straight_drive', anonymous=True)
        drive()
    except rospy.ROSInterruptException:
        pass
