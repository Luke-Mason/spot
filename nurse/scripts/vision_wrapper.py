#!/usr/bin/env python3

import subprocess
import rospy
import time
import command_line
import os

if __name__ == '__main__':

    # Wait for ROS to start.
    time.sleep(2)
    rospy.init_node("Vision", log_level=rospy.INFO)
    rospy.loginfo("STARTING Vision Service")

    subprocess.run(["python3", "/home/rmitaiil/workspace/aiil_workspace/noetic_workspace/src/spot/nurse/scripts/command_line.py", "192.168.80.3", "webrtc", "save", "--count", "0"])

    rospy.spin()

