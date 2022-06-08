#!/usr/bin/env python3

import time
import numpy as np
from SPOT.nodes.audio.dto.Listen import Listen
import message_filters
import rospy
from std_msgs.msg import String


class AudioNode():

  def __init__(self):
    self.awake_listen_duration = rospy.get_param("~awake_listen_duration", 2) 
    self.listen_to: Listen = Listen.awake

    # Subscribers
    listen_sub = message_filters.Subscriber('listen', String)
    listen_sub.registerCallback(self.set_listen)
    
    # Publishers
    self.command_pub = rospy.Publisher('/command', String, queue_size=2)
    self.response_pub = rospy.Publisher('/response', String, queue_size=2)
    self.say_pub = rospy.Publisher('/say', String, queue_size=2)

  
  def set_listen(self, listen_type: Listen):
        self.listen_to = listen_type



if __name__ == '__main__':
  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Audio Listener Node", log_level=rospy.INFO)
  rospy.loginfo("STARTING AUDIO NODE")
  audio = AudioNode()
  rospy.spin()