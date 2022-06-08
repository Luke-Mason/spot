#!/usr/bin/env python3

import time
import numpy as np
from AudioClip import AudioClip
from Listen import Listen, ListenerStatus
import message_filters
import rospy
from std_msgs.msg import String, Int16


class AudioTranslator():

  def __init__(self):

    # Initialise listening state is a listening duration just for the awake command.
    self.listener_status: Listen = ListenerStatus.awake

    # Subscribers
    listen_sub = message_filters.Subscriber('listen', Listen)
    listen_sub.registerCallback(self.set_listen)
    
    # Publishers
    self.command_pub = rospy.Publisher('/audio', AudioClip, queue_size=2)

    rate = rospy.Rate(10) # Hz
    while not rospy.is_shutdown:
      


      rate.sleep()


  def set_listen(self, listen_type: Listen):
        self.listener_status = listen_type




if __name__ == '__main__':
  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Audio Listener Node", log_level=rospy.INFO)
  rospy.loginfo("STARTING AUDIO LISTENER NODE")
  audio = AudioTranslator()
  rospy.spin()