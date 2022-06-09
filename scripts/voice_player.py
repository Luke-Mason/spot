#!/usr/bin/env python3

import time
import numpy as np
import message_filters
import rospy
from common import Listen, Say
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String, Float32MultiArray, UInt8MultiArray 
from enum import Enum
from threading import Timer


class VoicePlayer():

  def __init__(self):
    self.listener_type = rospy.get_param("~listener_type", "on_demand")

    say_sub = message_filters.Subscriber("/say", String)
    say_sub.registerCallback(self.say)

  def say(self, say: String):
    if Say[say.data].name == Say.im_listening.name:
      rospy.loginfo("Yes?")

# def main():
if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(1)
  
  rospy.init_node("Voice Player", log_level=rospy.INFO)
  rospy.loginfo("STARTING VOICE PLAYER")
  voice = VoicePlayer()
  rospy.spin()