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
    demands_topic = rospy.get_param("~demands_topic", "listen")
    say_topic = rospy.get_param("~say_topic", "say")
    
    self.demands_pub = rospy.Publisher("/" + demands_topic, String, queue_size=1)


    say_sub = message_filters.Subscriber(say_topic, String)
    say_sub.registerCallback(self.say)

  def say(self, say: String):
    rospy.loginfo("HELLOOOOOOOO")
    rospy.loginfo("Yes? " + say.data)
    rospy.loginfo(Say[say.data].name)
    if Say[say.data].name == Say.im_listening.name:
      rospy.loginfo("Yes?")
      self.demands_pub.publish(Listen.command.name)

# def main():
if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(1)
  
  rospy.init_node("Voice Player", log_level=rospy.INFO)
  rospy.loginfo("STARTING VOICE PLAYER")
  voice = VoicePlayer()
  rospy.spin()