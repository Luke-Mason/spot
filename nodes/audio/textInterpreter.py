#!/usr/bin/env python3

import time
import numpy as np
from Command import Command
from Listen import Listen, ListenerStatus
import message_filters
import rospy
from std_msgs.msg import String, Int16


class TextInterpreter():

  def __init__(self):

    # Interpret awake command initially.
    self.listener_status: Listen = ListenerStatus.awake

    # Set the change in context.
    listen_sub = message_filters.Subscriber('listen', ListenerStatus)
    listen_sub.registerCallback(self.set_listen)

    # Acts upon recieving the audio translations.
    translation_sub = message_filters.Subscriber('translation', String)
    translation_sub.registerCallback(self.interpret)
    
    # Publishers
    self.command_pub = rospy.Publisher('/command', Command, queue_size=2)
    self.response_pub = rospy.Publisher('/response', String, queue_size=2)
    self.say_pub = rospy.Publisher('/say', String, queue_size=2)

  def interpret(self, translation: String):
    # Check if it is the awake command
    if translation == KnownTasks.awake_call:


    try:
      move_command_index = translation.index(find)

    except ValueError:
      pass

  def set_listen(self, listener_status: Listen):
        self.listener_status = listener_status




if __name__ == '__main__':
  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Audio Listener Node", log_level=rospy.INFO)
  rospy.loginfo("STARTING AUDIO NODE")
  audio = AudioNode()
  rospy.spin()