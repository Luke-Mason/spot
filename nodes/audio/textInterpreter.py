#!/usr/bin/env python3

import time
from nodes.audio.knowledgebase.Knowledge import KnownTasks
from voice.knowledgebase.Knowledge import Sayings
from voice.msg.Say import Say
import rospy
import message_filters
from audio.msg.Listen import Listen, ListenerStatus
from audio.msg.Command import Command
from std_msgs.msg import String


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
    self.say_pub = rospy.Publisher('/say', Say, queue_size=2)

  def interpret(self, translation: String):
    
    # Check if it is the awake command
    if KnownTasks.awake_call in translation:
      self.say_pub.publish(Sayings.im_listening)
      return

    # See if there is a task in the audio fragment
    # for task in KnownTasks.all_known_tasks:
    #   try:
    #     move_command_index = translation.index(task)

    #   except ValueError:
    #     pass

  def set_listen(self, listener_status: Listen):
        self.listener_status = listener_status




if __name__ == '__main__':
  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Audio Text Interpreter", log_level=rospy.INFO)
  rospy.loginfo("STARTING TEXT INTERPRETER")
  audio = TextInterpreter()
  rospy.spin()