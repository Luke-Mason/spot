#!/usr/bin/env python3

import time
import rospy
import message_filters
from std_msgs.msg import String
from common import Listen, Say, Task


class TextInterpreter():

  def __init__(self):

    # Interpret awake command initially.
    self.listener_status = Listen.awake

    # Set the change in context.
    listen_sub = message_filters.Subscriber('listen', String)
    listen_sub.registerCallback(self.set_listen)

    # Acts upon recieving the audio translations.
    translation_sub = message_filters.Subscriber('translation', String)
    translation_sub.registerCallback(self.interpret)
    
    # Publishers
    self.command_pub = rospy.Publisher('/command', String, queue_size=2)
    self.response_pub = rospy.Publisher('/response', String, queue_size=2)
    self.say_pub = rospy.Publisher('/say', String, queue_size=2)

  def interpret(self, translation: String):
    # Check if it is the awake command
    if Task.awake_call.value in translation.data:
      self.say_pub.publish(String(data=Say.im_listening.name))
      return

    # See if there is a task in the audio fragment
    # for task in KnownTasks.all_known_tasks:
    #   try:
    #     move_command_index = translation.index(task)

    #   except ValueError:
    #     pass

  def set_listen(self, listener_status: String):
        self.listener_status = Listen[listener_status.data]




# def main():
if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Audio Text Interpreter", log_level=rospy.INFO)
  rospy.loginfo("STARTING TEXT INTERPRETER")
  audio = TextInterpreter()
  rospy.spin()