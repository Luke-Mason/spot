#!/usr/bin/env python3

import string
import time
import rospy
import message_filters
from std_msgs.msg import String
from common import Listen, Say, Task


class TextInterpreter():

  def __init__(self):
    # self.interpreter_type = rospy.get_param("~interpreter_type", "everything")

    self.heard_words = []

    # Interpret awake command initially.
    self.listener_status = Listen.awake

    # Set the change in context.
    listen_sub = message_filters.Subscriber('listen', String)
    listen_sub.registerCallback(self.set_listen)

    # Acts upon recieving the audio translations.
    translation_sub = message_filters.Subscriber('/translation', String)
    translation_sub.registerCallback(self.build_understanding)
    
    # Publishers
    self.command_pub = rospy.Publisher('/command', String, queue_size=1)
    self.response_pub = rospy.Publisher('/response', String, queue_size=1)
    self.say_pub = rospy.Publisher('/say', String, queue_size=1)

  def build_understanding(self, translation: String):
    # rospy.loginfo(translation.data)
    self.heard_words.extend(translation.data.split())
    rospy.loginfo(self.heard_words)
    # Check if it is the awake command
    if Task.awake_call.value in self.heard_words:
      self.interpret_as_awake()
      return
    
  
  def interpret_as_awake(self):
    say = String()
    say.data = Say.im_listening.name

    # TODO
    self.say_pub.publish(say)
    self.say_pub.publish(say)
    self.say_pub.publish(say)
    self.say_pub.publish(say)
    self.say_pub.publish(say)

    self.heard_words = []

  def set_listen(self, listener_status: String):
        self.listener_status = Listen[listener_status.data]




# def main():
if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Text Interpreter", log_level=rospy.INFO)
  rospy.loginfo("STARTING TEXT INTERPRETER")
  audio = TextInterpreter()
  rospy.spin()