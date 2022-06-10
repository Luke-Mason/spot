#!/usr/bin/env python3

import string
import time
import rospy
import message_filters
from std_msgs.msg import String
from common import Command, Listen, Say, Task


class TextInterpreter():

  def __init__(self):
    say_topic = rospy.get_param("~say_topic", "say")
    response_topic = rospy.get_param("~response_topic", "response")
    command_topic = rospy.get_param("~command_topic", "command")
    translation_topic = rospy.get_param("~translation_topic", "translation")
    demands_topic = rospy.get_param("~demands_topic", "listen")

    self.heard_words = []

    # Interpret awake command initially.
    self.listener_status = Listen.awake

    # Set the change in context.
    listen_sub = message_filters.Subscriber(demands_topic, String)
    listen_sub.registerCallback(self.set_listen)

    # Acts upon recieving the audio translations.
    translation_sub = message_filters.Subscriber(translation_topic, String)
    translation_sub.registerCallback(self.build_understanding)
    
    # Publishers
    self.command_pub = rospy.Publisher("/" + command_topic, String, queue_size=1)
    self.response_pub = rospy.Publisher("/" + response_topic, String, queue_size=1)
    self.say_pub = rospy.Publisher("/" + say_topic, String, queue_size=1)

  def check_call_called(self, call_phrase):

    # If all words in phrase are found in the heard words in order then return True
    current_index = 0

    for word in call_phrase.split():
      if word not in self.heard_words[current_index:]:
        return False

      current_index = self.heard_words.index(word)
    
    return True      

  def build_understanding(self, translation: String):
    # rospy.loginfo(translation.data)
    self.heard_words.extend(translation.data.split())
    rospy.loginfo(self.heard_words)

    # Check if it is the awake command
    for call_phrase in Task.awake.value:
      result = self.check_call_called(call_phrase)
      if result:
        self.say(Say.im_listening.name)
    
    for call_phrase in Task.find.value:
      result = self.check_call_called(call_phrase)
      if result:
        self.say(Say.ok_searching.name)

    for call_phrase in Task.go_to.value:
      result = self.check_call_called(call_phrase)
      if result:
        self.say(Say.ok_going.name)
    
  
  def say(self, say_type):
    say = String()
    say.data = say_type

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