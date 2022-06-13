#!/usr/bin/env python3

import string
import time
import rospy
import message_filters
from std_msgs.msg import String
from common import Command, Listen, Say, Calls, SayQuestionYes


class TextInterpreter():

  def __init__(self):
    say_topic = rospy.get_param("~say_topic", "say")
    response_topic = rospy.get_param("~response_topic", "response")
    command_topic = rospy.get_param("~command_topic", "command")
    translation_topic = rospy.get_param("~translation_topic", "translation")
    demands_topic = rospy.get_param("~demands_topic", "listen")
    
    self.max_words = rospy.get_param("~max_words", 50)

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

  def check_call_called(self, call_phrase, heard_words):

    # If all words in phrase are found in the heard words in order then return True
    current_index = 0

    # Check that the series of words are all existent in order.
    for word in call_phrase.split():
      if word not in heard_words[current_index:]:
        return False

      current_index = heard_words.index(word)
    
    return True      

  def build_understanding(self, translation: String):
    if len(translation.data) == 0:
      self.say_pub.publish(Say.nothing.name)
      return

    # self.heard_words.extend(translation.data.split())
    heard_words = translation.data.split()

    if self.listener_status == Listen.awake:

      # Check if it is the awake command
      for call_phrase in Calls.awake.value:
        result = self.check_call_called(call_phrase, heard_words)
        if result:
          self.say_pub.publish(SayQuestionYes().name)
          return

    elif self.listener_status == Listen.command:

      for call_phrase in Calls.stop.value:
        result = self.check_call_called(call_phrase, heard_words)
        if result:
          return
 
      for call_phrase in Calls.find.value:
        result = self.check_call_called(call_phrase, heard_words)
        if result:

          return

      for call_phrase in Calls.go_to.value:
        result = self.check_call_called(call_phrase, heard_words)
        if result:
          return

    elif self.listener_status == Listen.response:
      for call_phrase in Calls.go_to.value:
        result = self.check_call_called(call_phrase, heard_words)
        if result:
          return
    
    self.say_pub.publish(Say.did_not_understand.name)
      

    # if len(self.heard_words) > self.max_words:
    #   self.heard_words = self.heard_words[len(self.heard_words) - self.max_words:]
    
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