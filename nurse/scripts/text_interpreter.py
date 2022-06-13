#!/usr/bin/env python3

import time
import rospy
import message_filters
from std_msgs.msg import String
from common import Command, Commands, Listen, Sayings, Task


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

  def check_task_matches(self, task: Task, heard_words):
    for phrase in task.value:
      if self.check_phrase_called(phrase, heard_words):
        return True
    
    return False
  
  def check_phrase_called(self, phrase: str, heard_words):

    # If all words in phrase are found in the heard words in order then return True
    current_index = 0

    # Check that the series of words are all existent in order. (This allows for substrings to be matches as well)
    for sub_word in phrase.split():
      found = [word for word in heard_words[current_index:] if sub_word in word]
      if not found:
        return False
    
      current_index = heard_words.index(found[-1])
    return True      

  def build_understanding(self, translation: String):
    if len(translation.data) == 0:
      self.say_pub.publish(Sayings.nothing.name)
      return

    heard_words = translation.data.split()

    if self.listener_status == Listen.awake:

      # Check if it is the awake command
      if self.check_task_matches(Task.awake, heard_words):
        self.say_pub.publish(Sayings.im_listening.name)
      
      return

    elif self.listener_status == Listen.command:
      
      # Check all commands for matching words then run that command if found.
      for command_enum in Commands:
        command: Command = command_enum.value
        if self.check_task_matches(command.get_task_enum(), heard_words):
          self.command_pub.publish(command_enum.name)
          return

      self.say_pub.publish(Sayings.i_do_not_understand.name)

    elif self.listener_status == Listen.response:
      rospy.loginfo("Check for a valid response")
      self.say_pub.publish(Sayings.i_do_not_understand.name)
    
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