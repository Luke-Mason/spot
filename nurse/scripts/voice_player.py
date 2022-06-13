#!/usr/bin/env python3

import time
import message_filters
import rospy
from common import Say
from std_msgs.msg import String

class VoicePlayer():

  def __init__(self):
    demands_topic = rospy.get_param("~demands_topic", "listen")
    say_topic = rospy.get_param("~say_topic", "say")
    self.path_to_media = rospy.get_param("~path_to_media", "media/")

    self.demands_pub = rospy.Publisher("/" + demands_topic, String, queue_size=1)

    say_sub = message_filters.Subscriber(say_topic, String)
    say_sub.registerCallback(self.say)

  def say(self, say_name: String):
    Say[say_name.data].perform(self.demands_pub)


# def main():
if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Voice Player", log_level=rospy.INFO)
  rospy.loginfo("STARTING VOICE PLAYER")
  voice = VoicePlayer()
  rospy.spin()