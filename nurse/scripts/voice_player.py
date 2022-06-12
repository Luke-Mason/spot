#!/usr/bin/env python3

import random 
import time
import numpy as np
import message_filters
import rospy
from common import Listen, Say
from std_msgs.msg import String
import playsound

class VoicePlayer():

  def __init__(self):
    demands_topic = rospy.get_param("~demands_topic", "listen")
    say_topic = rospy.get_param("~say_topic", "say")
    
    self.demands_pub = rospy.Publisher("/" + demands_topic, String, queue_size=1)

    say_sub = message_filters.Subscriber(say_topic, String)
    say_sub.registerCallback(self.say)

    self.status = Listen.awake.name

  def say(self, say: String):
    if Say[say.data].name == Say.im_listening.name:
      rospy.loginfo("Yes?")
      self.status = Listen.command.name
      self.demands_pub.publish(Listen.command.name)
    if Say[say.data].name == Say.ok_going.name:
      rospy.loginfo("Okay, going now :)")

    if Say[say.data].name == Say.ok_searching.name:
      rospy.loginfo("Okay, searching now :)")

    if Say[say.data].name == Say.stopping.name:
      rospy.loginfo("Stopping")
      self.demands_pub.publish(Listen.awake.name)

    if Say[say.data].name == Say.nothing.name:
      rospy.loginfo("reset")
      self.demands_pub.publish(Listen.awake.name)

    if Say[say.data].name == Say.did_not_understand.name:
      rospy.loginfo("I dont understand, please say it again.")
      self.demands_pub.publish(self.status)

    if not Say[say.data].value:
      num = random.randint(0, len(Say[say.data].value))
      rospy.loginfo("Playing index " + str(num))
      rospy.loginfo("Playing sound " + str(Say[say.data].value[num]))

      playsound.playsound(Say[say.data].value[num], False)



# def main():
if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Voice Player", log_level=rospy.INFO)
  rospy.loginfo("STARTING VOICE PLAYER")
  voice = VoicePlayer()
  rospy.spin()