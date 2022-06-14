#!/usr/bin/env python3

import time
from types import SimpleNamespace
import message_filters
from noetic_workspace.src.spot.nurse.scripts.audio import AudioLoadSoundCommand
from noetic_workspace.src.spot.nurse.scripts.robot import Spot
import rospy
from common import Listen, Say, Sayings
from std_msgs.msg import String

class VoicePlayer():

  def __init__(self):
    demands_topic = rospy.get_param("~demands_topic", "listen")
    say_topic = rospy.get_param("~say_topic", "say")
    self.path_to_media = rospy.get_param("~path_to_media", "media/")
    self.demands_pub = rospy.Publisher("/" + demands_topic, String, queue_size=1)
    self.spot_enabled = rospy.get_param("~spot_enabled", False)

    say_sub = message_filters.Subscriber(say_topic, String)
    say_sub.registerCallback(self.say)
    self.prev_listen_status = Listen.awake

    # Load all sounds onto spot robot
    if self.spot_enabled:
      command = AudioLoadSoundCommand()
      for say_enum in Sayings:
        say: Say = say_enum.value
        for file in say.audio_files:
          path_to_file = self.path_to_media + "/" + file
          # command._run(robot, SimpleNamespace(name=file, src=path_to_file))
        return

  def say(self, saying_name: String):
    rospy.loginfo(saying_name.data)
    say: Say = Sayings[saying_name.data].value
    say.run(self.path_to_media, self.demands_pub, self.prev_listen_status, self.spot_enabled)
    if say.get_listen() is not None:
      self.prev_listen_status = say.get_listen()


if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(2)

  rospy.init_node("Voice Player", log_level=rospy.INFO)
  rospy.loginfo("STARTING VOICE PLAYER")
  voice = VoicePlayer()
  rospy.spin()