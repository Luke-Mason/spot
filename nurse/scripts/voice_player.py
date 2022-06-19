#!/usr/bin/env python3

import time
import message_filters
import rospy
from common import Listen, Say, Sayings
from std_msgs.msg import String
import random
from bosdyn.client.spot_cam.audio import AudioClient
from bosdyn.api.spot_cam import audio_pb2
from bosdyn.client import spot_cam, create_standard_sdk, Robot
import os
import simpleaudio as sa
import pydub

class VoicePlayer():

  def __init__(self):
    demands_topic = rospy.get_param("~demands_topic", "listen")
    say_topic = rospy.get_param("~say_topic", "say")
    self.path_to_media = rospy.get_param("~path_to_media", "media/")
    self.demands_pub = rospy.Publisher("/" + demands_topic, String, queue_size=1)
    self.spot_enabled = rospy.get_param("~spot_enabled", False)
    self.audio_gain = rospy.get_param("~audio_gain", 2)
     
    self.prev_listen_status = Listen.awake

    # Load all sounds onto spot robot
    if self.spot_enabled:
      rospy.loginfo("LOADING SOUNDS INTO SPOT")
      sdk = create_standard_sdk('nurse-spot')
      spot_cam.register_all_service_clients(sdk)
      robot: Robot = sdk.create_robot(os.environ["BOSDYN_CLIENT_IP"])
      robot.authenticate(os.environ["BOSDYN_CLIENT_USERNAME"], os.environ["BOSDYN_CLIENT_PASSWORD"])
      self.audio_client: AudioClient = robot.ensure_client("spot-cam-audio")

      for say_enum in Sayings:
        say: Say = say_enum.value
        for file in say.audio_files:
          path_to_file = self.path_to_media + "/" + file
          sound = audio_pb2.Sound(name=file)
          with open(path_to_file, 'rb') as fh:
              data = fh.read()
          self.audio_client.load_sound(sound, data)
      
      rospy.loginfo("FINISHED LOADING SOUNDS")


    say_sub = message_filters.Subscriber(say_topic, String)
    say_sub.registerCallback(self.say)

  def say(self, saying_name: String):
    rospy.loginfo(saying_name.data)
    say: Say = Sayings[saying_name.data].value

    if len(say.audio_files) > 0:
      num = random.randint(0, len(say.audio_files) - 1)
      file = str(say.audio_files[num])
      path = self.path_to_media + "/" + file

      # Pause the microphone listeners
      self.demands_pub.publish(Listen.paused.name)

      if self.spot_enabled:
        sound = audio_pb2.Sound(name=file)
        self.audio_client.play_sound(sound, self.audio_gain)
        rospy.timer.sleep(1.5)

      else:
        sound = pydub.AudioSegment.from_wav(path)
        playback = sa.play_buffer(
            sound.raw_data, 
            num_channels=sound.channels, 
            bytes_per_sample=sound.sample_width, 
            sample_rate=sound.frame_rate
            )
        playback.wait_done()
        rospy.timer.sleep(0.5)



      # Sleep because listener heard the last bit of audio and caused infinite loop of it talking to itself.

      # Continue listening
      self.demands_pub.publish(self.prev_listen_status.name if say.listen is None else say.listen.name)

    if say.listen is not None:
      self.prev_listen_status = say.listen


if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(2)

  rospy.init_node("Voice Player", log_level=rospy.INFO)
  rospy.loginfo("STARTING VOICE PLAYER")
  voice = VoicePlayer()
  rospy.spin()