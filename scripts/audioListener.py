#!/usr/bin/env python3

import time
import numpy as np
import message_filters
import rospy
# import sounddevice as sd
from common import Listen
from audio_common_msgs.msg import AudioData
from std_msgs.msg import String

class AudioListenerNode():

  def __init__(self):
    fs = rospy.get_param("~samplerate", 16000) 


    # Initialise listening state is a listening duration just for the awake command.
    self.attention_span = Listen.awake.value

    # Subscribers
    listen_sub = message_filters.Subscriber('listen', String)
    listen_sub.registerCallback(self.set_attention_span)
    
    audio_sub = message_filters.Subscriber('audio/audio', AudioData)
    audio_sub.registerCallback(self.listen_to_audio)

    # Publishers
    self.audio_pub = rospy.Publisher('/audio/translate', AudioData, queue_size=2)

    self.sound_data = []

  def listen_to_audio(self, audio_data: AudioData):
    # segment_duration = self.listener_status.duration / 2
    # curr_sound_data = sd.rec(segment_duration * fs, samplerate=fs, channels=1, dtype='int')
    # sd.wait()

    # Combine with previous data
    input_audio = np.append(self.sound_data, audio_data.data)


    # Default listening for awake command
    if self.attention_span == 0:

      # Update previous with current
      self.sound_data = audio_data.data
      
      # Publish the audio data
      self.audio_pub.publish(AudioData(data=input_audio))

    else:

      if self.attention_span == 1:
        self.audio_pub.publish(AudioData(data=input_audio))

        # Set to last clip incase it chops off a "HEY" from "Hey spot" to cancel or something
        self.sound_data = audio_data.data
      else:
        self.sound_data = input_audio

      self.attention_span -= 1




  def set_attention_span(self, listen: String):
        self.attention_span = Listen[listen.data].value


# def main():
if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Audio Listener", log_level=rospy.INFO)
  rospy.loginfo("STARTING AUDIO LISTENER")
  audio = AudioListenerNode()
  rospy.spin()