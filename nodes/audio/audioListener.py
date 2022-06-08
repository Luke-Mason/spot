#!/usr/bin/env python3

import time

import numpy as np
from AudioClip import AudioClip
from Listen import Listen, ListenerStatus
import message_filters
import rospy
import sounddevice as sd


class AudioListenerNode():

  def __init__(self):


    # Initialise listening state is a listening duration just for the awake command.
    self.listener_status: Listen = ListenerStatus.awake

    # Subscribers
    listen_sub = message_filters.Subscriber('listen', Listen)
    listen_sub.registerCallback(self.set_listen)
    
    # Publishers
    self.audio_pub = rospy.Publisher('/audio', AudioClip, queue_size=2)

    fs=16000
    prev_sound_data = []

    while not rospy.is_shutdown:
      segment_duration = self.listener_status.duration / 2
      curr_sound_data = sd.rec(segment_duration * fs, samplerate=fs, channels=1, dtype='int')
      sd.wait()

      # Combine with previous data
      input_audio = np.append(prev_sound_data, curr_sound_data)

      # Update previous with current
      prev_sound_data = curr_sound_data
      
      # Publish the audio data
      self.audio_pub.publish(AudioClip(fs, input_audio))


  def set_listen(self, listen_type: Listen):
        self.listener_status = listen_type




if __name__ == '__main__':
  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Audio Listener Node", log_level=rospy.INFO)
  rospy.loginfo("STARTING AUDIO LISTENER NODE")
  audio = AudioListenerNode()
  rospy.spin()