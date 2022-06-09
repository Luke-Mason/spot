#!/usr/bin/env python3

from io import StringIO
from sys import byteorder
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
    audio_sub = message_filters.Subscriber('/audio', AudioData)
    audio_sub.registerCallback(self.listen_to_audio)

    # Publishers
    self.audio_pub = rospy.Publisher('/audio/translate', AudioData, queue_size=2)

    self.sound_data = np.ndarray([])

  def listen_to_audio(self, audio_data: AudioData):
    self.sound_data = np.append(self.sound_data, np.frombuffer(audio_data.data, dtype=np.float32))

    if len(self.sound_data) >= 16000:
      audio = AudioData()
      audio.data = self.sound_data.tobytes()
      self.audio_pub.publish(audio)
      self.sound_data = []

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