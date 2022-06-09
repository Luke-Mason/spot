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
from std_msgs.msg import String, Float32MultiArray, UInt8MultiArray 

class AudioListenerNode():

  def __init__(self):
    self.samples_to_publish = rospy.get_param("~samples_to_publish", 2) 
    self.samples_to_keep = rospy.get_param("~samples_to_keep", 1) 
    self.sample_rate = rospy.get_param("~sample_rate", 16000)

    # Dynamic audio type to convert byte audio data into.
    audio_type_str = rospy.get_param("~sample_format", "F32LE")
    if audio_type_str == "F32LE":
      self.audio_type = np.float32
      self.data_class = Float32MultiArray
    elif audio_type_str == "U8":
      self.audio_type = np.uint8
      self.data_class = UInt8MultiArray

    else:
      raise Exception("Unknown audio type: " + str(audio_type_str))


    audio_sub = message_filters.Subscriber('/audio', AudioData)
    audio_sub.registerCallback(self.listen_to_audio)

    # Publishers
    self.audio_pub = rospy.Publisher('/audio/translate', Float32MultiArray, queue_size=2)

    self.sound_data = np.ndarray([])

  def listen_to_audio(self, audio_data: AudioData):

    self.sound_data = np.append(self.sound_data, np.frombuffer(audio_data.data, dtype=self.audio_type))

    if len(self.sound_data) >= self.sample_rate * self.samples_to_publish:
      # Package the data to send
      array = self.data_class()
      array.data = self.sound_data

      # Publish audio data to be translated
      self.audio_pub.publish(array)

      self.sound_data = array.data[len(self.sound_data) - (self.samples_to_keep * self.sample_rate):]

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