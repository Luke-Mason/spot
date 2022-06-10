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
from enum import Enum
from threading import Timer

Status = Enum("Status", "listening idle")

class AudioListenerNode():

  def __init__(self):
    self.listener_type = rospy.get_param("~listener_type", "on_demand")
    translate_topic = rospy.get_param("~translate_topic", "audio/translate")
    audio_topic = rospy.get_param("~audio_topic", "audio")

    if self.listener_type == "on_demand":
      demands_topic = rospy.get_param("~demands_topic", "listen")
      
      # Set the change in context.
      listen_sub = message_filters.Subscriber(demands_topic, String)
      listen_sub.registerCallback(self.start_listening)

    elif self.listener_type == "constant":
      self.samples_to_publish = rospy.get_param("~samples_to_publish", 2) 
      self.samples_to_keep = rospy.get_param("~samples_to_keep", 1) 
      self.sample_rate = rospy.get_param("~sample_rate", 16000)
    else:
      raise Exception("Unknown listener type: " + str(self.listener_type))


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

    self.status = Status.idle

    audio_sub = message_filters.Subscriber(audio_topic, AudioData)
    audio_sub.registerCallback(self.listen_to_audio)

    # Publishers
    self.audio_pub = rospy.Publisher("/" + translate_topic, self.data_class, queue_size=1)
    self.reset_collected_audio()

  def reset_collected_audio(self):
    self.collected_audio = np.ndarray([])

  def start_listening(self, listen: String):

    # Cancel any previous timer.
    self.timer.cancel()

    # Reset any current task it is listening to. 
    # (Useful for if we say awake call during listening for a response to reset listening)
    self.reset_collected_audio()

    # Get the listening duration for the listen type (command, or response etc).
    listen_duration = Listen[listen.data].value
    
    # Publish collected audio after duration.
    self.timer = Timer(float(listen_duration), self.publish_audio)
    self.timer.start()

  def listen_to_audio(self, audio_data: AudioData):
    if self.listener_type == "on_demand":
      if self.status == Status.idle:
        return
      elif self.status == Status.listening:
        self.collected_audio = np.append(self.collected_audio, np.frombuffer(audio_data.data, dtype=self.audio_type))

    elif self.listener_type == "constant":
      self.collected_audio = np.append(self.collected_audio, np.frombuffer(audio_data.data, dtype=self.audio_type))

      if self.collected_audio.size >= self.sample_rate * self.samples_to_publish:
        array = self.publish_audio()
        self.collected_audio = array.data[len(array.data) - (self.samples_to_keep * self.sample_rate):]
    else:
      raise Exception("Unrecognised listener type")

  def publish_audio(self):

    # Stop adding to collected sound
    self.status = Status.idle

    # Package the data to send
    array = self.data_class()
    array.data = self.collected_audio.tolist()

    # Publish audio data to be translated
    self.audio_pub.publish(array)
    self.reset_collected_audio()

    return array


# def main():
if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(1)
  
  rospy.init_node("Audio Listener", log_level=rospy.INFO)
  rospy.loginfo("STARTING AUDIO LISTENER")
  audio = AudioListenerNode()
  rospy.spin()