#!/usr/bin/env python3

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
    self.listener = Listen.awake
    translate_topic = rospy.get_param("~translate_topic", "translate")
    audio_topic = rospy.get_param("~audio_topic", "audio")
    demands_topic = rospy.get_param("~demands_topic", "listen")

    self.threshold_boost_multiplier = rospy.get_param("~threshold_boost_multiplier", 1.5)
    self.idle_wait_time = rospy.get_param("~idle_wait_time", 0.5)
    self.sample_rate = rospy.get_param("~sample_rate", 16000)
    
    # Setup the threshold value dynamically by listening to the environment
    threshold_listen_duration = rospy.get_param("~threshold_listen_duration", 2)

    rospy.loginfo("Setting up audio threshold ...")
    self.threshold_setup = False
    self.timer = Timer(threshold_listen_duration, self.setup_threshold)
    self.timer.start()


    # Set the change in context.
    listen_sub = message_filters.Subscriber(demands_topic, String)
    listen_sub.registerCallback(self.start_listening_to_demand)

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

  def setup_threshold(self):
    self.threshold = np.max(np.absolute(self.collected_audio)) * self.threshold_boost_multiplier
    rospy.loginfo("Threshold set to: " + str(self.threshold))
    self.reset_collected_audio()
    self.threshold_setup = True

  def start_listening_to_demand(self, listen: String):
    
    # Cancel any previous timer.
    # Reset any current task it is listening to. 
    # (Useful for if we say awake call during listening for a response to reset listening)
    self.cancel_timer()
    self.reset_collected_audio()
    self.listener = Listen[listen.data]
    self.status = Status.idle

  def cancel_timer(self):
    if self.timer is not None:
      self.timer.cancel()
      self.timer.join()

  def listen_to_audio(self, audio_data: AudioData):
    data = np.frombuffer(audio_data.data, dtype=self.audio_type)
    collected_data = np.append(self.collected_audio, data)
    
    if self.threshold_setup == False or self.listener == Listen.paused:
      return

    max_samples = self.listener.value

    # Start/Continue listening if the audio is above threshold.
    if np.max(np.absolute(data)) >= self.threshold:
      if self.status == Status.idle:
        self.status = Status.listening
        rospy.loginfo("LISTENING")

      # Publish collected audio after duration.
      self.cancel_timer()
      self.timer = Timer(self.idle_wait_time, self.publish_audio)
      self.timer.start()

    # Collect data as we are now in listening mode.
    if self.status == Status.listening:
      self.collected_audio = collected_data

    # Publish max recording (Stops recording forever if audio constantly above threshold).
    if self.status == Status.listening and self.collected_audio.size >= self.sample_rate * max_samples:
      # rospy.loginfo(self.collected_audio.size)
      self.publish_audio()

  def publish_audio(self):
    if self.timer is not None:
      self.timer.cancel()

    # Check that the collected audio is a factor of the sample rate, filling missing slots with 0
    remainder = len(self.collected_audio) % self.sample_rate
    if remainder > 0:
      left_over_slots = self.sample_rate - remainder
      self.collected_audio = np.concatenate((self.collected_audio, np.zeros(left_over_slots, dtype=self.audio_type)))
    
    # rospy.loginfo(len(self.collected_audio))

    # Package the data to send
    array = self.data_class()
    array.data = list(self.collected_audio.tolist())

    # Publish audio data to be translated
    self.audio_pub.publish(array)
    self.reset_collected_audio()

    # Set back to constant if changed
    self.status = Status.idle
    rospy.loginfo("IDLE")


# def main():
if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(1)
  
  rospy.init_node("Audio Listener", log_level=rospy.INFO)
  rospy.loginfo("STARTING AUDIO LISTENER")
  audio = AudioListenerNode()
  rospy.spin()