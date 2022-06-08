#!/usr/bin/env python3

import time
import message_filters
from nodes.audio.msg.AudioClip import AudioClip
from nodes.audio.msg.Listen import ListenerStatus, Listen
import rospy
from transformers import Wav2Vec2ForCTC, Wav2Vec2Tokenizer
import torch
from std_msgs.msg import String


class AudioTranslator():

  def __init__(self):
    self.tokenizer = Wav2Vec2Tokenizer.from_pretrained("facebook/wav2vec2-base-960h")
    self.model = Wav2Vec2ForCTC.from_pretrained("facebook/wav2vec2-base-960h")

    # Initialise listening state is a listening duration just for the awake command.
    self.listener_status = ListenerStatus.awake

    # Subscribers
    listen_sub = message_filters.Subscriber('listen', Listen)
    listen_sub.registerCallback(self.set_listen)
    audio_sub = message_filters.Subscriber('/audio', AudioClip)
    audio_sub.registerCallback(self.translate)

    # Publishes translation
    self.translation_pub = rospy.Publisher('/translation', String, queue_size=2)


  def translate(self, audioClip: AudioClip):
    input_values = self.tokenizer(audioClip.data, return_tensors="pt").input_values
    logits = self.model(input_values).logits
    predicted_ids = torch.argmax(logits, dim=-1)
    transcription = self.tokenizer.batch_decode(predicted_ids)[0]
    
    self.translation_pub.publish(transcription)

    print(transcription)
    rospy.loginfo(transcription)

  def set_listen(self, listen_type: Listen):
        self.listener_status = listen_type


if __name__ == '__main__':
  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Audio Translator", log_level=rospy.INFO)
  rospy.loginfo("STARTING AUDIO TRANSLATOR")
  audio = AudioTranslator()
  rospy.spin()