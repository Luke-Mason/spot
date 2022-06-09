#!/usr/bin/env python3

import time
import numpy as np
import message_filters
import rospy
from transformers import Wav2Vec2ForCTC, Wav2Vec2Tokenizer
from std_msgs.msg import String, Float32MultiArray


class AudioTranslator():

  def __init__(self):
    self.tokenizer = Wav2Vec2Tokenizer.from_pretrained("facebook/wav2vec2-base-960h")
    self.model = Wav2Vec2ForCTC.from_pretrained("facebook/wav2vec2-base-960h")

    audio_sub = message_filters.Subscriber('/audio/translate', Float32MultiArray)
    audio_sub.registerCallback(self.translate)

    # Publishes translation
    self.translation_pub = rospy.Publisher('/translation', String, queue_size=2)


  def translate(self, array: Float32MultiArray):
    rospy.loginfo(np.shape(array.data))
    input_values = self.tokenizer(array.data, return_tensors="pt").input_values
    logits = self.model(input_values).logits
    predicted_ids = np.argmax(logits.cpu().detach().numpy(), axis=-1)
    transcription = self.tokenizer.batch_decode(predicted_ids)[0]
    
    self.translation_pub.publish(transcription)

    rospy.loginfo("Translation = " + transcription)


# def main():
if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Audio Translator", log_level=rospy.INFO)
  rospy.loginfo("STARTING AUDIO TRANSLATOR")
  audio = AudioTranslator()
  rospy.spin()