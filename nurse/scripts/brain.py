#!/usr/bin/env python3

import time
import message_filters
import rospy
from common import Commands 
from std_msgs.msg import String



class Brain():

  def __init__(self):
    response_topic = rospy.get_param("~response_topic", "response")
    command_topic = rospy.get_param("~command_topic", "command")
    say_topic = rospy.get_param("~say_topic", "say")

    self.say_pub = rospy.Publisher("/" + say_topic, String, queue_size=1)

    response_sub = message_filters.Subscriber(response_topic, String)
    response_sub.registerCallback(self.recieved_response)

    command_sub = message_filters.Subscriber(command_topic, String)
    command_sub.registerCallback(self.recieved_command)

  def recieved_command(self, command: String):

    self.say_pub.publish(Commands[command.data].value.get_say().name)

  def recieved_response(self, response: String):
    # self.say_pub.publish(Response[response.data].value.get_say().name)
    rospy.loginfo("recieved response")



# def main():
if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Brain", log_level=rospy.INFO)
  rospy.loginfo("STARTING BRAIN")
  voice = Brain()
  rospy.spin()