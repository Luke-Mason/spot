#!/usr/bin/env python3

import time
import message_filters
import rospy
from common import Command, Commands 
from std_msgs.msg import String
from spot_msgs.msg import NavigateToGoal, NavigateToAction
import actionlib
import time
import subprocess


class Brain():

  def __init__(self):
    response_topic = rospy.get_param("~response_topic", "response")
    command_topic = rospy.get_param("~command_topic", "command")
    say_topic = rospy.get_param("~say_topic", "say")
    self.graph_upload_path = rospy.get_param("~graph_upload_path", "/home/rmitaiil/Desktop/map_folder/downloaded_graph")
    self.initial_localization_waypoint = rospy.get_param("~initial_localization_waypoint", "nc")
    self.navigate_to = rospy.get_param("~navigate_to", "rp")

    self.say_pub = rospy.Publisher("/" + say_topic, String, queue_size=1)

    response_sub = message_filters.Subscriber(response_topic, String)
    response_sub.registerCallback(self.recieved_response)

    command_sub = message_filters.Subscriber(command_topic, String)
    command_sub.registerCallback(self.recieved_command)

    self.robot_status = 0

  def spin_camera(self):
    for i in range(0,360,15):
      subprocess.run(["python3", "/home/rmitaiil/workspace/aiil_workspace/noetic_workspace/src/spot/nurse/scripts/command_line.py", "192.168.80.3", "ptz", "set_position", "mech", str(i), "0", "0"])



  def recieved_command(self, command_enum: String):
    """
    Recieves the command and then calls the run code for that command, futher command specifics are run after with one to one mapping.
    """
    command: Command = Commands[command_enum.data].value
    command.run(self.say_pub)

    # Runs code for go_room command specifically.
    if command_enum.data == Commands.go_room.name:
      # Sends spot to goal wiht graph nav, and calls done call back.
      self.send_to_goal(self.initial_localization_waypoint, self.navigate_to, self.done_cb)

    # Runs code for the find command specifically.
    if command_enum.data == Commands.find.name:
      # TODO Make the vision element start processing.
      # subprocess.run(["roslaunch", "/home/rmitaiil/workspace/aiil_workspace/noetic_workspace/src/spot/nurse/launch/vision.launch"])
      self.spin_camera()

  def recieved_response(self, response: String):
    # TODO allow for spot to ask questions and expect reponses, and so handle them here.
    # self.say_pub.publish(Response[response.data].value.get_say().name)
    rospy.loginfo("recieved response")

  # Callbacks definition
  def active_cb(self):
      rospy.loginfo("Goal pose being processed")

  def feedback_cb(self,feedback):
      rospy.loginfo("Current location: " + str(feedback))

  def do_nothing(self, status, result):
    self.status = status
    # TODO Refactor duplicated code.
    if status == 3:
        rospy.loginfo("\n\nGoal reached\n\n")
        return "Done"
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")


  def done_cb(self, status, result):
    """
    Done call back that then sends spot back to the position it came from and then does nothing more.
    """
    self.status = status
    if status == 3:
        rospy.loginfo("\n\nGoal reached\n\n")
        # TODO Start the vision face detection process.
        # subprocess.run(["roslaunch", "/home/rmitaiil/workspace/aiil_workspace/noetic_workspace/src/spot/nurse/launch/vision.launch"])
        #subprocess.run(["python3", "/home/rmitaiil/workspace/aiil_workspace/noetic_workspace/src/spot/nurse/scripts/command_line.py", "192.168.80.3", "webrtc", "save", "--count", "0"])
        self.spin_camera()
        self.send_to_goal(self.navigate_to, self.initial_localization_waypoint, self.do_nothing)
        return "Done"
    if status == 2 or status == 8:
        rospy.loginfo("Goal cancelled")
    if status == 4:
        rospy.loginfo("Goal aborted")

  def send_to_goal(self, navigate_from, navigate_to, done):
      # Connecting to the nav client server.
      navclient = actionlib.SimpleActionClient('/spot/navigate_to', NavigateToAction)
      navclient.wait_for_server()

      # Creating the goal object
      navigate_goal = NavigateToGoal()
      navigate_goal.upload_path = self.graph_upload_path
      navigate_goal.initial_localization_fiducial= False
      navigate_goal.initial_localization_waypoint = navigate_from
      navigate_goal.navigate_to = navigate_to

      # SENDING THE GOAL, calls done callback when completed.
      navclient.send_goal(navigate_goal, done, self.active_cb, self.feedback_cb)
      finished = navclient.wait_for_result()

      if not finished:
          rospy.logerr("Action server not available!")
          return
      
# def main():
if __name__ == '__main__':

  # Wait for ROS to start.
  time.sleep(1)

  rospy.init_node("Brain", log_level=rospy.INFO)
  rospy.loginfo("STARTING BRAIN")
  voice = Brain()
  rospy.spin()