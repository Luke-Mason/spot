#!/usr/bin/env python3

import time
from types import SimpleNamespace
import bosdyn.client
from bosdyn.client import Robot
import json
from audio import AudioLoadSoundCommand, AudioPlaySoundCommand
import rospy
import argparse
from bosdyn.client.spot_cam.audio import AudioClient
from bosdyn.api.spot_cam import audio_pb2
# class Spot():
#     spot = None
#     robot = None
#     sdk = None
#     state_client = None
#     id_client = None
#     lease_client = None
#     audio_client = None

#     def __init__(self) -> None:
        


#     def get_robot():
#         if Spot.spot is None:
#             spot =  Spot()

            

        
#         return spot

# def main():
if __name__ == '__main__':

    # Wait for ROS to start.
    time.sleep(1)

    rospy.init_node("Spot", log_level=rospy.INFO)
    #   rospy.loginfo("STARTING SPOT")
    #   spot = Spot.get_robot()

    mysdk = bosdyn.client.create_standard_sdk('nurse-spot')
    robot: Robot = mysdk.create_robot('192.168.80.3')
    robot.authenticate('admin', 'eco1nifqyn99')
    # robot.regi()
    state_client = robot.ensure_client('robot-state')
    id_client = robot.get_cached_robot_id()
    rospy.loginfo(id_client)

    audio_client: AudioClient = robot.ensure_client("spot-cam-audio")
    # audio_client

    # lease_client = robot.ensure_client('lease')

    # lease = lease_client.acquire()
    # lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)

    # audio_client = robot.ensure_client()
    # time.sleep(3)
    # rospy.loginfo(robot.list_services())

    # path_to_file = "../workspace/aiil_workspace/noetic_workspace/src/spot/nurse/media/hello.wav"

    # sound = audio_pb2.Sound(name="hello")
    # with open(path_to_file, 'rb') as fh:
    #     data = fh.read()
    # robot.ensure_client(AudioClient.default_service_name).load_sound(sound, data)

    # sound = audio_pb2.Sound(name="hello")
    # gain = 0
    # if gain:
    #     gain = max(gain, 0.0)
    # robot.ensure_client(AudioClient.default_service_name).play_sound(sound, gain)

    # rospy.loginfo(json.dump(lease_client.list_leases()))
    # rospy.loginfo(json.dump(state_client.get_robot_state()))
