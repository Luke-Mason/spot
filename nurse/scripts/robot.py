#!/usr/bin/env python3

import time
from types import SimpleNamespace
import bosdyn.client
from bosdyn.client import Robot
import json
from audio import AudioGetAudioCaptureChannel, AudioLoadSoundCommand, AudioPlaySoundCommand
import rospy
import argparse
from bosdyn.client.spot_cam.audio import AudioClient
from bosdyn.api.spot_cam import audio_pb2
from bosdyn.client import spot_cam
import os
import grpc
import io
from aiortc.contrib.media import MediaRecorder
from webrtc import InterceptStdErr
import asyncio
import base64
import json
import logging
import sys
import threading
from webrtc import start_webrtc

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
    spot_cam.register_all_service_clients(mysdk)
    robot: Robot = mysdk.create_robot(os.environ["BOSDYN_CLIENT_IP"])
    robot.authenticate(os.environ["BOSDYN_CLIENT_USERNAME"], os.environ["BOSDYN_CLIENT_PASSWORD"])


    # robot.regi()
    # state_client = robot.ensure_client('robot-state')
    # id_client = robot.get_cached_robot_id()
    # rospy.loginfo(id_client)

    audio_client: AudioClient = robot.ensure_client("spot-cam-audio")

    # channel: grpc.Channel = audio_client.get_audio_capture_channel()
    # channel.stream_stream()
    # rospy.loginfo(channel)

    # def handle_response(response):
    #     pass

    # def handle_audio_error_from_response(response):
        return None

    # request = audio_pb2.GetAudioCaptureChannelRequest()
    # response: grpc.StreamStreamMultiCallable = AudioClient.call(AudioGetAudioCaptureChannel(), request,
    #                     handle_response,
    #                     handle_audio_error_from_response)

    ffmpeg_options = {
        "number": 100,
        "freq": 16000,
        "sample_fmt": "u8"
        # "codec": "pcm_s16le"
        }

    audio_byte_stream = io.BytesIO()
    recorder = MediaRecorder(audio_byte_stream, options=ffmpeg_options)

    webrtc_options = {"cam_ssl_cert": False}

    def process_audio_frame():
        pass

    # Suppress all exceptions and log them instead.
    sys.stderr = InterceptStdErr()
    shutdown_flag = threading.Event()
    webrtc_thread = threading.Thread(target=start_webrtc,
                                        args=[shutdown_flag, webrtc_options, process_audio_frame, recorder], daemon=True)
    webrtc_thread.start()

    try:
        webrtc_thread.join()
        print('Successfully saved webrtc images to local directory.')
    except KeyboardInterrupt:
        shutdown_flag.set()
        webrtc_thread.join(timeout=3.0)



    # audio_client

    # lease_client = robot.ensure_client('lease')

    # lease = lease_client.acquire()
    # lease_keep_alive = bosdyn.client.lease.LeaseKeepAlive(lease_client)

    # audio_client = robot.ensure_client()
    # time.sleep(3)
    # rospy.loginfo(robot.list_services())

    # path_to_file = "../workspace/aiil_workspace/noetic_workspace/src/spot/nurse/media/i_did_not_catch_that.wav"
    # sound = audio_pb2.Sound(name="hello")
    # with open(path_to_file, 'rb') as fh:
    #     data = fh.read()
    # audio_client.load_sound(sound, data)

    # sound = audio_pb2.Sound(name="hello")
    # audio_client.play_sound(sound, 3)

    # rospy.loginfo(json.dump(lease_client.list_leases()))
    # rospy.loginfo(json.dump(state_client.get_robot_state()))
