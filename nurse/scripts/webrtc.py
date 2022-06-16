# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

import asyncio
import base64
import json
import logging
import sys
import threading
import time

from aiortc import (
    RTCConfiguration,
    RTCPeerConnection,
    RTCSessionDescription,
    MediaStreamTrack,
)
from aiortc.contrib.media import MediaRecorder
import requests

import cv2
import numpy as np

from bosdyn.client.command_line import (Command, Subcommands)
from webrtc_client import WebRTCClient

import colorsys
import os
import subprocess

#import tensorflow as tf

#from keras import backend as K
#from keras.models import load_model
#from tensorflow.python.framework.ops import disable_eager_execution

import face_recognition

#import urllib3
#urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


#disable_eager_execution()
#os.environ['CUDA_VISIBLE_DEVICES'] = '0'

logging.basicConfig(level=logging.DEBUG, filename='webrtc.log', filemode='a+')
STDERR = logging.getLogger('stderr')



class InterceptStdErr:
    """Intercept all exceptions and print them to StdErr without interrupting."""
    _stderr = sys.stderr

    def __init__(self):
        pass

    def write(self, data):
        STDERR.error(data)

class WebRTCCommands(Subcommands):
    """Commands related to the Spot CAM's WebRTC service"""

    NAME = 'webrtc'

    def __init__(self, subparsers, command_dict):
        super(WebRTCCommands, self).__init__(subparsers, command_dict,
                                             [WebRTCSaveCommand, WebRTCRecordCommand])


class WebRTCSaveCommand(Command):
    """Save webrtc stream as a sequence of images"""

    NAME = 'save'
    print("In save func")

    def __init__(self, subparsers, command_dict):
        super(WebRTCSaveCommand, self).__init__(subparsers, command_dict)
        self._parser.add_argument('track', default='video', const='video', nargs='?',
                                  choices=['video'])
        self._parser.add_argument('--sdp-filename', default='h264.sdp',
                                  help='File being streamed from WebRTC server')
        self._parser.add_argument('--sdp-port', default=31102, help='SDP port of WebRTC server')
        self._parser.add_argument('--cam-ssl-cert', default=None,
                                  help="Spot CAM's client cert path to check with Spot CAM server")
        self._parser.add_argument('--dst-prefix', default='h264.sdp',
                                  help='Filename prefix to prepend to all output data')
        self._parser.add_argument('--count', type=int, default=0,
                                  help='Number of images to save. 0 to stream without saving.')

    def _run(self, robot, options):
        # Suppress all exceptions and log them instead.
        sys.stderr = InterceptStdErr()

        if not options.cam_ssl_cert:
            options.cam_ssl_cert = False

        shutdown_flag = threading.Event()
        webrtc_thread = threading.Thread(target=start_webrtc,
                                         args=[shutdown_flag, options, process_frame], daemon=True)
        webrtc_thread.start()

        try:
            webrtc_thread.join()
            print('Successfully saved webrtc images to local directory.')
        except KeyboardInterrupt:
            shutdown_flag.set()
            webrtc_thread.join(timeout=3.0)


class WebRTCRecordCommand(Command):
    """Save webrtc stream as video or audio"""

    NAME = 'record'
    print("In record class")

    def __init__(self, subparsers, command_dict):
        super(WebRTCRecordCommand, self).__init__(subparsers, command_dict)
        self._parser.add_argument('track', default='video', const='video', nargs='?',
                                  choices=['video', 'audio'])
        self._parser.add_argument('--sdp-filename', default='h264.sdp',
                                  help='File being streamed from WebRTC server')
        self._parser.add_argument('--sdp-port', default=31102, help='SDP port of WebRTC server')
        self._parser.add_argument('--cam-ssl-cert', default=None,
                                  help="Spot CAM's client cert path to check with Spot CAM server")
        self._parser.add_argument('--dst-prefix', default='h264.sdp',
                                  help='Filename prefix to prepend to all output data')
        self._parser.add_argument('--time', type=int, default=10,
                                  help='Number of seconds to record.')

    def _run(self, robot, options):
        # Suppress all exceptions and log them instead.
        sys.stderr = InterceptStdErr()

        if not options.cam_ssl_cert:
            options.cam_ssl_cert = False

        if options.track == 'video':
            recorder = MediaRecorder(f'{options.dst_prefix}.mp4')
        else:
            recorder = MediaRecorder(f'{options.dst_prefix}.wav')

        # run event loop
        loop = asyncio.get_event_loop()
        loop.run_until_complete(record_webrtc(options, recorder))


# WebRTC must be in its own thread with its own event loop.
async def record_webrtc(options, recorder):

    print("In record func")
    config = RTCConfiguration(iceServers=[])
    client = WebRTCClient(options.hostname, options.username, options.password, options.sdp_port,
                          options.sdp_filename, options.cam_ssl_cert, config,
                          media_recorder=recorder, recorder_type=options.track)
    await client.start()

    # wait for connection to be established before recording
    while client.pc.iceConnectionState != 'completed':
        await asyncio.sleep(0.1)

    # start recording
    await recorder.start()
    try:
        await asyncio.sleep(options.time)
    except KeyboardInterrupt:
        pass
    finally:
        # close everything
        await client.pc.close()
        await recorder.stop()


# WebRTC must be in its own thread with its own event loop.
def start_webrtc(shutdown_flag, options, process_func, recorder=None):
    print("In start func")
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    config = RTCConfiguration(iceServers=[])
    client = WebRTCClient(options.hostname, options.username, options.password, options.sdp_port,
                          options.sdp_filename, options.cam_ssl_cert, config,
                          media_recorder=recorder)

    asyncio.gather(client.start(), process_func(client, options, shutdown_flag),
                   monitor_shutdown(shutdown_flag, client))
    loop.run_forever()

def facial_recognition(face_img, stream_img):

    face_locations = face_recognition.face_locations(face_img)

    # Get the single face encoding out of elon-musk-1.jpg
    face_location = face_locations[0]  # Only use the first detected face
    face_encodings = face_recognition.face_encodings(face_img, [face_location])
    person_face_encoding = face_encodings[0]  # Pull out the one returned face encoding


    # Load the image with unknown to compare
    #image = face_recognition.load_image_file("cover2.jpg")  # Load the image we are comparing
    unknwon_face_encodings = face_recognition.face_encodings(stream_img)

    # Loop over each unknwon face encoding to see if the face matches either known encodings
    #print('Matches for elon-musk-in-group.jpg')
    matches = []
    for unknwon_face_encoding in unknwon_face_encodings:
        matches.append(face_recognition.compare_faces(
            [person_face_encoding],  # The known face encodings (can be only 1 - less is faster)
            unknwon_face_encoding  # The single unknown face encoding
        ))
        #print(matches)

    print(matches)
    face_locations = face_recognition.face_locations(stream_img)
    #print(face_locations)

    for i in range(len(matches)):
        if matches[i][0] == True:

            subprocess.run(["seq", "2", "|", "xargs", "-I{}", "python3", "/home/rmitaiil/workspace/aiil_workspace/noetic_workspace/src/spot/nurse/scripts/command_line.py", "192.168.80.3", "media_log",  "store_retrieve" ,"pano"])
            # seq 10 | xargs -I{} python -m command_line 192.168.80.3 media_log store_retrieve pano
            print("Image found at location: " + str(face_locations[i]))
            print()

    # if found, take photo


# Frame processing occurs; otherwise it waits.
async def process_frame(client, options, shutdown_flag):
    count = 0
    print("In process func")
    print("attempting to create yolo object")
    #yolo = YOLO()

    print("yolo object created")
    # set start time to current time
    start_time = time.time()
    print(start_time)
    # displays the frame rate every 2 second
    display_time = 2
    # Set primarry FPS to 0
    fps = 0

    # we create the video capture object cap
    
    while asyncio.get_event_loop().is_running():
        try:
            frame = await client.video_frame_queue.get()

            if options.count == 0:

                pil_image = frame.to_image()
                cv_image = np.array(pil_image)
                # OpenCV needs BGR
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                
                frame = cv2.resize(cv_image, None, fx=1.0, fy=1.0, interpolation=cv2.INTER_AREA)
                # detect object on our frame
                #r_image, ObjectsList = yolo.detect_img(frame)
                
                # show us frame with detection
                #cv2.imshow("Web cam input", r_image)


                if (time.time() - start_time) > 3:

                    print("Time to recognize face")


                    face_img = face_recognition.load_image_file("/home/rmitaiil/workspace/aiil_workspace/noetic_workspace/src/spot/nurse/scripts/images/dev.jpg")

                    facial_recognition(face_img, cv_image)

                    start_time = time.time()

                cv2.imshow('display', cv_image)
                if cv2.waitKey(25) & 0xFF == ord("q"):
                    cv2.destroyAllWindows()
                    break

                # calculate FPS
                '''
                fps += 1
                TIME = time.time() - start_time
                if TIME > display_time:
                    print("FPS:", fps / TIME)
                    fps = 0
                    start_time = time.time()
                '''

                cv2.waitKey(1)
            else:
                frame.to_image().save(f'{options.dst_prefix}-{count}.jpg')
                count += 1
                if count >= options.count:
                    break
        except Exception as e:
            print(e)
        try:
            # discard audio frames
            while not client.audio_frame_queue.empty():
                await client.audio_frame_queue.get()
        except Exception as e:
            print(e)

    shutdown_flag.set()


# Flag must be monitored in a different coroutine and sleep to allow frame
# processing to occur.
async def monitor_shutdown(shutdown_flag, client):
    while not shutdown_flag.is_set():
        await asyncio.sleep(1.0)

    await client.pc.close()
    asyncio.get_event_loop().stop()
