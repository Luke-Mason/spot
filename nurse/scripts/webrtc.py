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

class YOLO(object):
    # TODO fix paths
    def __init__(self, **kwargs):

        print("init function")
        self.model_path = "yolo/yolov3.h5"
        self.anchors_path = "yolo/yolo_anchors.txt"
        self.classes_path = "yolo/all_classes.txt"
        self.score = 0.3
        self.iou = 0.45
        self.model_image_size = (320, 320) #(480, 640)
        self.text_size = 1
        self.class_names = self._get_class()
        self.anchors = self._get_anchors()
        self.sess = K.get_session()
        print("SESSION")
        print(self.sess)
        self.boxes, self.scores, self.classes = self.generate()


    def _get_class(self):
        classes_path = os.path.expanduser(self.classes_path)
        print(classes_path)
        with open(classes_path) as f:
            class_names = f.readlines()

        class_names = [c.strip() for c in class_names]
        print("CLASS NAMES")
        print(class_names)
        return class_names

    def _get_anchors(self):
        anchors_path = os.path.expanduser(self.anchors_path)
        with open(anchors_path) as f:
            anchors = f.readline()
        anchors = [float(x) for x in anchors.split(',')]
        print("ANCHORS")
        print(np.array(anchors).reshape(-1, 2))
        return np.array(anchors).reshape(-1, 2)

    def generate(self):
        model_path = os.path.expanduser(self.model_path)
        assert model_path.endswith('.h5'), 'Keras model or weights must be a .h5 file.'

        # Load model, or construct model and load weights.
        # num_anchors = len(self.anchors)
        # num_classes = len(self.class_names)
        # is_tiny_version = num_anchors == 6  # default setting
        # try:
        print("Trying to load weights")
        self.yolo_model = load_model(model_path, compile=False)
        print("Successfully loaded weights")
        # except:
        #     self.yolo_model = tiny_yolo_body(Input(shape=(None, None, 3)), num_anchors // 2, num_classes) \
        #         if is_tiny_version else yolo_body(Input(shape=(None, None, 3)), num_anchors // 3, num_classes)
        #     self.yolo_model.load_weights(self.model_path)  # make sure model, anchors and classes match
        # else:
        #     assert self.yolo_model.layers[-1].output_shape[-1] == \
        #            num_anchors / len(self.yolo_model.output) * (num_classes + 5), \
        #         'Mismatch between model and given anchor and class sizes'

        print('{} model, anchors, and classes loaded.'.format(model_path))

        # Generate colors for drawing bounding boxes.
        hsv_tuples = [(x / len(self.class_names), 1., 1.)
                      for x in range(len(self.class_names))]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)),
                self.colors))

        np.random.shuffle(self.colors)  # Shuffle colors to decorrelate adjacent classes.

        # Generate output tensor targets for filtered bounding boxes.
        self.input_image_shape = K.placeholder(shape=(2,))
        boxes, scores, classes = self.yolo_eval(self.yolo_model.output, self.anchors,
                                                len(self.class_names), self.input_image_shape,
                                                score_threshold=self.score, iou_threshold=self.iou)
        return boxes, scores, classes

    def detect_image(self, image):
        if self.model_image_size != (None, None):
            assert self.model_image_size[0] % 32 == 0, 'Multiples of 32 required'
            assert self.model_image_size[1] % 32 == 0, 'Multiples of 32 required'
            boxed_image = self.image_preprocess(np.copy(image), tuple(reversed(self.model_image_size)))
            image_data = boxed_image

        out_boxes, out_scores, out_classes = self.sess.run(
            [self.boxes, self.scores, self.classes],
            feed_dict={
                self.yolo_model.input: image_data,
                self.input_image_shape: [image.shape[0], image.shape[1]],  # [image.size[1], image.size[0]],
                K.learning_phase(): 0
            })

        # print('Found {} boxes for {}'.format(len(out_boxes), 'img'))

        thickness = (image.shape[0] + image.shape[1]) // 600
        fontScale = 1
        ObjectsList = []

        for i, c in reversed(list(enumerate(out_classes))):
            predicted_class = self.class_names[c]
            box = out_boxes[i]
            score = out_scores[i]

            label = '{} {:.2f}'.format(predicted_class, score)
            # label = '{}'.format(predicted_class)
            scores = '{:.2f}'.format(score)

            top, left, bottom, right = box
            top = max(0, np.floor(top + 0.5).astype('int32'))
            left = max(0, np.floor(left + 0.5).astype('int32'))
            bottom = min(image.shape[0], np.floor(bottom + 0.5).astype('int32'))
            right = min(image.shape[1], np.floor(right + 0.5).astype('int32'))

            mid_h = (bottom - top) / 2 + top
            mid_v = (right - left) / 2 + left

            # put object rectangle
            cv2.rectangle(image, (left, top), (right, bottom), self.colors[c], thickness)

            # get text size
            (test_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                                                  thickness / self.text_size, 1)

            # put text rectangle
            cv2.rectangle(image, (left, top), (left + test_width, top - text_height - baseline), self.colors[c],
                          thickness=cv2.FILLED)

            # put text above rectangle
            cv2.putText(image, label, (left, top - 2), cv2.FONT_HERSHEY_SIMPLEX, thickness / self.text_size, (0, 0, 0),
                        1)

            # add everything to list
            ObjectsList.append([top, left, bottom, right, mid_v, mid_h, label, scores])

        return image, ObjectsList

    def close_session(self):
        self.sess.close()

    def detect_img(self, image):
        # image = cv2.imread(image, cv2.IMREAD_COLOR)
        original_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        original_image_color = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)

        r_image, ObjectsList = self.detect_image(original_image_color)
        return r_image, ObjectsList

    def image_preprocess(self, image, target_size, gt_boxes=None):

        ih, iw = target_size
        h, w, _ = image.shape

        scale = min(iw / w, ih / h)
        nw = int(scale * w)
        nh = int(scale * h)

        image_resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_CUBIC)

        image_paded = np.full(shape=[ih, iw, 3], fill_value=128.0)
        dw, dh = (iw - nw) // 2, (ih - nh) // 2
        image_paded[dh:nh + dh, dw:nw + dw, :] = image_resized

        image_paded = np.array(image_paded, dtype='float32')

        image_paded = image_paded / 255.

        image_paded = np.expand_dims(image_paded, axis=0)

        return image_paded

    def yolo_eval(self, yolo_outputs,
                  anchors,
                  num_classes,
                  image_shape,
                  max_boxes=20,
                  score_threshold=.6,
                  iou_threshold=.5):
        """Evaluate YOLO model on given input and return filtered boxes."""
        num_layers = len(yolo_outputs)
        anchor_mask = [[6, 7, 8], [3, 4, 5], [0, 1, 2]] if num_layers == 3 else [[3, 4, 5],
                                                                                 [1, 2, 3]]  # default setting
        input_shape = K.shape(yolo_outputs[0])[1:3] * 32
        boxes = []
        box_scores = []
        for l in range(num_layers):
            _boxes, _box_scores = self.yolo_boxes_and_scores(yolo_outputs[l],
                                                             anchors[anchor_mask[l]], num_classes, input_shape,
                                                             image_shape)
            boxes.append(_boxes)
            box_scores.append(_box_scores)
        boxes = K.concatenate(boxes, axis=0)
        box_scores = K.concatenate(box_scores, axis=0)

        mask = box_scores >= score_threshold
        max_boxes_tensor = K.constant(max_boxes, dtype='int32')
        boxes_ = []
        scores_ = []
        classes_ = []
        for c in range(num_classes):
            # TODO: use keras backend instead of tf.
            class_boxes = tf.boolean_mask(boxes, mask[:, c])
            class_box_scores = tf.boolean_mask(box_scores[:, c], mask[:, c])
            nms_index = tf.image.non_max_suppression(
                class_boxes, class_box_scores, max_boxes_tensor, iou_threshold=iou_threshold)
            class_boxes = K.gather(class_boxes, nms_index)
            class_box_scores = K.gather(class_box_scores, nms_index)
            classes = K.ones_like(class_box_scores, 'int32') * c
            boxes_.append(class_boxes)
            scores_.append(class_box_scores)
            classes_.append(classes)
        boxes_ = K.concatenate(boxes_, axis=0)
        scores_ = K.concatenate(scores_, axis=0)
        classes_ = K.concatenate(classes_, axis=0)

        return boxes_, scores_, classes_

    def yolo_boxes_and_scores(self, feats, anchors, num_classes, input_shape, image_shape):
        '''Process Conv layer output'''
        box_xy, box_wh, box_confidence, box_class_probs = self.yolo_head(feats,
                                                                         anchors, num_classes, input_shape)
        boxes = self.yolo_correct_boxes(box_xy, box_wh, input_shape, image_shape)
        boxes = K.reshape(boxes, [-1, 4])
        box_scores = box_confidence * box_class_probs
        box_scores = K.reshape(box_scores, [-1, num_classes])
        return boxes, box_scores

    def yolo_head(self, feats, anchors, num_classes, input_shape, calc_loss=False):
        """Convert final layer features to bounding box parameters."""
        num_anchors = len(anchors)
        # Reshape to batch, height, width, num_anchors, box_params.
        anchors_tensor = K.reshape(K.constant(anchors), [1, 1, 1, num_anchors, 2])

        grid_shape = K.shape(feats)[1:3]  # height, width
        grid_y = K.tile(K.reshape(K.arange(0, stop=grid_shape[0]), [-1, 1, 1, 1]),
                        [1, grid_shape[1], 1, 1])
        grid_x = K.tile(K.reshape(K.arange(0, stop=grid_shape[1]), [1, -1, 1, 1]),
                        [grid_shape[0], 1, 1, 1])
        grid = K.concatenate([grid_x, grid_y])
        grid = K.cast(grid, K.dtype(feats))

        feats = K.reshape(
            feats, [-1, grid_shape[0], grid_shape[1], num_anchors, num_classes + 5])

        # Adjust preconditions to each spatial grid point and anchor size.
        box_xy = (K.sigmoid(feats[..., :2]) + grid) / K.cast(grid_shape[::-1], K.dtype(feats))
        box_wh = K.exp(feats[..., 2:4]) * anchors_tensor / K.cast(input_shape[::-1], K.dtype(feats))
        box_confidence = K.sigmoid(feats[..., 4:5])
        box_class_probs = K.sigmoid(feats[..., 5:])

        if calc_loss == True:
            return grid, feats, box_xy, box_wh
        return box_xy, box_wh, box_confidence, box_class_probs

    def yolo_correct_boxes(self, box_xy, box_wh, input_shape, image_shape):
        '''Get corrected boxes'''
        box_yx = box_xy[..., ::-1]
        box_hw = box_wh[..., ::-1]
        input_shape = K.cast(input_shape, K.dtype(box_yx))
        image_shape = K.cast(image_shape, K.dtype(box_yx))
        new_shape = K.round(image_shape * K.min(input_shape / image_shape))
        offset = (input_shape - new_shape) / 2. / input_shape
        scale = input_shape / new_shape
        box_yx = (box_yx - offset) * scale
        box_hw *= scale

        box_mins = box_yx - (box_hw / 2.)
        box_maxes = box_yx + (box_hw / 2.)
        boxes = K.concatenate([
            box_mins[..., 0:1],  # y_min
            box_mins[..., 1:2],  # x_min
            box_maxes[..., 0:1],  # y_max
            box_maxes[..., 1:2]  # x_max
        ])

        # Scale boxes back to original image shape.
        boxes *= K.concatenate([image_shape, image_shape])
        return boxes


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
        self._parser.add_argument('--count', type=int, default=1,
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


# Frame processing occurs; otherwise it waits.
async def process_frame(client, options, shutdown_flag):
    count = 0
    print("In process func")
    print("attempting to create yolo object")
    #yolo = YOLO()

    print("yolo object created")
    # set start time to current time
    start_time = time.time()
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
                known_image = face_recognition.load_image_file("images/dev.jpg")

                face_locations = face_recognition.face_locations(known_image)

                # Get the single face encoding out of elon-musk-1.jpg
                face_location = face_locations[0]  # Only use the first detected face
                face_encodings = face_recognition.face_encodings(known_image, [face_location])
                elon_musk_knwon_face_encoding_1 = face_encodings[0]  # Pull out the one returned face encoding


                # Load the image with unknown to compare
                #image = face_recognition.load_image_file("cover2.jpg")  # Load the image we are comparing
                unknwon_face_encodings = face_recognition.face_encodings(cv_image)

                # Loop over each unknwon face encoding to see if the face matches either known encodings
                #print('Matches for elon-musk-in-group.jpg')
                matches = []
                for unknwon_face_encoding in unknwon_face_encodings:
                    matches.append(face_recognition.compare_faces(
                        [elon_musk_knwon_face_encoding_1],  # The known face encodings (can be only 1 - less is faster)
                        unknwon_face_encoding  # The single unknown face encoding
                    ))
                    #print(matches)

                print(matches)
                face_locations = face_recognition.face_locations(cv_image)
                print(face_locations)


                for i in range(len(matches)):
                    if matches[i][0] == True:
                        print("Image found at location: " + str(face_locations[i]))
                        print()

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
