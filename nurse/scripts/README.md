<!--
Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.

Downloading, reproducing, distributing or otherwise using the SDK Software
is subject to the terms and conditions of the Boston Dynamics Software
Development Kit License (20191101-BDSDK-SL).
-->

# Spot CAM Services

These examples demonstrate how to interact with the Spot CAM.

## Setup Dependencies
These examples need to be run with python3, and have the Spot SDK installed. See the requirements.txt file for a list of dependencies which can be installed with pip.
```
python3 -m pip install -r requirements.txt
```

Older versions of pip may have trouble installing all of the requirements.  If you run into a problem, upgrade pip by running
```
python3 -m pip install --upgrade pip
```

## Running the Example
To run the examples:
```
USERNAME=<username>
PASSWORD=<password>
ROBOT_IP=<ip-address>

# Version Service
python -m command_line 192.168.80.3 version software

# Audio Service
python -m command_line 192.168.80.3 audio set_volume 1
python -m command_line 192.168.80.3 audio get_volume
python -m command_line 192.168.80.3 audio load autonomous_robot_en data/autonomous_robot_en.wav
python -m command_line 192.168.80.3 audio list
python -m command_line 192.168.80.3 audio play autonomous_robot_en
python -m command_line 192.168.80.3 audio delete autonomous_robot_en
python -m command_line 192.168.80.3 audio list

# Compositor Service
python -m command_line 192.168.80.3 compositor list
python -m command_line 192.168.80.3 compositor get
python -m command_line 192.168.80.3 compositor set mech
python -m command_line 192.168.80.3 compositor visible
python -m command_line 192.168.80.3 compositor get_colormap
python -m command_line 192.168.80.3 compositor set_colormap jet

# Lighting Service
python -m command_line 192.168.80.3 lighting set 0.1 0.2 0.3 0.4
python -m command_line 192.168.80.3 lighting get

# Media Log Service
python -m command_line 192.168.80.3 media_log list_cameras
python -m command_line 192.168.80.3 media_log store pano
python -m command_line 192.168.80.3 media_log store_retrieve ptz
# The UUID is given by the 'store' command
IMAGE_UUID=f0e835c2-54d4-11ea-9365-00044be03a91
python -m command_line 192.168.80.3 media_log status $IMAGE_UUID
python -m command_line 192.168.80.3 media_log retrieve $IMAGE_UUID
python -m command_line 192.168.80.3 media_log delete $IMAGE_UUID

# You should not see the UUID in the list of logpoints
python -m command_line 192.168.80.3 media_log list_logpoints

# You should see 10 stitched jpeg images
seq 10 | xargs -I{} python -m command_line 192.168.80.3 media_log store_retrieve pano

# Ptz Service
python -m command_line 192.168.80.3 ptz list
python -m command_line 192.168.80.3 ptz set_position mech 0 0 1
python -m command_line 192.168.80.3 ptz get_position mech

# Network Service
# Get Spot CAM network settings
python -m command_line 192.168.80.3 network settings
# Set Spot CAM network settings (examples arguments below are the default values)
python -m command_line 192.168.80.3 network set 192.168.50.6 255.255.255.0 192.168.50.3 1500

# Get Spot CAM ICE settings
python -m command_line 192.168.80.3 network ice_settings
# Set Spot CAM ICE settings (example JSON file provided)
python -m command_line 192.168.80.3 network set_ice ice.json

# WebRTC Service
# Save images to .jpg files
python -m command_line 192.168.80.3 webrtc save
# Display stream
python -m command_line 192.168.80.3 webrtc save --count 0
# Save 10 seconds of video (no audio)
python -m command_line 192.168.80.3 webrtc record --time 10
# Save 10 seconds of audio
python -m command_line 192.168.80.3 webrtc record audio --time 10
```
