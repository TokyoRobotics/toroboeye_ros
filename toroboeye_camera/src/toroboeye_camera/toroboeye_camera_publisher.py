#!/usr/bin/env python
# -*- coding: utf-8 -*-

# BSD 3-Clause License
# Copyright (c) 2021, Tokyo Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, 
#   this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, 
#   this list of conditions and the following disclaimer in the documentation 
#   and/or other materials provided with the distribution.
# * Neither the name of the Tokyo Robotics Inc. nor the names of its contributors 
#   may be used to endorse or promote products derived from this software 
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# import os

import rospy
from sensor_msgs.msg import Image, CameraInfo

from toroboeye_camera import toroboeye_camera_client
from toroboeye_msgs.srv import *

class ToroboeyeCameraPublisher():

    def __init__(self):

        self._frame = None

        self._camera_info = CameraInfo()
        # prefix = rospy.get_param("toroboeye_camera_publisher_node/prefix")


        self._pub_color_image = rospy.Publisher('color/image', Image, queue_size = 1)
        self._pub_camera_info = rospy.Publisher('camera_info', CameraInfo, queue_size = 1)
        self._pub_depth_image = rospy.Publisher('depth/image', Image, queue_size = 1)

        publish_rate = rospy.get_param("toroboeye_camera_publisher_node/publish_rate")
        rospy.Timer(rospy.Duration(publish_rate), self.captured_frame_callback)
    
    def captured_frame_callback(self, event=None):
        #### update frame by compare timestamp ####
        self.update_captured_frame()

        #### publish camera message ####
        self.publish_camera_messages()

    def update_captured_frame(self):

        frame = toroboeye_camera_client.update_frame()

        if (self._frame == None):
            self._frame = frame

        elif (frame.timestamp > self._frame.timestamp):
            self._camera_info = toroboeye_camera_client.update_intrinsics().intrinsics
            self._frame = frame
                
    def publish_camera_messages(self):

        self._pub_color_image.publish(self._frame.color_image)
        self._pub_depth_image.publish(self._frame.depth_image)
        self._pub_camera_info.publish(self._camera_info)