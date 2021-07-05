#!/usr/bin/env python
## -*- coding: utf-8 -*-

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

###############################################################################
# this node is simple node to control capturing of toroboeye camera using ROS .
# you will access toroboeye by using toroboeye client module.
# Please check toroboeye cilent module.
###############################################################################

import rospy
from cv_bridge import CvBridge
import cv2
import os 
import datetime
from rospy.exceptions import ROSInterruptException

from toroboeye_camera import toroboeye_camera_client
from toroboeye_camera.toroboeye_camera_service import ToroboeyeCameraService
from pytoroboeye import toroboeye

ABSPATH = os.path.abspath(__file__)
NODE_NAME = os.path.splitext(os.path.basename(ABSPATH))[0]

class ToroboeyeCaptureExampleNode:

    def __init__(self):
        
        rospy.init_node(NODE_NAME, anonymous=True)
        rospy.loginfo('Node starts: ' + NODE_NAME)

        self._adjustment_depth_scale = rospy.get_param("adjustment_depth_scale")
        ip_address                   = rospy.get_param("device_ip_address")
        
        is_connection = False
        if(is_connection == False):
            toroboeye_camera_client.connect(ip_address)
            is_connection = True

        state         = toroboeye_camera_client.get_capture_setting()
        is_activation = state.state_activation

        if(is_activation == int(toroboeye.State.ACTIVATION.INACTIVE)):
            toroboeye_camera_client.activate()
            toroboeye_camera_client.wait_for_active()

    def capture(self):

        rospy.loginfo("Call Capture Service")
        toroboeye_camera_client.capture()

        ## Get Captured Frame
        toroboeye_camera_client.wait_for_frame()
        frame       = toroboeye_camera_client.update_frame()
        bridge      = CvBridge()
        color_image = bridge.imgmsg_to_cv2(frame.color_image, 'rgb8')
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        depth_image = bridge.imgmsg_to_cv2(frame.depth_image, '32FC1')
        depth_image *= self._adjustment_depth_scale

        timestamp   = frame.timestamp

        self.save_image(color_image, depth_image, timestamp)

        rospy.signal_shutdown("Finish Capturing")
    
    def save_image(self, color_image, depth_image, timestamp):

        ## Get Current Timestamp
        dt_object = datetime.datetime.fromtimestamp(timestamp)
        dt_text   = dt_object.strftime('%Y-%m-%d_%H-%M-%S')
        rospy.loginfo('timestamp = '+dt_text)

        ## Saving Each Images
        rospy.loginfo("Save Color and Depth Images")
        script_directory_name   = os.path.dirname(__file__)
        specific_directory_name = rospy.get_param("save_directory")
        save_directory_name     = os.path.join(script_directory_name, specific_directory_name)

        if(not os.path.isdir(save_directory_name)):
            os.makedirs(save_directory_name)

        color_image_file_name = save_directory_name + "color_image_" + str(timestamp) + '_' + str(dt_text) + '.png'
        depth_image_file_name = save_directory_name + "depth_image_" + str(timestamp) + '_' + str(dt_text) + '.png'
        cv2.imwrite(color_image_file_name, color_image)
        cv2.imwrite(depth_image_file_name, depth_image)

if __name__ == '__main__':

    try:
        tc = ToroboeyeCameraService()
        te = ToroboeyeCaptureExampleNode()
        te.capture()
        rospy.spin()
    except ROSInterruptException:
        pass