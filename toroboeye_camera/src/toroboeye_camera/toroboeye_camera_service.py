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

import copy
import cv2
import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Header

from pytoroboeye import toroboeye
from toroboeye_msgs.msg import Device
from toroboeye_msgs.srv import *


class ToroboeyeCameraService():
    def __init__(self):
        self._tc                          = toroboeye.Camera()
        self._service_connect             = rospy.Service('connect', Connect, self._service_connect_callback)
        self._service_disconnect          = rospy.Service('disconnect', Disconnect, self._service_disconnect_callback)
        self._service_get_capture_setting = rospy.Service('get_capture_setting',GetCaptureSetting, self._service_get_capture_setting_callback)
        self._service_get_status          = rospy.Service('get_status',GetStatus, self._service_get_status_callback)
        self._service_set_capture_setting = rospy.Service('set_capture_setting',SetCaptureSetting, self._service_set_capture_setting_callback)
        self._service_write               = rospy.Service('write',Write, self._service_write_callback)
        self._service_activate            = rospy.Service('activate', Activate, self._service_activate_callback)
        self._service_deactivate          = rospy.Service('deactivate', Deactivate, self._service_deactivate_callback)
        self._service_capture             = rospy.Service('capture', Capture, self._service_capture_callback)
        self._service_wait_for_state      = rospy.Service('wait_for_state',WaitForState, self._service_wait_for_state_callback)
        self._service_wait_for_active     = rospy.Service('wait_for_active',WaitForActive, self._service_wait_for_active_callback)
        self._service_wait_for_inactive   = rospy.Service('wait_for_inactive',WaitForInactive, self._service_wait_for_inactive_callback)
        self._service_wait_for_frame      = rospy.Service('wait_for_frame', WaitForFrame, self._service_wait_for_frame_callback)
        self._service_stop                = rospy.Service('stop', Stop, self._service_stop_callback)
        self._service_get_intrinsics      = rospy.Service('get_intrinsics', GetIntrinsics, self._service_get_intrinsics_callback)

        #### update message ####
        self._service_update_frame        = rospy.Service('update_frame', WaitForFrame, self._service_update_frame_callback)
        self._service_update_intrinsics   = rospy.Service('update_intrinsics', GetIntrinsics, self._service_update_intrinsics_callback)
        self._frame_timestamp             = 0.0
        self._frame                       = None
        self._color_image                 = None
        self._depth_image                 = None
        self._bridge                      = CvBridge()
        self._intrinsics                  = None
        self._is_connection               = False
        self._is_activation               = False 
        self._is_captured                 = False
        self._header                      = Header()
        self._prefix                      = rospy.get_param("prefix")
        self._header.frame_id             = self._prefix + '/camera_optical_frame'

        #### Depth Filter value ####
        self._adjustment_depth_scale      = rospy.get_param("adjustment_depth_scale")
        self._upper_base_limit       = rospy.get_param("upper_base_limit")
        self._lower_amplitude_limit       = rospy.get_param("lower_amplitude_limit")
        self._gradient_threshold          = rospy.get_param("gradient_threshold")
        self._edge_dilation_value         = rospy.get_param("edge_dilation_value")
        self._min_area_x                  = rospy.get_param("min_area_x")
        self._min_area_y                  = rospy.get_param("min_area_y")
        self._max_area_x                  = rospy.get_param("max_area_x")
        self._max_area_y                  = rospy.get_param("max_area_y")
        self._depth_range_min             = rospy.get_param("depth_range_min")
        self._depth_range_max             = rospy.get_param("depth_range_max")

        #### Color Filter Value ####
        self._gain_red                    = rospy.get_param("colorbalance_gain_red")
        self._gain_green                  = rospy.get_param("colorbalance_gain_green")
        self._gain_blue                   = rospy.get_param("colorbalance_gain_blue")
        self._input_range_min             = rospy.get_param("input_level_dark")
        self._input_range_max             = rospy.get_param("input_level_light")
        self._output_range_min            = rospy.get_param("output_level_dark")
        self._output_range_max            = rospy.get_param("output_level_light")
        self._brightness_gamma            = rospy.get_param("brightness_gamma")
        self._sharpness_level             = rospy.get_param("sharpness_level")

    def _service_connect_callback(self, req):
        res = ConnectResponse()
        res.success = True
        res.message = ''
        try:
            self._tc.connect(ip = req.ip,sync_time=req.sync_time)
            self._is_connection = True
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res

    def _service_disconnect_callback(self, req):
        res = DisconnectResponse()
        res.success = True
        res.message = ''
        try:
            self._tc.disconnect()
            self._is_connection = False
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res

    def _service_get_capture_setting_callback(self, req):
        res = GetCaptureSettingResponse()
        res.success = True
        res.message = ''
        try:
            if(self._is_connection):
                res.device_illuminant_power                           = self._tc.get(toroboeye.Setting.DEVICE.ILLUMINANT_POWER.ID)
                res.depth_illuminant_color                            = self._tc.get(toroboeye.Setting.DEPTH.ILLUMINANT_COLOR.ID)
                res.depth_coding_pattern                              = self._tc.get(toroboeye.Setting.DEPTH.CODING_PATTERN.ID)
                res.depth_accuracy                                    = self._tc.get(toroboeye.Setting.DEPTH.ACCURACY.ID)
                res.depth_exposure_time                               = self._tc.get(toroboeye.Setting.DEPTH.EXPOSURE_TIME.ID)
                res.color_strobe_intensity                            = self._tc.get(toroboeye.Setting.COLOR.STROBE_INTENSITY.ID)
                res.color_exposure_time                               = self._tc.get(toroboeye.Setting.COLOR.EXPOSURE_TIME.ID)
                res.device_configuration                              = Device() 
                dc                                                    = self._tc.get(toroboeye.Info.DEVICE.CONFIGURATION.ID)
                res.device_configuration.camera_product_name          = dc["camera_product_name"]
                res.device_configuration.camera_serial_number         = dc["camera_serial_number"]
                res.device_configuration.controller_serial_number     = dc["controller_serial_number"]
                res.device_configuration.controller_firmware_version  = dc["controller_firmware_version"]
                res.device_configuration.api_version                  = dc["api_version"]
                res.state_activation                                  = self._tc.get(toroboeye.State.ACTIVATION.ID)
                res.state_processing                                  = self._tc.get(toroboeye.State.PROCESSING.ID)

        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res

    def _service_get_status_callback(self, req):
        res = GetStatusResponse()
        res.success = True
        res.message = ''
        try:
            res.state = self._tc.get(id = req.id)

        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res


    def _service_set_capture_setting_callback(self, req):
        res = SetCaptureSettingResponse()
        res.success = True
        res.message = ''
        try:
            self._tc.set(toroboeye.Setting.DEVICE.ILLUMINANT_POWER.ID    , req.device_illuminant_power      , req.data)
            self._tc.set(toroboeye.Setting.DEPTH.ILLUMINANT_COLOR.ID     , req.depth_illuminant_color       , req.data)
            self._tc.set(toroboeye.Setting.DEPTH.CODING_PATTERN.ID       , req.depth_coding_pattern         , req.data)
            self._tc.set(toroboeye.Setting.DEPTH.ACCURACY.ID             , req.depth_accuracy               , req.data)
            self._tc.set(toroboeye.Setting.DEPTH.EXPOSURE_TIME.ID        , req.depth_exposure_time          , req.data)
            self._tc.set(toroboeye.Setting.COLOR.STROBE_INTENSITY.ID     , req.color_strobe_intensity       , req.data)
            self._tc.set(toroboeye.Setting.COLOR.EXPOSURE_TIME.ID        , req.color_exposure_time          , req.data)
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res

    def _service_write_callback(self, req):
        res = WriteResponse()
        res.success = True
        res.message = ''
        try:
            self._tc.write()
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res

    def _service_activate_callback(self, req):
        res = ActivateResponse()
        res.success = True
        res.message = ''
        try:
            self._tc.activate()
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res

    def _service_deactivate_callback(self, req):
        res = DeactivateResponse()
        res.success = True
        res.message = ''
        try:
            self._tc.deactivate()
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res

    def _service_capture_callback(self, req):
        res = CaptureResponse()
        res.success = True
        res.message = ''
        try:
            self._tc.capture(oneshot = req.oneshot)
            self._is_captured = True
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res

    def _service_wait_for_state_callback(self, req):
        res = WaitForStateResponse()
        res.success = True
        res.message = ''
        try:
            self.wait_for_state(req.activation, req.processing, req.timeout, callback = None)
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res
    
    def _service_wait_for_active_callback(self, req):
        res = WaitForActiveResponse()
        res.success = True
        res.message = ''
        try:
            # self._tc.wait_for_active(callback=lambda progress: rospy.loginfo('\033[A\33[2K\r    progress = {}%'.format(progress)))
            self._tc.wait_for_active(callback=lambda progress: rospy.loginfo('Activating. . .'.format(progress)))
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res

    def _service_wait_for_inactive_callback(self, req):
        res = WaitForInactiveResponse()
        res.success = True
        res.message = ''
        try:
            self._tc.wait_for_inactive(callback=None)
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res

    def _service_wait_for_frame_callback(self, req):
        
        res = WaitForFrameResponse()
        res.success = True
        res.message = ''
        
        try:               
            self._is_activation = (self._tc.get(toroboeye.State.ACTIVATION.ID) == toroboeye.State.ACTIVATION.ACTIVE)
            if(self._is_connection and self._is_activation):
                frame = self._tc.wait_for_frame(req.timeout)
                if(frame.timestamp > self._frame_timestamp):

                    self._frame           = frame
                    self._frame_timestamp = frame.timestamp
                    self._is_captured     = True

                    res.color_image = self._bridge.cv2_to_imgmsg(frame.color_image, "rgb8")
                    depth_image     = frame.depth_image/self._adjustment_depth_scale
                    res.depth_image = self._bridge.cv2_to_imgmsg(depth_image, "32FC1")
                    res.score_image = self._bridge.cv2_to_imgmsg(frame.score_image, "rgb8")
                    res.timestamp   = frame.timestamp
                
                if frame.timestamp is None:
                    res.success = False
                    res.message = "Fail to capture"
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
   
        return res
    
    def _process_frame(self, frame):

        color_image = copy.deepcopy(frame.color_image)
        depth_image = copy.deepcopy(frame.depth_image)
        score_image = copy.deepcopy(frame.score_image)

        color_image = toroboeye.utility.color.image_enhance(color_image,
                                                            colorbalance_gain=[self._gain_red, self._gain_green, self._gain_blue],
                                                            contrast_range=[self._input_range_min, self._input_range_max, self._output_range_min, self._output_range_max],
                                                            brightness_gamma=self._brightness_gamma,
                                                            sharpness_level=self._sharpness_level)

        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        
        #### cut reflection ####
        depth_image = toroboeye.utility.depth.cutoff_by_reflection_properties(depth_image, score_image, amplitude_limit=self._upper_base_limit, base_limit=self._lower_amplitude_limit)

        #### cut noise by depth gradient ####
        depth_image = toroboeye.utility.depth.cutoff_by_depth_gradient(depth_image, gradient_threshold=self._gradient_threshold, edge_dilation=self._edge_dilation_value)

        #### crop captured area #### 
        depth_image = toroboeye.utility.depth.crop_area(depth_image, self._min_area_x, self._min_area_y, self._max_area_x, self._max_area_y)

        #### cut off depth calue by specific range ####
        depth_image = toroboeye.utility.depth.cutoff_out_of_depth_range(depth_image, self._depth_range_min, self._depth_range_max)  

        depth_image /= self._adjustment_depth_scale
        depth_image = self._bridge.cv2_to_imgmsg(depth_image, "32FC1")        
        color_image = self._bridge.cv2_to_imgmsg(color_image, "rgb8")
        score_image = self._bridge.cv2_to_imgmsg(score_image, "rgb8")

        depth_image.header = self._header
        color_image.header = self._header

        return color_image, depth_image, score_image
     
    def _service_update_frame_callback(self, req):

        res = WaitForFrameResponse()
        res.success = True
        res.message = ''

        try:
            if(self._frame != None):
                if(self._is_captured):
                    frame = self._frame
                    color_image, depth_image, score_image = self._process_frame(frame)
                    res.color_image = color_image 
                    res.depth_image = depth_image
                    res.score_image = score_image
                    res.timestamp   = frame.timestamp
 
                    self._frame = None
            else:
                res.color_image                 = Image()
                res.depth_image                 = Image()
                res.color_image.encoding        = "rgb8"
                res.color_image.header.frame_id = self._prefix + "/camera_optical_frame"
                res.depth_image.encoding        = "32FC1"
                res.depth_image.header.frame_id = self._prefix + "/camera_optical_frame"
                res.timestamp                   = 0.0
                
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
   
        return res

    def _service_stop_callback(self, req):
        res = StopResponse()
        res.success = True
        res.message = ''
        try:
            self._tc.stop()
        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res

    def _service_get_intrinsics_callback(self, req):
        res = GetIntrinsicsResponse()
        res.success = True
        res.message = ''
        try:
            intrinsics = self._tc.get_intrinsics()
            fx = intrinsics.fx
            fy = intrinsics.fy
            cx = intrinsics.cx
            cy = intrinsics.cy

            res.intrinsics.height           = intrinsics.height
            res.intrinsics.width            = intrinsics.width
            res.intrinsics.distortion_model = intrinsics.model
            res.intrinsics.D                = intrinsics.dist_coeffs
            res.intrinsics.K                = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            res.intrinsics.R                = [1, 0, 0, 0, 1, 0, 0, 0, 1] 
            res.intrinsics.P                = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0] 

            self._intrinsics = res

        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res

    def _service_update_intrinsics_callback(self, req):
        res = GetIntrinsicsResponse()
        res.success = True
        res.message = ''
        try:
            if(self._intrinsics != None):
                res = self._intrinsics
            res.intrinsics.header = self._header

        except Exception as e:
            res.success = False
            res.message = e.message
            rospy.logerr(e.message)
        return res