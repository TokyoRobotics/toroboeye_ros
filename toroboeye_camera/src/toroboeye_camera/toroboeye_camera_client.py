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

import rospy
import os

from pytoroboeye import toroboeye
from toroboeye_msgs.srv import *

ABSPATH = os.path.abspath(__file__)
NODE_NAME = os.path.splitext(os.path.basename(ABSPATH))[0]

def wait_for_service():
    rospy.loginfo('Node starts: ' + NODE_NAME)
    rospy.loginfo('waiting services')
    rospy.wait_for_service('connect')
    rospy.wait_for_service('disconnect')
    rospy.wait_for_service('get_capture_setting')
    rospy.wait_for_service('set_capture_setting')
    rospy.wait_for_service('write')
    rospy.wait_for_service('activate')
    rospy.wait_for_service('deactivate')
    rospy.wait_for_service('capture')
    rospy.wait_for_service('wait_for_state')
    rospy.wait_for_service('wait_for_active')
    rospy.wait_for_service('wait_for_inactive')
    rospy.wait_for_service('wait_for_frame')
    rospy.wait_for_service('stop')
    rospy.wait_for_service('get_intrinsics')
    rospy.wait_for_service('update_frame')
    rospy.wait_for_service('update_intrinsics')
   
    rospy.loginfo('finish waiting services')

def connect(ip):
    try:
        service = rospy.ServiceProxy('connect', Connect)
        rospy.loginfo(' Connecting to ToroboEye controller . . . ')
        response = service(ip, True)
        if response.success:
            rospy.loginfo('Connection Successful.')
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        # print("service call failed: %s" % e)
        rospy.logerr(" Connection Failed. Plese Check LAN Cable Connection on controller.")
        raise
    else:
        if not response.success:
            raise Exception(response.message)
        


def disconnect():
    try:
        service = rospy.ServiceProxy('disconnect',Disconnect)
        rospy.loginfo("Disconnecting...")
        response = service()
        if response.success:
            rospy.loginfo("Successful Disconnecting.")
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to Disconnect.")
        raise
    else:
        if not response.success:
            raise Exception(response.message)

def get_capture_setting():
    try:
        service = rospy.ServiceProxy('get_capture_setting',GetCaptureSetting)
        rospy.loginfo('Loading Current Setting . . .')
        response = service()
        if response.success:
            rospy.loginfo('Loading to Completed.' )
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to Loading Setting. Plese Check Controller Status")
        raise
    else:
        if not response.success:
            raise Exception(response.message)
    return response 

def get_status(id):
    try:
        service = rospy.ServiceProxy('get_status', GetStatus)
        rospy.loginfo("Checking Current Status...")
        response = service(id)
        if response.success:
            rospy.loginfo('Confirmed Status.')
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to Confirm Status.")
        raise
    else:
        if not response.success:
            raise Exception(response.message)
    return response.state

def set_capture_setting(
            device_illuminant_power = 8,
            depth_illuminant_color  = toroboeye.Setting.DEPTH.ILLUMINANT_COLOR.RED,
            depth_coding_pattern    = toroboeye.Setting.DEPTH.CODING_PATTERN.GRAYCODE_BASE,
            depth_accuracy          = 2,
            depth_exposure_time     = 1,
            depth_multiple_exposure_times = [],
            depth_adequate_score    = 40,
            color_strobe_intensity  = 4,
            color_exposure_time     = 1
            ):
    try:
        service = rospy.ServiceProxy('set_capture_setting',SetCaptureSetting)
        rospy.loginfo("Loading Current Capturing Setting. . .")
        response = service(
            device_illuminant_power,
            depth_illuminant_color ,
            depth_coding_pattern   ,
            depth_accuracy         ,
            depth_exposure_time    ,
            depth_multiple_exposure_times,
            depth_adequate_score   ,
            color_strobe_intensity ,
            color_exposure_time    ,
            ""                      
        )

        if response.success:
            rospy.loginfo("Successful Loading Capturing Setting.")
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        print("service call failed: %s" % e)
        rospy.logerr("Failed to Load Current Capturing Setting.")
        raise
    else:
        if not response.success:
            raise Exception(response.message)

def write():
    try:
        service = rospy.ServiceProxy('write',Write)
        print('[write pattern sets data into torobo eye device]')
        response = service()
        if response.success:
            rospy.loginfo('success')
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        print("service call failed: %s" % e)
        raise
    else:
        if not response.success:
            raise Exception(response.message)

def activate():
    try:
        service = rospy.ServiceProxy('activate', Activate)
        rospy.loginfo("Selected Activation.")
        response = service()
        if response.success:
            rospy.loginfo('Foward to Command...')
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.loginfo("Failed to Activate.")
        raise
    else:
        if not response.success:
            raise Exception(response.message)

def deactivate():
    try:
        service = rospy.ServiceProxy('deactivate',Deactivate)
        rospy.loginfo("Selected Deactivation")
        response = service()
        if response.success:
            rospy.loginfo("Foward Command. . .")
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to Deactivate")
        raise
    else:
        if not response.success:
            raise Exception(response.message)

def capture(oneshot = True):
    try:
        service = rospy.ServiceProxy('capture',Capture)
        rospy.loginfo('Capturing by Oneshot . . .')
        response = service(oneshot)
        if response.success:
            rospy.loginfo('Successful Oneshot Capturing')
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to Capture. Please Check Activation")
        print("service call failed: %s" % e)
        raise
    else:
        if not response.success:
            raise Exception(response.message)

def wait_for_state(activation, processing, timeout=None):
    try:
        service = rospy.ServiceProxy('wait_for_state',WaitForState)
        print('[block thread until state of torobo eye device get to specified state]')
        response = service(activation, processing, timeout)
        if response.success:
            rospy.loginfo('success')
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        print("service call failed: %s" % e)
        raise
    else:
        if not response.success:
            raise Exception(response.message)

def wait_for_active(timeout=None):
    try:
        service = rospy.ServiceProxy('wait_for_active',WaitForActive)
        rospy.loginfo("Activating. . .")
        response = service(timeout)
        if response.success:
            rospy.loginfo('Successful Activation.')
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to Activate.")
        raise
    else:
        if not response.success:
            raise Exception(response.message)

def wait_for_inactive(timeout=None):
    try:
        service = rospy.ServiceProxy('wait_for_inactive',WaitForInactive)
        rospy.loginfo('Deactivating. . .')
        response = service(timeout)
        if response.success:
            rospy.loginfo("Successful Deactication.")
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to Deativate.")
        raise
    else:
        if not response.success:
            raise Exception(response.message)

def wait_for_frame(timeout = 5.0):
    try:
        service = rospy.ServiceProxy('wait_for_frame',WaitForFrame)
        rospy.loginfo("Getting Captured Flame. . .")
        response = service(timeout)
        if response.success:
            rospy.loginfo("Successfully Getting Captured Frame.")
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to get Captured Flame.")
        raise
    else:
        if not response.success:
            raise Exception(response.message)
        return response

def update_frame(timeout = 5.0):
    try:
        service = rospy.ServiceProxy('update_frame',WaitForFrame)
        rospy.loginfo("Getting Captured Flame. . .")
        response = service(timeout)
        if response.success:
            rospy.loginfo("Successfully Getting Captured Frame.")
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to get Captured Flame.")
        raise
    else:
        if not response.success:
            raise Exception(response.message)
        return response

def stop():
    try:
        service = rospy.ServiceProxy('stop',Stop)
        rospy.loginfo("Stopping Capturing...")
        response = service()
        if response.success:
            rospy.loginfo('Successful Stoping Capturing')
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to Stop Capturing.")
        raise
    else:
        if not response.success:
            raise Exception(response.message)

def get_intrinsics():
    try:
        service = rospy.ServiceProxy('get_intrinsics', GetIntrinsics)
        rospy.loginfo('Loading Current Camera Parameter . . .')
        response = service()
        if response.success:
            rospy.loginfo('Loading Completed.')
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to Loading Camera Parameter. Plese Check Contrtoller Status.")
        raise
    else:
        if not response.success:
            raise Exception(response.message)
        return response

def update_intrinsics():
    try:
        service = rospy.ServiceProxy('update_intrinsics', GetIntrinsics)
        rospy.loginfo('Update Camera Parameter . . .')
        response = service()
        if response.success:
            rospy.loginfo('Loading Completed.')
        else:
            rospy.logerr(response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Failed to Update Camera Parameter. Plese Check Contrtoller Status.")
        raise
    else:
        if not response.success:
            raise Exception(response.message)
        return response