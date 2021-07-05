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

import os

import rospy
from rospy.exceptions import ROSInterruptException

from toroboeye_camera.toroboeye_camera_publisher import ToroboeyeCameraPublisher

ABSPATH = os.path.abspath(__file__)
NODE_NAME = os.path.splitext(os.path.basename(ABSPATH))[0]

if __name__ == "__main__":

    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        rospy.loginfo('Node starts: ' + NODE_NAME)
        ToroboeyeCameraPublisher()
        rospy.spin()
    except ROSInterruptException:
        rospy.logerr("Failed to launch ToroboeyeCamera Node.") 