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


from __future__ import print_function

import os
import cv2
import time
import copy
import yaml
import traceback
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor

import open3d
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *

from concurrent.futures import ThreadPoolExecutor
from pytoroboeye import toroboeye

import rospkg
import rospy
import roslib.packages
import rosnode
from cv_bridge import CvBridge
from rospy.core import is_shutdown

from toroboeye_camera import toroboeye_camera_client as tc

from std_msgs.msg import Header

class ToroboEyeOperator(Plugin):

    _update_qtui_signal = pyqtSignal('PyQt_PyObject')

    def __init__(self, context):
        super(self.__class__, self).__init__(context)
        self.setObjectName('ToroboEyeOperator')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser
        parser.add_argument("-q", "--quiet", action="store_true",
                            dest="quiet",
                            help="Put plugin in silent mode")

        args, unknown= parser.parse_known_args(context.argv())
        if not args.quiet:
            rospy.loginfo('arguments: {}'.format(args))

            if len(unknown) != 0:
                rospy.logwarn('{} is not valid Augument. Please Check The Specified Auguments '.format(unknown))

        # load ui
        self._widget = QWidget()
        ui_file = os.path.join(rospkg.RosPack().get_path('toroboeye_gui'), 'resource', 'toroboeye_operator.ui')
        loadUi(ui_file, self._widget)

        self._widget.setObjectName('ToroboEyeUi')

        context.add_widget(self._widget)
        self.serial_number = context.serial_number()

        # Create Widgets
        self._create_widget_components()

        # Member Variables
        self._captured_frame = None

        self._executor = ThreadPoolExecutor(max_workers=1)
        self._future = None
        self._vp = None
        self._intrinsics = None
        self._last_dump_dir = os.path.expanduser('~')


        self._nameSpace = rospy.get_namespace()
        if (self._nameSpace == "/"):
            self._nameSpace = "toroboeye"
        if self._nameSpace[0] == "/":
            self._nameSpace = self._nameSpace[1:]
        if self._nameSpace[-1] == "/":
            self._nameSpace = self._nameSpace[:-1]


        # Default valud of settings
        self._default_device_illuminant_power      = rospy.get_param("device_illuminant_power")
        self._default_depth_illuminant_color_index = rospy.get_param("depth_illuminant_color_index")
        self._default_depth_coding_pattern_index   = rospy.get_param("depth_coding_pattern_index")
        self._default_depth_accuracy               = rospy.get_param("depth_accuracy")
        self._default_depth_exposure_time          = rospy.get_param("depth_exposure_time")
        self._default_depth_multiple_exposure_times= rospy.get_param("depth_multiple_exposure_times")
        self._default_depth_adequate_score         = rospy.get_param("depth_adequate_score")
        self._default_color_strobe_intensity       = rospy.get_param("color_strobe_intensity")
        self._default_color_exposure_time          = rospy.get_param("color_exposure_time")

        # load last ip address
        #### connection param ####
        ip_address = rospy.get_param("device_ip_address")
        self._widget.objLineEdit_IpAddress.setText(ip_address)

        tc.wait_for_service()
        self._bridge             = CvBridge()
        self._header             = Header()
        prefix                   = rospy.get_param("prefix")
        self._header.frame_id    = prefix + '_camera_optical_frame'
        self._intrinsics_for_pcd = None

        #### Depth Filter value ####
        self._adjustment_depth_scale = rospy.get_param("adjustment_depth_scale")
        self._upper_base_limit       = rospy.get_param("upper_base_limit")
        self._lower_amplitude_limit  = rospy.get_param("lower_amplitude_limit")
        self._gradient_threshold     = rospy.get_param("gradient_threshold")
        self._edge_dilation_value    = rospy.get_param("edge_dilation_value")
        self._min_area_x             = rospy.get_param("min_area_x")
        self._min_area_y             = rospy.get_param("min_area_y")
        self._max_area_x             = rospy.get_param("max_area_x")
        self._max_area_y             = rospy.get_param("max_area_y")
        self._depth_range_min        = rospy.get_param("depth_range_min")
        self._depth_range_max        = rospy.get_param("depth_range_max")

        #### Color Filter Value ####
        self._gain_red         = rospy.get_param("colorbalance_gain_red")
        self._gain_green       = rospy.get_param("colorbalance_gain_green")
        self._gain_blue        = rospy.get_param("colorbalance_gain_blue")
        self._input_range_min  = rospy.get_param("input_level_dark")
        self._input_range_max  = rospy.get_param("input_level_light")
        self._output_range_min = rospy.get_param("output_level_dark")
        self._output_range_max = rospy.get_param("output_level_light")
        self._brightness_gamma = rospy.get_param("brightness_gamma")
        self._sharpness_level  = rospy.get_param("sharpness_level")

	    # save and laod paramter file directory 
        self._dir_path_of_toroboeye_conf = os.path.join(roslib.packages.get_pkg_dir('toroboeye_camera'), 'params')

        # set default
        self._widget.objPushButton_Settings_SetDefault.click()
        self._widget.objPushButton_Device_SetDefault.click()

    
    def shutdown_plugin(self):
        self.destroy()

    def destroy(self):
        rospy.loginfo("Close GUI")

    def _create_widget_components(self):

        # Update ui (non ui thread)
        self._update_qtui_signal.connect(self._update_qtui_slot)

        # PushButton (click)
        self._widget.objPushButton_CaptureOneShot.clicked.connect(self._slot_button_clicked)
        self._widget.objPushButton_Dump.clicked.connect(self._slot_button_clicked)
        self._widget.objPushButton_Settings_SetDefault.clicked.connect(self._slot_button_clicked)
        self._widget.objPushButton_Device_SetDefault.clicked.connect(self._slot_button_clicked)

        # PushButton (toggle)
        self._widget.objPushButton_Connect.toggled.connect(self._slot_button_toggled)
        self._widget.objPushButton_Activate.toggled.connect(self._slot_button_toggled)
        self._widget.objPushButton_CaptureContinuous.toggled.connect(self._slot_button_toggled)

        # Slider (value)
        self._widget.objSlider_Depth_Accuracy.valueChanged.connect(self._slot_value_changed)
        self._widget.objSlider_Color_StrobeIntensity.valueChanged.connect(self._slot_value_changed)
        self._widget.objSlider_Color_ExposureTime.valueChanged.connect(self._slot_value_changed)
        self._widget.objSlider_Device_IlluminantPower.valueChanged.connect(self._slot_value_changed)

        ## Action menu (push button)
        self._widget.actionSaveSettings.clicked.connect(self._slot_button_clicked)
        self._widget.actionLoadSettings.clicked.connect(self._slot_button_clicked)

        # Map for multi exposure checkboxes
        self._multi_exposure_checkbox_map = {}
        self._multi_exposure_checkbox_map[1] = self._widget.objCheckBox_Depth_ExposureTime_1
        self._multi_exposure_checkbox_map[2] = self._widget.objCheckBox_Depth_ExposureTime_2
        self._multi_exposure_checkbox_map[3] = self._widget.objCheckBox_Depth_ExposureTime_3
        self._multi_exposure_checkbox_map[4] = self._widget.objCheckBox_Depth_ExposureTime_4
        self._multi_exposure_checkbox_map[5] = self._widget.objCheckBox_Depth_ExposureTime_5
        self._multi_exposure_checkbox_map[6] = self._widget.objCheckBox_Depth_ExposureTime_6
        self._multi_exposure_checkbox_map[7] = self._widget.objCheckBox_Depth_ExposureTime_7


    def _update_qtui_slot(self, callback):
        callback()

    def _slot_button_clicked(self):
        sender = self.sender()

        if sender == self._widget.objPushButton_CaptureOneShot:
            self._executor_submit(self.__camera_capture_oneshot)

        if sender == self._widget.objPushButton_Dump:
            self.__camera_dump()

        if sender == self._widget.objPushButton_Settings_SetDefault:
            self._widget.objComboBox_Depth_IlluminantColor.setCurrentIndex(self._default_depth_illuminant_color_index)
            self._widget.objComboBox_Depth_CodingPattern.setCurrentIndex(self._default_depth_coding_pattern_index)
            self._widget.objSlider_Depth_Accuracy.setValue(self._default_depth_accuracy)
            for k, v in self._multi_exposure_checkbox_map.items():
                if k == self._default_depth_exposure_time or k in self._default_depth_multiple_exposure_times:
                    v.setChecked(True)
                else:
                    v.setChecked(False)
            self._widget.objSpinBox_Depth_Adequate_Score.setValue(self._default_depth_adequate_score)
            self._widget.objSlider_Color_StrobeIntensity.setValue(self._default_color_strobe_intensity)
            self._widget.objSlider_Color_ExposureTime.setValue(self._default_color_exposure_time)

        if sender == self._widget.objPushButton_Device_SetDefault:
            self._widget.objSlider_Device_IlluminantPower.setValue(self._default_device_illuminant_power)

        #action menu
        if sender == self._widget.actionSaveSettings:

            default_capture_settings = None
            
            with open(os.path.join(self._dir_path_of_toroboeye_conf, "param_capture_setting.yaml")) as file:
                default_capture_settings = yaml.load(file, Loader=yaml.SafeLoader)

            children = self._widget.objTabWidget.findChildren(QSlider)
            for child in children:

                if(child.objectName() == "objSlider_Device_IlluminantPower"):
                    default_capture_settings["device_illuminant_power"] = child.value()
                elif(child.objectName() == "objSlider_Depth_Accuracy"):
                    default_capture_settings["depth_accuracy"] = child.value()
                # elif(child.objectName() == "objSlider_Depth_ExposureTime"):
                #     default_capture_settings["depth_exposure_time"] = child.value()
                elif(child.objectName() == "objSlider_Color_ExposureTime"):
                    default_capture_settings["color_exposure_time"] = child.value()
                elif(child.objectName() == "objSlider_Color_StrobeIntensity"):
                    default_capture_settings["color_strobe_intensity"] = child.value()

            depth_multiple_exposure_times = []
            depth_exposure_time = 0
            for i in range(1, 8):
                if self._multi_exposure_checkbox_map[i].isChecked():
                    depth_multiple_exposure_times.append(i)
                    depth_exposure_time = i
            if len(depth_multiple_exposure_times) > 1:
                depth_exposure_time = 0
            default_capture_settings["depth_exposure_time"] = depth_exposure_time
            default_capture_settings["depth_multiple_exposure_times"] = depth_multiple_exposure_times

            default_capture_settings["depth_adequate_score"] = self._widget.objSpinBox_Depth_Adequate_Score.value()

            children = self._widget.objTabWidget.findChildren(QComboBox)
            for child in children:
                if(child.objectName() == "objComboBox_Depth_IlluminantColor"):
                    default_capture_settings["depth_illuminant_color_index"] = child.currentIndex()
                elif(child.objectName() == "objComboBox_Depth_CodingPattern"):
                    default_capture_settings["depth_coding_pattern_index"] = child.currentIndex()

            default_capture_settings["device_ip_address"] = str(self._widget.objLineEdit_IpAddress.text())

            home_dir = os.path.expanduser("~")
            dir_path_of_toroboeye_conf = os.path.join(home_dir, '.toroboeye')
            if not os.path.exists(dir_path_of_toroboeye_conf):
                os.makedirs(dir_path_of_toroboeye_conf)

            dt_object = datetime.now()
            dt_text   = dt_object.strftime('%Y-%m-%d_%H-%M-%S')

            save_file_name = "param_capture_setting_" + dt_text + ".yaml"
            dir_path_of_toroboeye_conf = os.path.join(roslib.packages.get_pkg_dir('toroboeye_camera'), 'params')
            fname = QFileDialog.getSaveFileName(parent=self._widget, caption='Save file', directory=(os.path.join(self._dir_path_of_toroboeye_conf, save_file_name)))

            if fname[0]:
                with open(fname[0], 'w') as f:
                    yaml.safe_dump(default_capture_settings, f, default_flow_style=False)
            
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText("Saved Capture Settting : " + save_file_name))
    

        if sender == self._widget.actionLoadSettings:

            fname = QFileDialog.getOpenFileName(parent=self._widget, caption='Open file', directory=(os.path.join(self._dir_path_of_toroboeye_conf, 'param_capture_setting.yaml')))

            load_parameters = None
            # fname[0] includes selected file path (with file name)
            if fname[0]:
                with open(fname[0], 'r') as f:
                    load_parameters = yaml.safe_load(f)
            else:
                return

            self._widget.objComboBox_Depth_IlluminantColor.setCurrentIndex(load_parameters["depth_illuminant_color_index"])
            self._widget.objComboBox_Depth_CodingPattern.setCurrentIndex(load_parameters["depth_coding_pattern_index"])
            self._widget.objSlider_Device_IlluminantPower.setValue(load_parameters["device_illuminant_power"])
            self._widget.objSlider_Depth_Accuracy.setValue(load_parameters["depth_accuracy"])
            # self._widget.objSlider_Depth_ExposureTime.setValue(load_parameters["depth_accuracy"])


            depth_multiple_exposure_times = load_parameters["depth_multiple_exposure_times"]
            depth_exposure_time = load_parameters["depth_exposure_time"]
            for i in range(1, 8):
                if i == depth_exposure_time or i in depth_multiple_exposure_times:
                    self._multi_exposure_checkbox_map[i].setChecked(True)
                else:
                    self._multi_exposure_checkbox_map[i].setChecked(False)

            self._widget.objSpinBox_Depth_Adequate_Score.setValue(load_parameters["depth_adequate_score"])

            self._widget.objSlider_Color_StrobeIntensity.setValue(load_parameters["color_strobe_intensity"])
            self._widget.objSlider_Color_ExposureTime.setValue(load_parameters["color_exposure_time"])
            self._widget.objLineEdit_IpAddress.setText(load_parameters["device_ip_address"])

            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText("Loaded Capture Settting Selected."))
 
    def _slot_button_toggled(self, checked):
        sender = self.sender()

        if sender == self._widget.objPushButton_Connect:
            if checked:
                self._widget.objPushButton_Connect.blockSignals(True)
                self._widget.objPushButton_Connect.setChecked(False)
                self._widget.objPushButton_Connect.blockSignals(False)
                self._executor_submit(self.__camera_connect)   
            else:
                self._widget.objPushButton_Connect.blockSignals(True)
                self._widget.objPushButton_Connect.setChecked(True)
                self._widget.objPushButton_Connect.blockSignals(False)
                self._executor_submit(self.__camera_disconnect)   

        if sender == self._widget.objPushButton_Activate:
            if checked:
                self._widget.objPushButton_Activate.blockSignals(True)
                self._widget.objPushButton_Activate.setChecked(False)
                self._widget.objPushButton_Activate.blockSignals(False)
                # if self._widget.objComboBox_Depth_IlluminantColor.currentText() == 'WHITE' and self._widget.objSlider_Depth_ExposureTime.value() <= 2:
                #     # self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText("'Settings::Depth::Exposure_time' must be '3' or more\nwhen 'Settings::Depth::Illuminant_color' is 'WHITE'"))
                #     ret = QMessageBox.warning(None, "Warning", "'Settings::Depth::Exposure_time' must be '3' or more\nwhen 'Settings::Depth::Illuminant_color' is 'WHITE'", QMessageBox.Ok) 
                #     return
                if self._widget.objComboBox_Depth_IlluminantColor.currentText() == 'WHITE' and \
                   (self._multi_exposure_checkbox_map[1].isChecked() or self._multi_exposure_checkbox_map[2].isChecked()):
                    ret = QMessageBox.warning(self, "Warning", "'Settings::Depth::Exposure_time' must be '3' or more\nwhen 'Settings::Depth::Illuminant_color' is 'WHITE'", QMessageBox.Ok)
                    return
                if (not self._multi_exposure_checkbox_map[1].isChecked() and \
                    not self._multi_exposure_checkbox_map[2].isChecked() and \
                    not self._multi_exposure_checkbox_map[3].isChecked() and \
                    not self._multi_exposure_checkbox_map[4].isChecked() and \
                    not self._multi_exposure_checkbox_map[5].isChecked() and \
                    not self._multi_exposure_checkbox_map[6].isChecked() and \
                    not self._multi_exposure_checkbox_map[7].isChecked() ):
                    ret = QMessageBox.warning(self, "Warning", "'Settings::Depth::Exposure_time' must be specified one value.", QMessageBox.Ok)
                    return
                self._executor_submit(self.__camera_activate)   
            else:
                self._widget.objPushButton_Activate.blockSignals(True)
                self._widget.objPushButton_Activate.setChecked(True)
                self._widget.objPushButton_Activate.blockSignals(False)
                self._executor_submit(self.__camera_deactivate)  

        if sender == self._widget.objPushButton_CaptureContinuous:
            if checked:
                self._widget.objPushButton_CaptureContinuous.blockSignals(True)
                self._widget.objPushButton_CaptureContinuous.setChecked(False)
                self._widget.objPushButton_CaptureContinuous.blockSignals(False)
                self._executor_submit(self.__camera_capture_continuous)   
            else:
                self._widget.objPushButton_CaptureContinuous.blockSignals(True)
                self._widget.objPushButton_CaptureContinuous.setChecked(True)
                self._widget.objPushButton_CaptureContinuous.blockSignals(False)
                self._executor_submit(self.__camera_capture_stop, forcibly=True)   


    def _slot_value_changed(self, value):
        sender = self.sender()

        if sender == self._widget.objSlider_Depth_Accuracy:
            self._widget.objLabel_Depth_Accuracy.setText(str(value))
            
        if sender == self._widget.objSlider_Color_StrobeIntensity:
            self._widget.objLabel_Color_StrobeIntensity.setText(str(value))

        if sender == self._widget.objSlider_Color_ExposureTime:
            self._widget.objLabel_Color_ExposureTime.setText(str(value))

        if sender == self._widget.objSlider_Device_IlluminantPower:
            self._widget.objLabel_Device_IlluminantPower.setText(str(value))


    def _executor_submit(self, function, forcibly=False):

        def __exception_callback(_future):
            if _future.exception() is not None:
                traceback_message = traceback.format_exc()
                print(traceback_message)
                self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText(traceback_message))

        if forcibly or (self._future is None) or self._future.done():
            self._future = self._executor.submit(function)
            
            # catch the Exception data from non main method
            self._future.add_done_callback(lambda future: __exception_callback(future))

    def __camera_connect(self):

        ip_address = str(self._widget.objLineEdit_IpAddress.text())

        # save ip_address
        home_dir = os.path.expanduser("~")
        dir_path_of_toroboeye_conf = os.path.join(home_dir, '.toroboeye')
        if not os.path.exists(dir_path_of_toroboeye_conf):
            os.makedirs(dir_path_of_toroboeye_conf)

        file_path_of_toroboeye_conf = os.path.join(dir_path_of_toroboeye_conf, 'toroboeye_conf.yaml')
        with open(file_path_of_toroboeye_conf, 'w') as f:
            yaml.safe_dump({'ip_address': ip_address}, f)


        try:
            tc.connect(ip=ip_address)
            get_response = tc.get_capture_setting()

            state_activation            = get_response.state_activation
            state_processing            = get_response.state_processing
            device_illuminant_power     = get_response.device_illuminant_power
            depth_illuminant_color      = get_response.depth_illuminant_color
            depth_coding_pattern        = get_response.depth_coding_pattern
            depth_accuracy              = get_response.depth_accuracy
            depth_exposure_time         = get_response.depth_exposure_time
            depth_multiple_exposure_times  = get_response.depth_multiple_exposure_times
            depth_adequate_score        = get_response.depth_adequate_score
            color_strobe_intensity      = get_response.color_strobe_intensity
            color_exposure_time         = get_response.color_exposure_time

            self._intrinsics            = tc.get_intrinsics().intrinsics
            self._intrinsics_for_pcd    = toroboeye.Intrinsics(
                self._intrinsics.width,              #width
                self._intrinsics.height,             #height
                self._intrinsics.K[0],               #fx
                self._intrinsics.K[4],               #fy
                self._intrinsics.K[2],               #cx
                self._intrinsics.K[5],               #cy
                self._intrinsics.D,                  #dist_coeffs
                self._intrinsics.distortion_model    #model
            )

            device_configuration        = get_response.device_configuration
            api_version                 = device_configuration.api_version

        except Exception as e:
            traceback_message = traceback.format_exc()
            rospy.loginfo(traceback_message)
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText(traceback_message))

            return

        def func():
            if state_activation == toroboeye.State.ACTIVATION.INACTIVE:
                self._widget.objPushButton_Activate.blockSignals(True)
                self._widget.objPushButton_Activate.setChecked(False)
                self._widget.objPushButton_Activate.setText('Activate')
                self._widget.objPushButton_Activate.blockSignals(False)
                self._widget.objProgressBar_Activate.blockSignals(True)
                self._widget.objProgressBar_Activate.setValue(0)
                self._widget.objProgressBar_Activate.blockSignals(False)
                self._widget.objPushButton_CaptureContinuous.blockSignals(True)
                self._widget.objPushButton_CaptureContinuous.setChecked(False)
                self._widget.objPushButton_CaptureContinuous.setText('Continuous')
                self._widget.objPushButton_CaptureContinuous.blockSignals(False)
            elif state_activation == toroboeye.State.ACTIVATION.ACTIVE:
                if state_processing != toroboeye.State.PROCESSING.CAPTURING:
                    self._widget.objPushButton_Activate.blockSignals(True)
                    self._widget.objPushButton_Activate.setChecked(True)
                    self._widget.objPushButton_Activate.setText('Activated')
                    self._widget.objPushButton_Activate.blockSignals(False)
                    self._widget.objProgressBar_Activate.blockSignals(True)
                    self._widget.objProgressBar_Activate.setValue(100)
                    self._widget.objProgressBar_Activate.blockSignals(False)
                    self._widget.objPushButton_CaptureContinuous.blockSignals(True)
                    self._widget.objPushButton_CaptureContinuous.setChecked(False)
                    self._widget.objPushButton_CaptureContinuous.setText('Continuous')
                    self._widget.objPushButton_CaptureContinuous.blockSignals(False)
                else:
                    self._widget.objPushButton_Activate.blockSignals(True)
                    self._widget.objPushButton_Activate.setChecked(True)
                    self._widget.objPushButton_Activate.setText('Activated')
                    self._widget.objPushButton_Activate.blockSignals(False)
                    self._widget.objProgressBar_Activate.blockSignals(True)
                    self._widget.objProgressBar_Activate.setValue(100)
                    self._widget.objProgressBar_Activate.blockSignals(False)
                    self._widget.objPushButton_CaptureContinuous.blockSignals(True)
                    self._widget.objPushButton_CaptureContinuous.setChecked(True)
                    self._widget.objPushButton_CaptureContinuous.setText('Stop')
                    self._widget.objPushButton_CaptureContinuous.blockSignals(False)

                    self._executor_submit(self.__camera_capture_continuous, forcibly=True)


        self._update_qtui_signal.emit(func)

        text = toroboeye.Setting.DEPTH.ILLUMINANT_COLOR(depth_illuminant_color).name
        illuminant_color_index = self._widget.objComboBox_Depth_IlluminantColor.findText(text, Qt.MatchFixedString)
        if illuminant_color_index < 0:
            raise Exception('invalid illuminant color: {}'.format(depth_illuminant_color))

        text = toroboeye.Setting.DEPTH.CODING_PATTERN(depth_coding_pattern).name
        depth_coding_pattern_index = self._widget.objComboBox_Depth_CodingPattern.findText(text, Qt.MatchFixedString)
        if depth_coding_pattern_index < 0:
            raise Exception('invalid coding pattern {}'.format(depth_coding_pattern))

        self._update_qtui_signal.emit(lambda: [
            self._widget.objSlider_Device_IlluminantPower.setValue(device_illuminant_power),
            self._widget.objComboBox_Depth_IlluminantColor.setCurrentIndex(illuminant_color_index),
            self._widget.objComboBox_Depth_CodingPattern.setCurrentIndex(depth_coding_pattern_index),
            self._widget.objSlider_Depth_Accuracy.setValue(depth_accuracy),
            # self._widget.objSlider_Depth_ExposureTime.setValue(depth_exposure_time),
            self._widget.objSpinBox_Depth_Adequate_Score.setValue(depth_adequate_score),
            self._widget.objSlider_Color_StrobeIntensity.setValue(color_strobe_intensity),
            self._widget.objSlider_Color_ExposureTime.setValue(color_exposure_time)
            ])

        if type(depth_exposure_time) is not list:
            depth_exposure_time = [depth_exposure_time]

        self._update_qtui_signal.emit(lambda: [
            self._multi_exposure_checkbox_map[1].setChecked((1 in depth_exposure_time or 1 in depth_multiple_exposure_times)),
            self._multi_exposure_checkbox_map[2].setChecked((2 in depth_exposure_time or 2 in depth_multiple_exposure_times)),
            self._multi_exposure_checkbox_map[3].setChecked((3 in depth_exposure_time or 3 in depth_multiple_exposure_times)),
            self._multi_exposure_checkbox_map[4].setChecked((4 in depth_exposure_time or 4 in depth_multiple_exposure_times)),
            self._multi_exposure_checkbox_map[5].setChecked((5 in depth_exposure_time or 5 in depth_multiple_exposure_times)),
            self._multi_exposure_checkbox_map[6].setChecked((6 in depth_exposure_time or 6 in depth_multiple_exposure_times)),
            self._multi_exposure_checkbox_map[7].setChecked((7 in depth_exposure_time or 7 in depth_multiple_exposure_times)),
            ])

        self._update_qtui_signal.emit(lambda: [
            self._widget.objLineEdit_Info_CameraPN.setText(device_configuration.camera_product_name),
            self._widget.objLineEdit_Info_CameraSN.setText(device_configuration.camera_serial_number),
            self._widget.objLineEdit_Info_ControllerSN.setText(device_configuration.controller_serial_number),
            self._widget.objLineEdit_Info_Firmware.setText(device_configuration.controller_firmware_version),
            self._widget.objLineEdit_Info_API_Version.setText(device_configuration.api_version),
            self._widget.objLineEdit_CameraParams_Width.setText(str(int(self._intrinsics_for_pcd.width))),
            self._widget.objLineEdit_CameraParams_Height.setText(str(int(self._intrinsics_for_pcd.height))),
            self._widget.objLineEdit_CameraParams_Fx.setText(str(self._intrinsics_for_pcd.fx)),
            self._widget.objLineEdit_CameraParams_Fy.setText(str(self._intrinsics_for_pcd.fy)),
            self._widget.objLineEdit_CameraParams_Cx.setText(str(self._intrinsics_for_pcd.cx)),
            self._widget.objLineEdit_CameraParams_Cy.setText(str(self._intrinsics_for_pcd.cy)),
            self._widget.objLineEdit_CameraParams_DistCoeffs.setText(str(self._intrinsics_for_pcd.dist_coeffs)),
            ])

        self._update_qtui_signal.emit(lambda: [
            self._widget.objPushButton_Connect.blockSignals(True),
            self._widget.objPushButton_Connect.setText('Connected'),
            self._widget.objPushButton_Connect.setChecked(True),
            self._widget.objPushButton_Connect.blockSignals(False),
            ])

        self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText('Connected.'))

        #add header
        self._intrinsics.header = self._header


    def __camera_disconnect(self):

        try:
            tc.disconnect()
        except Exception as e:
            traceback_message = traceback.format_exc()
            print(traceback_message)
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText(traceback_message))

        self._update_qtui_signal.emit(lambda: [
            self._widget.objPushButton_Connect.blockSignals(True),
            self._widget.objPushButton_Connect.setText('Connect'),
            self._widget.objPushButton_Connect.setChecked(False),
            self._widget.objPushButton_Connect.blockSignals(False),
            self._widget.objProgressBar_Activate.setValue(0)
            ])

    def __camera_activate(self):

        device_illuminant_power    = self._widget.objSlider_Device_IlluminantPower.value()
        depth_illuminant_color     = str(self._widget.objComboBox_Depth_IlluminantColor.currentText())
        depth_coding_pattern       = str(self._widget.objComboBox_Depth_CodingPattern.currentText())
        depth_accuracy             = self._widget.objSlider_Depth_Accuracy.value()
        # depth_exposure_time        = self._widget.objSlider_Depth_ExposureTime.value()
        color_strobe_intensity     = self._widget.objSlider_Color_StrobeIntensity.value()
        color_exposure_time        = self._widget.objSlider_Color_ExposureTime.value()

        depth_exposure_time        = self.__read_depth_exposure_time()
        depth_multiple_exposure_time = []
        if type(depth_exposure_time) is list:
            depth_multiple_exposure_times = depth_exposure_time
            depth_exposure_time    = 0
        depth_adequate_score       = self._widget.objSpinBox_Depth_Adequate_Score.value()

        try:
            tc.set_capture_setting(
                device_illuminant_power,                                                    
                getattr(toroboeye.Setting.DEPTH.ILLUMINANT_COLOR, depth_illuminant_color),  
                getattr(toroboeye.Setting.DEPTH.CODING_PATTERN,  depth_coding_pattern)  ,   
                depth_accuracy         ,
                depth_exposure_time    ,
                depth_multiple_exposure_times,
                depth_adequate_score   ,
                color_strobe_intensity ,
                color_exposure_time
            )

        except Exception as e:
            traceback_message = traceback.format_exc()
            print(traceback_message)
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText(traceback_message))
            return

        try:
            tc.activate()
        except Exception as e:
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText(str(e)))  # check restriction
            return

        try:
            tc.wait_for_active()

            self._update_qtui_signal.emit(lambda: [
                self._widget.objProgressBar_Activate.setValue(100),
                self._widget.objPlainTextEdit_Log.document().setPlainText('Finished.')
                ])

            self._update_qtui_signal.emit(lambda: [
                self._widget.objPushButton_Activate.blockSignals(True),
                self._widget.objPushButton_Activate.setText('Activated'),
                self._widget.objPushButton_Activate.setChecked(True),
                self._widget.objPushButton_Activate.blockSignals(False),
                ])
        except Exception as e:
            traceback_message = traceback.format_exc()
            print(traceback_message)
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText(traceback_message))
            return


    def __read_depth_exposure_time(self):

        depth_exposure_time_list = []

        if(self._widget.objCheckBox_Depth_ExposureTime_1.isChecked()):
            depth_exposure_time_list.append(1)
        if(self._widget.objCheckBox_Depth_ExposureTime_2.isChecked()):
            depth_exposure_time_list.append(2)
        if(self._widget.objCheckBox_Depth_ExposureTime_3.isChecked()):
            depth_exposure_time_list.append(3)
        if(self._widget.objCheckBox_Depth_ExposureTime_4.isChecked()):
            depth_exposure_time_list.append(4)
        if(self._widget.objCheckBox_Depth_ExposureTime_5.isChecked()):
            depth_exposure_time_list.append(5)
        if(self._widget.objCheckBox_Depth_ExposureTime_6.isChecked()):
            depth_exposure_time_list.append(6)
        if(self._widget.objCheckBox_Depth_ExposureTime_7.isChecked()):
            depth_exposure_time_list.append(7)
    
        return depth_exposure_time_list


    def __camera_deactivate(self):

        try:
            tc.deactivate()
            tc.wait_for_inactive()

            self._update_qtui_signal.emit(lambda: self._widget.objProgressBar_Activate.setValue(0))

            self._update_qtui_signal.emit(lambda: [
                self._widget.objPushButton_Activate.blockSignals(True),
                self._widget.objPushButton_Activate.setText('Activate'),
                self._widget.objPushButton_Activate.setChecked(False),
                self._widget.objPushButton_Activate.blockSignals(False),
                ])

        except Exception as e:
            traceback_message = traceback.format_exc()
            print(traceback_message)
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText(traceback_message))


    def __process_frame(self, frame, update_gui=True):
        
        color_image = copy.deepcopy(frame.color_image)
        depth_image = copy.deepcopy(frame.depth_image)
        score_image = copy.deepcopy(frame.score_image)

        depth_image = self._bridge.imgmsg_to_cv2(depth_image, '32FC1')
        score_image = self._bridge.imgmsg_to_cv2(score_image, "rgb8")
        color_image = self._bridge.imgmsg_to_cv2(color_image, "rgb8")
        
        depth_image = depth_image * self._adjustment_depth_scale

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
        depth_image = self._bridge.cv2_to_imgmsg(depth_image)
        color_image = self._bridge.cv2_to_imgmsg(color_image, "rgb8")

        timestamp   = copy.deepcopy(frame.timestamp)
        dt_object = datetime.fromtimestamp(timestamp)
        self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText('Succeeded at {}.'.format(dt_object)))

        return color_image, depth_image


    def __camera_capture_oneshot(self):

        try:
            tc.capture(oneshot=True)
            frame = tc.wait_for_frame(timeout=30.0)

        except Exception as e:
            traceback_message = traceback.format_exc()
            print(traceback_message)
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText(traceback_message))

        if frame.timestamp is None:
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText('Failed.'))
            return

        self._captured_frame = frame

    def __camera_capture_continuous(self):

        try:
            state_processing = tc.get_status(toroboeye.State.PROCESSING.ID)
            if state_processing != toroboeye.State.PROCESSING.CAPTURING:  
                tc.capture(oneshot=False)

            else:
                try:
                    tc.capture(oneshot=False)
                except Exception as e:
                    pass  # ignore error
                

            self._update_qtui_signal.emit(lambda: [
                self._widget.objPushButton_CaptureContinuous.blockSignals(True),
                self._widget.objPushButton_CaptureContinuous.setText('Stop'),
                self._widget.objPushButton_CaptureContinuous.setChecked(True),
                self._widget.objPushButton_CaptureContinuous.blockSignals(False),
                ])

            # time lag for changed checkbox
            while not self._widget.objPushButton_CaptureContinuous.isChecked():
                time.sleep(0.001)

            def recursive_fn():
                state_processing = tc.get_status(toroboeye.State.PROCESSING.ID)
                if state_processing != toroboeye.State.PROCESSING.CAPTURING:  

                    self._update_qtui_signal.emit(lambda: [
                        self._widget.objPushButton_CaptureContinuous.blockSignals(True),
                        self._widget.objPushButton_CaptureContinuous.setText('Continuous'),
                        self._widget.objPushButton_CaptureContinuous.setChecked(False),
                        self._widget.objPushButton_CaptureContinuous.blockSignals(False),
                        ])

                    return

                if self._widget.objPushButton_CaptureContinuous.isChecked():
                    frame = tc.wait_for_frame(timeout=5.0)
                    if frame is not None:
                        frame.color_image.header = self._header
                        frame.depth_image.header = self._header
                        self._captured_frame = frame

                    self._executor_submit(recursive_fn, forcibly=True)
            recursive_fn()

        except Exception as e:
            traceback_message = traceback.format_exc()
            print(traceback_message)
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText(traceback_message))



    def __camera_capture_stop(self):

        try:
            tc.stop()
        except Exception as e:
            # print(e)
            traceback_message = traceback.format_exc()
            print(traceback_message)
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText(traceback_message))

        self._update_qtui_signal.emit(lambda: [
            self._widget.objPushButton_CaptureContinuous.blockSignals(True),
            self._widget.objPushButton_CaptureContinuous.setText('Continuous'),
            self._widget.objPushButton_CaptureContinuous.setChecked(False),
            self._widget.objPushButton_CaptureContinuous.blockSignals(False),
            ])


    def __camera_dump(self):

        if self._captured_frame is None or self._captured_frame.timestamp is None:
            rospy.loginfo('Error: there is no frame...')
            return

        dt_object = datetime.fromtimestamp(self._captured_frame.timestamp)
        dt_text = dt_object.strftime('%Y-%m-%d_%H-%M-%S')

        path, selected_filter = QFileDialog.getSaveFileName(
            parent=self._widget, caption='Dump Data', directory=(os.path.join(os.path.expanduser('~'), dt_text)))

        if path == '':
            return
        if os.path.isfile(path):
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText('Dump error! Please specify directory.'))
            return
        if os.path.isfile(os.path.join(path, dt_text)):
            self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText('Dump error! Another file with same name exists.'))
            return

        self._last_dump_dir = path
        print(self._last_dump_dir)

        dump_dir = os.path.join(path, dt_text)
        if not os.path.exists(dump_dir):
            os.makedirs(dump_dir)

        color_image, depth_image = self.__process_frame(self._captured_frame, update_gui=False) 
        depth_image = self._bridge.imgmsg_to_cv2(depth_image, '32FC1')
        color_image = self._bridge.imgmsg_to_cv2(color_image, "rgb8")
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        if self._widget.checkBox_DumpColor.isChecked():
            cv2.imwrite(os.path.join(dump_dir, 'color.png'), color_image)
        if self._widget.checkBox_DumpDepth.isChecked():
            cv2.imwrite(os.path.join(dump_dir, 'depth.exr'), depth_image)

        if self._widget.checkBox_DumpPcd.isChecked() or self._widget.checkBox_DumpPly.isChecked():
            pcd = toroboeye.utility.pointcloud.create(color_image, depth_image, self._intrinsics_for_pcd)
            if self._widget.checkBox_DumpPcd.isChecked():
                open3d.io.write_point_cloud(os.path.join(dump_dir, 'pointcloud.pcd'), pcd)
            if self._widget.checkBox_DumpPly.isChecked():
                open3d.io.write_point_cloud(os.path.join(dump_dir, 'pointcloud.ply'), pcd)

        self._update_qtui_signal.emit(lambda: self._widget.objPlainTextEdit_Log.document().setPlainText('Dumped to ' + dump_dir))