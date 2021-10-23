#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Inverse Kinematics Pick and Place Demo
*** modified ***
modelState service
placing picture
three blocks

        Biggest part of the file was created by Rethink Robotics.
        For my project I creates the following functions:
            load_gazebo_models
            number_to_configuration
            delete_gazebo_models
            get_model_pose
            get_actual_pose
"""
import signal

import rospy
import os

from gazebo_msgs.srv import (
    SpawnModel,
    SetModelState,
    DeleteModel,
    GetModelState,
)

def signal_handler():
    print('Signal handler')

def delete_model(model_name, delete_service = None):
    if(delete_service == None):
        delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_service(model_name)
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


def delete_all(parametr='bottles'):
    delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    rospy.wait_for_service('/gazebo/delete_model')

    for num in [1,2,3,4,5,6]:
        delete_model("bottle_" + str(num), delete_service)
    for num in [1,2,3]:
        delete_model("cup_" + str(num), delete_service)

if __name__ == '__main__':
    delete_all('bottles')
    # load_models()