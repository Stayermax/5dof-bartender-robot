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
from constants import *

import moveit_commander
from gazebo_msgs.srv import (
    SpawnModel,
    SetModelState,
    DeleteModel,
    GetModelState
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from gazebo_msgs.msg import ModelState 

def signal_handler():
    print('Signal handler')

def spawn_model(model_name, spawn_service=None):
    if(spawn_service == None):
        spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        rospy.wait_for_service('/gazebo/spawn_sdf_model')

    scene = moveit_commander.PlanningSceneInterface()
    reference_frame = "world"
    model_path = "/home/user/Project models/"
    if('bottle' in model_name):
        # model_path += "Round bottles/"
        model_path += "Jack_bottles/"

    m_name = Names[model_name]
    m_pose = Poses[model_name]
    m_color = Colors[model_name]
    with open(model_path + m_name + "/model.sdf", "r") as mm:
        xml = mm.read().replace('\n', '')

    try:
        spawn_service(model_name, xml, "/", m_pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

    signal.signal(signal.SIGINT, signal_handler)

def spawn_all_models():
    
    spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
    for num in [1,2,3,4,5,6]:
        spawn_model("bottle_" + str(num), spawn_service)
    for num in [1,2,3]:
        spawn_model("cup_" + str(num), spawn_service)



def reset_model_position(model, wait = True):
    print("Reseting position of model " + '\"' + model + '\"')
    state_msg = ModelState()
    state_msg.model_name = model
    state_msg.pose.position.x = Poses[model].position.x
    state_msg.pose.position.y = Poses[model].position.y
    state_msg.pose.position.z = Poses[model].position.z
    state_msg.pose.orientation.x = Poses[model].orientation.x
    state_msg.pose.orientation.y = Poses[model].orientation.y
    state_msg.pose.orientation.z = Poses[model].orientation.z
    state_msg.pose.orientation.w = Poses[model].orientation.w
    
    if(wait):
        rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        set_model_state()
        

    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))

def reset_all():
    rospy.wait_for_service('/gazebo/set_model_state')
    for num in [1,2,3,4,5,6]:
        reset_model_position("bottle_" + str(num), wait = False)
    for num in [1,2,3]:
        reset_model_position("cup_" + str(num), wait = False)

if __name__ == '__main__':
    # delete_gazebo_models('bottles')
    # load_models()
    spawn_all_models()