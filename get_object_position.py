#!/usr/bin/env python
import rospy
import os

from gazebo_msgs.srv import (
    SpawnModel,
    SetModelState,
    DeleteModel,
    GetModelState,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

def gms_client(model_name, relative_entity_name):
    """
        Returns position of chosen model in simulation world as object with properties:
            .x
            .y
            .z
    """
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        respl = gms(model_name, relative_entity_name)
        return respl
    except rospy.ServiceException, e:
        print('Service call failed: %s' % e)

def get_object_position(object):
    """
        Returns position of chosen model in simulation world as list
    """
    _pose = gms_client(object, "world").pose.position
    x = round(_pose.x,2)
    y = round(_pose.y,2)
    z = round(_pose.z,2)
    pose = [x,y,z]
    return pose

if __name__ == '__main__':
    object = 'bottle_1'
    pos = get_object_position(object)
    print(object + ' position is: ' + str(pos))