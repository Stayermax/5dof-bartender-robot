#!/usr/bin/env python
"""
    Inverse kinematics file
"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from math import acos
from math import sqrt
from numpy import arctan2
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
# from moveit_
from get_object_position import get_object_position
from spawn_models import reset_model_position
from constants import *

from moveit_python import planning_scene_interface

def inv_kin(xp, yp, zp, joint_array):
    L = 0.54 # Length of arm elbow 
    l_base = 0.16 # base hight
    l_45 = 0.28  # length from joint 4 to joint 5
    length = sqrt(xp ** 2 + yp ** 2)

    h = abs(zp - l_base)  # distance in z direction between first joint and 4th joint
    d = length - l_45 # distance from the base to the beggining of the gripper projected on XY plane
    x = sqrt(h ** 2 + d ** 2) / 2 # half of the distance from the base to the beggining of the gripper

    alpha = acos(x / L)
    beta = acos(d / (2 * x))
    theta_0 = arctan2(yp, xp) # angle on the target

    # print("INVERCE KINEMATICS CONFIGURATION:")
    # print("ALPHA = " + str(alpha))
    # print("BETA = " + str(beta))

    theta_1 = pi / 2 - (alpha + beta)
    theta_2 = 2 * alpha
    theta_3 = -(theta_1 + theta_2) + pi / 2

    joint_array[0] = theta_0
    joint_array[1] = -theta_1
    joint_array[2] = -theta_2
    joint_array[3] = -theta_3
    joint_array[4] = 0
 
    # if(zp > l_base): # target is higher than robot base
    #     theta_1 = - (alpha + beta)
    #     theta_2 = - (2 * alpha)
    #     theta_3 =  - abs(alpha-beta)
    #     # theta_3 =  (alpha - beta)
    #     # theta_3 = pi - alpha + beta
    # else: # target is lower than robot base
    #     theta_1 = - (alpha - beta)
    #     theta_2 = - (2 * alpha)
    #     theta_3 = - (alpha+beta)

    # print("THETA 1 = " + str(theta_1))
    # print("THETA 2 = " + str(theta_2))
    # print("THETA 3 = " + str(theta_3))
    

    # joint_array[0] = theta_0 
    # joint_array[1] = theta_1 
    # joint_array[2] = theta_2
    # joint_array[3] = theta_3
    # joint_array[4] = 0

    # print(joint_array)

    return joint_array

def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

class MoveGroupPythonIntefaceTutorial(object):

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_tutorial',
                        anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        planning_frame = group.get_planning_frame()

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.robot = robot
        self.scene = scene
        self.group = group
        self.group_names = robot.get_group_names()
        # print("Robot groups: " + str(self.group_names))
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame

        self.current_bottle = None
        self.current_bottle_orig_pos = [None, None, None]
    
    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4
    ):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        box_name = 'box'
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "arm"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        # planning_scene_interface.

        return self.wait_for_state_update('box', box_is_known=True, timeout=timeout)


    def attach_object(self, box_name, timeout=4):
        grasping_group = "gripper"
        group = moveit_commander.MoveGroupCommander(grasping_group)
        eef_link = group.get_end_effector_link()
        touch_links = self.robot.get_link_names(group=grasping_group)
        print("touch_links: "+ str(touch_links))
        self.scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_name, box_is_attached=True, box_is_known=True, timeout=timeout)
       
        
    def go_to_init_state(self):
        group = self.group
        joint_goal = group.get_current_joint_values()

        joint_goal = [0, 0, 0, 0, 0]
        group.go(joint_goal, wait=True)
        rospy.sleep(1)

        # print(group.get_current_pose())
        group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_xyz(self,xp, yp, zp):
        coords = [xp, yp, zp]
        # print("Going to position" + str(coords))
        group = self.group

        joint_goal = group.get_current_joint_values()
        inv_kin(xp, yp, zp, joint_goal)

        group.go(joint_goal, wait=True, )
        # rospy.sleep(3)
        # check to see if the position values are close to xp, yp, zp:
        # print(group.get_current_pose())
        # group.stop()
        

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def rotate_gripper(self, angle):
        group = self.group

        joint_goal = group.get_current_joint_values()
        joint_goal[4] = angle
        group.go(joint_goal, wait=True)

        # check to see if the position values are close to xp, yp, zp:
        # group.stop()
        rospy.sleep(0.5)

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def grab(self, bottle):
        """
        Grabs bottle from it's position and remembers it's original position
        :param bottle: bottle name (i.e. "bootle_1")
        :return:
        """
        if(self.current_bottle != None):
            print("Arm is already holding " + self.current_bottle)
            return -1
        else:
            self.current_bottle = bottle
            # simulation
            self.current_bottle_orig_pos = get_object_position(bottle)
            self.current_bottle_orig_pos[-1] = BZP
            # real_world
            # self.current_bottle_orig_pos = Real_poses(bottle)

            x,y,z = self.current_bottle_orig_pos
            self.open_gripper()
            self.go_to_xyz(x*SDC, y*SDC, BUO) 
            self.go_to_xyz(x, y, 0.4) 
            self.go_to_xyz(x, y, BZP)
            self.close_gripper()
            # self.attach_box(bottle)
            # self.go_to_xyz(x, y, BUO)
            self.go_to_xyz(x*SDC, y*SDC, BUO)
            return 1

    def pour(self, cup):
        """
        Pours bottle to the cup
        :param cup: cup name (i.e. "cup_1")
        :return:
        """
        if (self.current_bottle == None):
            print("Arm doesn't holding anything, nothing to pour")
            return -1
        else:
            
            # simulation
            pos, angle = PourPos[cup] 
            # real_world
            # pos, angle = Real_world_PourPos[cup] 
            x, y, z = pos
 
            self.go_to_xyz(x, y, CUO)
            self.rotate_gripper(angle = angle)
            rospy.sleep(1)
            self.rotate_gripper(angle = 0)
            return 1

    def release(self):
        """
        Puts bottle on it's original position
        :return:
        """
        if (self.current_bottle == None):
            print("Arm doesn't holding anything, nothing to release")
            return -1
        else:
            x,y,z = self.get_arm_pose()
            self.go_to_xyz(x*SDC, y*SDC, BUO)

            x, y, z = self.current_bottle_orig_pos
            self.go_to_xyz(x*SDC, y*SDC, BUO)
            # self.go_to_xyz(x, y, BUO)
            self.go_to_xyz(x, y, z)
            self.open_gripper()
            # reset_model_position(self.current_bottle)
            # self.open_gripper()
            # self.go_to_xyz(x, y, BUO)
            # self.go_to_xyz(x*DSDC, y*DSDC, BUO)
            self.go_to_xyz(x*SDC, y*SDC, BUO)

            self.current_bottle = None
            self.current_bottle_orig_pos = [None, None, None]
            return 1
    
    def close_gripper(self, goal = -0.075):
        """
        Closes the gripper
        """
        group = moveit_commander.MoveGroupCommander('gripper')
        joint_goal = group.get_current_joint_values()
        # print("Current gripper joint position: " + str(joint_goal))
        joint_goal[0] = abs(goal)
        joint_goal[1] = goal
        group.go(joint_goal, wait=True)
        # print("Gripper is closed")
        rospy.sleep(1)

        # print(group.get_current_pose())
        group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
  
    def open_gripper(self):
        """
        Closes the gripper
        """
        group = moveit_commander.MoveGroupCommander('gripper')
        joint_goal = group.get_current_joint_values()
        # print("Current gripper joint position: " + str(joint_goal))
        joint_goal[0] = 0
        joint_goal[1] = 0
        group.go(joint_goal, wait=True)
        # print("Gripper is open")
        rospy.sleep(1)

        # print(group.get_current_pose())
        group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def get_arm_pose(self):
        group = moveit_commander.MoveGroupCommander('arm')
        pos = group.get_current_pose().pose.position
        x = pos.x
        y = pos.y
        z = pos.z
        coords = [x,y,z]
        return coords

    def get_gripper_pose(self):
        group = moveit_commander.MoveGroupCommander('gripper')
        pos = group.get_current_pose().pose.position
        x = pos.x
        y = pos.y
        z = pos.z
        coords = [x,y,z]
        return coords

    def get_arm_joints(self):
        group = self.group
        current_joints = group.get_current_joint_values()
        return current_joints

    def set_joints(self, state = [0,0,0,0,0]):    
        group = self.group
        joint_goal = group.get_current_joint_values()

        joint_goal = state
        print("Going to joints state: " + str(state))
        group.go(joint_goal, wait=True)
        rospy.sleep(1)

        # print(group.get_current_pose())
        group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

def main():
    try:
        robot = MoveGroupPythonIntefaceTutorial()
        robot.go_to_init_state()
        robot.close_gripper()
        # while(True):
        #     x = float(raw_input("x = "))
        #     y = float(raw_input("y = "))
        #     z = float(raw_input("z = "))
        #     robot.go_to_xyz(x,y,z)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()
