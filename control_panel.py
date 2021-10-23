#!/usr/bin/env python
"""
    Control panel file
"""

import pddl_solver as pddl
import ik
import rospy
from get_object_position import get_object_position
import time
from constants import *
from spawn_models import reset_model_position, reset_all, spawn_model, spawn_all_models
from delete_models import delete_all, delete_model

def control_panel():
    robot = ik.MoveGroupPythonIntefaceTutorial()
    # robot.go_to_init_state()
    # robot.open_gripper()

    bottle = 'bottle_1'
    current_bottle_orig_pos = get_object_position(bottle)
    # current_bottle_orig_pos[-1] += BZS
    while(True):
        print()
        cmd = raw_input("Enter command:\n open, close, init,\n gtb, hover, gtc, move,\n pour, cb, rb, ra,\n pgr, parm, pj,\n setj, att, box,\n del, dela, spawn, exit:\n")
        if(cmd == 'open'):      # open the gripper
            robot.open_gripper()
        elif(cmd == 'close'):   # close the gripper
            goal = float(raw_input("Enter closing goal in range [-0.12; 0]:\n"))
            if(goal==""):
                goal = -0.075
            while(goal > 0 or goal < -0.12):
                goal = float(raw_input("Enter closing goal in range [-0.12; 0]:\n"))
            robot.close_gripper(goal)
        elif(cmd == 'init'):    # go to initial pose
            robot.go_to_init_state()
        elif(cmd == 'gtb'):     # go to bottle
            x,y,z = current_bottle_orig_pos
            h = raw_input("Set z level: ")
            if(h == ""): 
                h = BZS
            else:
                h = float(h)
            robot.go_to_xyz(x, y, z + h)
        elif(cmd == 'hover'):   # hover over the bottle
            x,y,z = current_bottle_orig_pos
            robot.go_to_xyz(x, y, BUO)
        elif(cmd == 'gtc'):     # go to cup
            x,y,z = get_object_position('cup_1')
            robot.go_to_xyz(x, y, CUO)
        elif(cmd == 'move'):    # go to cup
            x,y,z = robot.get_arm_pose()
            dir = raw_input("Enter coord: x,y or z:\n")
            while(dir not in ['x','y','z']):
                dir = raw_input("Enter coord: x,y or z:\n")
            step = float(raw_input("Enter step size:\n"))
            if(dir == 'x'):
                x += step
            elif(dir == 'y'):
                y += step
            elif(dir == 'z'):
                z += step
            robot.go_to_xyz(x, y, z)
        elif(cmd == 'pour'):      # turn gripper on pouring angle
            robot.rotate_gripper(angle = 1)
            rospy.sleep(1.5)
            robot.rotate_gripper(angle = 0)
        elif(cmd == 'cb'):      # change bottle
            b_n = int(raw_input("Enter bottle number from 1 to 6\n"))
            while(b_n not in [1,2,3,4,5,6]):
                b_n = int(raw_input("Enter bottle number from 1 to 6\n"))
            bottle = 'bottle_' + str(b_n)
            current_bottle_orig_pos = get_object_position(bottle)
            # current_bottle_orig_pos[-1] += BZS
        elif(cmd == 'rb'):      # reset bottle position
            reset_model_position(bottle)
        elif(cmd == 'ra'):      # reset all models positions
            reset_all()
        elif(cmd == 'pgr'):     # print gripper postiion
            pos = robot.get_gripper_pose()
            print("Current gripper coordinates: " + str(pos))
        elif(cmd == 'parm'):    # print arm postiion
            pos = robot.get_arm_pose()
            print("Current arm coordinates: " + str(pos))
        elif(cmd == 'pj'):      # print arm joints
            current_joints = robot.get_arm_joints()
            print("Current joints poistion: " + str(current_joints))
        elif(cmd == 'setj'):    # set robot joint angles
            joints = robot.get_arm_joints()
            # joints[0] = float(raw_input("Enter theta_0")) # We don't want to change the arm direction
            t1 = raw_input("Enter theta_1: ")
            t2 = raw_input("Enter theta_2: ")
            t3 = raw_input("Enter theta_3: ")
            if(t1 != ''):
                joints[1] = float(t1)
            if(t2 != ''):
                joints[2] = float(t2)
            if(t3 != ''):
                joints[3] = float(t3)
            joints[4] = 0
            robot.set_joints(joints)
        elif(cmd == 'att'): # attaches object to the gripper
            robot.attach_object(bottle)
            attached_objects = robot.scene.get_attached_objects([bottle])
            print("Attached objects: " + str(attached_objects))
        elif(cmd == 'box'):
            robot.add_box()
            robot.attach_object('box')
            attached_objects = robot.scene.get_attached_objects([bottle])
            print("Attached objects: " + str(attached_objects))
        elif(cmd == 'del'):
            delete_model(bottle)
            print("Bottle " + str(bottle.split('_')[1]) + " was deleted")
        elif(cmd == 'dela'):
            delete_all()
            print("All models were deleted")
        elif(cmd == 'spawn'):
            spawn_model(bottle)
            print("Bottle " + str(bottle.split('_')[1]) + " was spawned")
        elif(cmd == 'exit'):    # exit control panel script
            print('Finish performance')
            return
        else:
            print('Wrong command')

if __name__ == '__main__':
    control_panel()