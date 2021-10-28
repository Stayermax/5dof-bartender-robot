#!/usr/bin/env python
"""
    Main algorithm file
"""

import pddl_solver as pddl
import ik
import rospy
from get_object_position import get_object_position
import time
from constants import *
from spawn_models import reset_model_position, reset_all, spawn_model, spawn_all_models
from delete_models import delete_all, delete_model

def main(new_plan = True):
    CRED = '\033[42m'
    CEND = '\033[0m'

    # 1) Initiate robot
    robot = ik.MoveGroupPythonIntefaceTutorial()
    robot.open_gripper()
    robot.go_to_init_state()
    reset_all()
    print(CRED + "ROBOT INITIATED" + CEND)
    print("\n==========================================\n")

    # 2) Choose cocktails + Transfer cocktails to PDDL + Transfer PDDL to IPC plan
    print(CRED + 'Starting project' + CEND)
    if(new_plan):
        pddl.coctails_enter()
        start = time.time()
        pddl.solver()
        print(CRED + 'PDDL solution found in ' + str(time.time() - start) + " seconds" + CEND)
    
    # 3) IPC plan to Python actions
    plan = pddl.plan_to_actions()
    print(CRED + "Actions plan:" + CEND)
    for step in plan:
        print(step)
    for step in plan:
        if (step['action'] == "grab"):
            print(CRED + "Grabbing bottle with " + BottleToDrink[step['bottle']] + CEND)
            robot.grab(step['bottle'])
        elif (step['action'] == "pour"):
            print(CRED + "Pouring " + BottleToDrink[step['bottle']]+ " to " + step['cup'] + CEND)
            robot.pour(step['cup'])
        elif (step['action'] == "release"):
            print(CRED + "Releasing bottle with " + BottleToDrink[step['bottle']]) + CEND
            robot.release()
        elif (step['action'] == "serve"):
            print(CRED + "Cocktail in " + step['cup'] + " is done" + CEND)
        elif (step['action']== "finished"):
            robot.go_to_init_state()
            print(CRED + 'All cocktails are done' + CEND)


if __name__ == '__main__':
    main()