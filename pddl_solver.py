#!/usr/bin/env python
import os
import requests
import time

pddl_folder_path = "/home/user/catkin_ws/src/py_moveit/scripts/pddl/"

def coctails_enter():
    problem_file = open(pddl_folder_path + 'Problem.pddl', 'w')
    with open(pddl_folder_path + 'pddl_components/problem_beginning.txt') as pr_b:
        for line in pr_b:
            problem_file.write(line)
    pr_b.close()
    print('Enter name of the cocktails that you want to be prepared')
    print('Coctails options: RandS VandJ WandJ RVAJ RVAJSW RASW WAJ')
    cs = raw_input()
    if(cs==""):
        cs = 'RandS RVAJ WandJ'
    while(len(cs.split())!=3):
        print('Please enter 3 cocktails with spaces between names like this:')
        print('RandS RVAJ WandJ')
        cs = raw_input()
    one, two, three = cs.split()
    # (order RASW cup1)
    # (order RVAJSW cup2)
    # (order WAJ cup3)
    problem_file.write('    (order ' + one + ' cup1)\n')
    problem_file.write('    (order ' + two + ' cup2)\n')
    problem_file.write('    (order ' + three + ' cup3)\n')
    with open(pddl_folder_path + 'pddl_components/problem_ending.txt') as pr_e:
        for line in pr_e:
            problem_file.write(line)
    pr_e.close()

def solver():
    # pddl_folder_path = "pddl/"
    data = {'domain': open(pddl_folder_path + 'Domain.pddl', 'r').read(), 'problem': open(pddl_folder_path + 'Problem.pddl', 'r').read()}
    resp = requests.post('http://solver.planning.domains/solve',verify=False, json=data).json()
    # print(resp)
    with open(pddl_folder_path + 'plan.ipc', 'w') as f:
        for act in resp['result']['plan']:
            t = act['name'].replace('rum', 'bottle_1')
            t = t.replace('vodka', 'bottle_2')
            t = t.replace('absinthe', 'bottle_3')
            t = t.replace('juice', 'bottle_4')
            t = t.replace('soda', 'bottle_5')
            t = t.replace('whiskey', 'bottle_6')
            t = t.replace('cup1', 'cup_1')
            t = t.replace('cup2', 'cup_2')
            t = t.replace('cup3', 'cup_3')
            f.write(t + '\n')
    print('Solution stored in pddl2/plan.ipc file')

def plan_to_actions():
   file = open(pddl_folder_path + 'plan.ipc', 'r')
   plan = []
   for line in file:
       curr_action = line[1:-2].split()
       robot_action = {"action": "", "bottle": "", "cup": ""}
       robot_action['action'] = curr_action[0]
       if(curr_action[0] == "grab"):
           robot_action['bottle'] = curr_action[2]
       elif(curr_action[0] == "pour"):
           robot_action['bottle'] = curr_action[2]
           robot_action['cup'] = curr_action[3]
       elif(curr_action[0] == "release"):
           robot_action['bottle'] = curr_action[2]
       elif(curr_action[0] == "serve"):
           robot_action['cup'] = curr_action[2]
       elif(curr_action[0] == "finished"):
           pass
       plan.append(robot_action)
   return plan



if __name__ == '__main__':
    coctails_enter()
    start = time.time()
    solver()
    print('Solution time:', time.time() - start, "seconds")
    plan = plan_to_actions()
    for step in plan:
        print(step)
