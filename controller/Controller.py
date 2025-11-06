import copy
import math
import random
import numpy as np
import ActService as act
import time
import SharedData

shared = SharedData.get_shared()

going = False
waiting_turn = False
waiting_going = False
safe_going_time = 3
going_time = 0
turn_time = 0
best_dir = 0

min_dist = 0.55
safe_dist = 0.7
move_log = []
block_req = 3

def controller_loop():

    #setup
    while True:
        if len(shared["setup"]) >= 4 and all(shared["setup"]):
            break

    print("[Controller] avviato")
    while True:
        global going, best_dir
        sensor = shared["sensor_data"]
        if not sensor:
            continue

        gap_len = sensor["gap_len"]
        gap_best_dir = sensor["best_dir"]
        gap_distance = sensor["gap_distance"]
        closest = sensor["closest"]
        total_point = sensor["total_point"]

        if shared["objective"] == None and len(shared["blocks_pos"]) == block_req:
            shared["objective"] = create_objective(shared["blocks_pos"])

        if shared["objective"] == []:
            shared["command"] = "stop"
            break

        best_dir = get_best_gap(gap_len, gap_best_dir, gap_distance)

        if shared["state"] == "waiting":
            print('waiting')
            shared["command"] = "stop"
        elif not best_dir or waiting_turn:
            print('turn_around')
            going = False
            turn_around()
        elif closest[0] <= min_dist or waiting_going:
            print('safe_plan')
            safe_plan(closest,total_point)
        elif shared["desire"] != None:
            print('go_to_objective' + str(shared["objective"][0]))
            print(shared["desire"])
            go_to(shared["desire"],total_point)
        elif going:
            print('going_plan')
            going = going_plan(best_dir,total_point)
        else:
            print('standing_plan')
            going = standing_plan(best_dir,total_point)
        time.sleep(0.1)
    #codice per fermare la simulazione


def go_to(desire, total_point):
    best_dir, dist = desire
    if dist <= 0.3:
        shared["command"] = 'stop'
        objectives = shared["objective"]
        if objectives:
            objectives.pop(0)
            shared["objective"] = objectives
            shared["desire"] = None
    else:
        if total_point / 2 - 10 < best_dir < total_point / 2 + 10:
            shared["command"] = 'forward'
        elif best_dir < total_point / 2:
            shared["command"] = 'right'
        else:
            shared["command"] = 'left'


def create_objective(blocks):
    bks = copy.deepcopy(blocks)
    obj = []
    for i in range(block_req):
        block = random.choice(bks)
        obj.append(block)
        bks.remove(block)
    return obj

def get_best_gap(gap_len, gap_best_dir, gap_distance):
    val = []
    max_val = 0
    for i in range(len(gap_len)):
        value = gap_len[i]*(gap_distance[i]**1/2)
        val.append(value)
        max_val = max(value,max_val)
    for i in range(len(val)):
        if val[i] >= max_val:
            return gap_best_dir[i]
    return False, False

def standing_plan(best_dir,total_point):
    if total_point / 2 - 2 < best_dir < total_point / 2 + 2:
        shared["command"] = 'forward'
        return True
    elif best_dir < total_point / 2:
        shared["command"] = 'right'
        return False
    else:
        shared["command"] = 'left'
        return False

def going_plan(ray_direction,total_point):
    if ray_direction < total_point / 3:
        shared["command"] = 'right'
        return False
    elif ray_direction > total_point / 3 * 2:
        shared["command"] = 'left'
        return False
    shared["command"] = 'forward'
    return True

def safe_plan(closest,total_point):
    global waiting_going, best_dir, going
    if waiting_going == False:
        waiting_going = True
        going = False
    if closest[0] <= safe_dist:
        if closest[1] < total_point / 4 or closest[1] > total_point / 4 * 3:
            shared["command"] = 'forward'
        elif closest[1] < total_point / 2:
            shared["command"] = 'left'
        else:
            shared["command"] = 'right'
    else:
        waiting_going = False

def turn_around():
    global waiting_turn ,turn_time
    if waiting_turn == False:
        turn_time = time.time()
        waiting_turn = True
    print(time.time() - turn_time)
    if time.time() - turn_time < act.time_90_turn*2:
        shared["command"] = 'right'
    else:
        waiting_turn = False

