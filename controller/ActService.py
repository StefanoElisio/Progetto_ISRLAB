import SimService
import SharedData
import time

shared = SharedData.get_shared()

wheel_max_speed = 10
turn_speed = 5
time_90_turn = 3.5
wheel_speed = 10
time_to_turn = 3

def act_loop():
    global sim, wheel_FR, wheel_RR, wheel_FL, wheel_RL
    _, sim = SimService.get_sim()

    wheel_FR = sim.getObject(':/front_right_wheel')
    wheel_RR = sim.getObject(':/rear_right_wheel')
    wheel_FL = sim.getObject(':/front_left_wheel')
    wheel_RL = sim.getObject(':/rear_left_wheel')

    shared["setup"] += [True]
    print("[ActService] avviato")

    while True:
        cmd = shared["command"]
        if cmd == "forward":
            go_forward()
        elif cmd == "back":
            go_back()
        elif cmd == "left":
            turn_left()
        elif cmd == "right":
            turn_right()
        elif cmd == "stop":
            stop_robot()
        time.sleep(0.1)

def on_speed_change(value):
    global wheel_speed
    wheel_speed = wheel_max_speed * value / 5
    print(wheel_speed)
    keep_state()

def change_turn_time(value):
    global time_to_turn
    time_to_turn = time_90_turn * value / 5

def go_forward():
    sim.setJointTargetVelocity(wheel_FR, -wheel_speed)
    sim.setJointTargetVelocity(wheel_RR, -wheel_speed)
    sim.setJointTargetVelocity(wheel_FL, wheel_speed)
    sim.setJointTargetVelocity(wheel_RL, wheel_speed)
    global state
    state = go_forward


def stop_robot():
    sim.setJointTargetVelocity(wheel_FR, 0)
    sim.setJointTargetVelocity(wheel_RR, 0)
    sim.setJointTargetVelocity(wheel_FL, 0)
    sim.setJointTargetVelocity(wheel_RL, 0)
    global state
    state = stop_robot

state = stop_robot

def go_back():
    sim.setJointTargetVelocity(wheel_FR, wheel_speed)
    sim.setJointTargetVelocity(wheel_RR, wheel_speed)
    sim.setJointTargetVelocity(wheel_FL, -wheel_speed)
    sim.setJointTargetVelocity(wheel_RL, -wheel_speed)
    global state
    state = go_back

def turn_left():
    sim.setJointTargetVelocity(wheel_FR, -turn_speed)
    sim.setJointTargetVelocity(wheel_RR, -turn_speed)
    sim.setJointTargetVelocity(wheel_FL, -turn_speed)
    sim.setJointTargetVelocity(wheel_RL, -turn_speed)
    global state
    state = turn_left

def turn_right():
    sim.setJointTargetVelocity(wheel_FR, turn_speed)
    sim.setJointTargetVelocity(wheel_RR, turn_speed)
    sim.setJointTargetVelocity(wheel_FL, turn_speed)
    sim.setJointTargetVelocity(wheel_RL, turn_speed)
    global state
    state = turn_right

def keep_state():
        state()