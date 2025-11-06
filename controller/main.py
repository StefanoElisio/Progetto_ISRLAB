import ActService
import SimService
from pynput import keyboard
import time
import Controller
import SenseService as sense
import CameraService as cam
import SharedData
from multiprocessing import Process

turning = False

# Stato dei tasti1
pressed_keys = set()
# Listener per tasti premuti
def on_press(key):
    try:
        if key.char:
            pressed_keys.add(key.char)
    except AttributeError:
        pressed_keys.add(key)
# Listener per tasti rilasciati
def on_release(key):
    try:
        if key.char:
            pressed_keys.discard(key.char)
    except AttributeError:
        pressed_keys.discard(key)
    if key == keyboard.Key.esc:
        return False  # Interrompe il listener


if __name__ == "__main__":
    # Avvia il listener da tastiera in background
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    shared = SharedData.get_shared()

    p_sim = Process(target=SimService.sim_loop)
    p_act = Process(target=ActService.act_loop)
    p_sense = Process(target=sense.sense_loop)
    p_cam = Process(target=cam.camera_loop)
    p_ctrl = Process(target=Controller.controller_loop)

    processes = [p_sim, p_act, p_sense, p_cam, p_ctrl]
    client,sim = SimService.get_sim()

    for p in processes:
        p.start()

    try:
        while True:
            if turning:
                # Controllo se ho finito di girare
                if time.time() - turn_start_time >= ActService.time_to_turn:
                    ActService.stop_robot()
                    turning = False

            # Controllo dei tasti
            if 'w' in pressed_keys:
                ActService.stop_robot()
                ActService.go_forward()
                turning = False
            elif 's' in pressed_keys:
                ActService.stop_robot()
                ActService.go_back()
                turning = False
            elif 'a' in pressed_keys:
                ActService.stop_robot()
                ActService.turn_left()
                turn_start_time = time.time()
                turning = True
            elif 'd' in pressed_keys:
                ActService.stop_robot()
                ActService.turn_right()
                turn_start_time = time.time()
                turning = True
            elif 'q' in pressed_keys:
                ActService.stop_robot()
                ActService.turn_left()
            elif 'e' in pressed_keys:
                ActService.stop_robot()
                ActService.turn_right()
            elif 'x' in pressed_keys:
                ActService.stop_robot()
            elif '1' in pressed_keys:
                ActService.on_speed_change(1)
            elif '2' in pressed_keys:
                ActService.on_speed_change(2)
            elif '3' in pressed_keys:
                ActService.on_speed_change(3)
            elif '4' in pressed_keys:
                ActService.on_speed_change(4)
            elif '5' in pressed_keys:
                ActService.on_speed_change(5)
            elif '6' in pressed_keys:
                ActService.change_turn_time(1)
            elif '7' in pressed_keys:
                ActService.change_turn_time(2)
            elif '8' in pressed_keys:
                ActService.change_turn_time(3)
            elif '9' in pressed_keys:
                ActService.change_turn_time(4)
            elif '0' in pressed_keys:
                ActService.change_turn_time(5)
            elif 'z' in pressed_keys:
                sense.show_map()
            else:
                # Nessun tasto rilevante premuto
                pass

            time.sleep(0.03) # Piccola pausa per non sovraccaricare la CPU

    except KeyboardInterrupt:
        print("Interruzione manuale.")
    finally:
        print("Chiudo la simulazione.")
        SimService.stop_sim(client,sim)
        listener.stop()
        for p in processes:
            if p.is_alive():
                p.terminate()
                p.join()