from multiprocessing import Manager

manager = Manager()

shared = manager.dict({
    "objective": None,
    "position": None,
    "state": None,
    "sensor_data": None,
    "command": None,
    "blocks_pos": [],
    "setup" : [],
    "desire" : None
})

def get_shared():
    return shared
