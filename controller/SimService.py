# SimService.py
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import traceback
import SharedData

shared = SharedData.get_shared()

def get_sim():
    # crea connessione locale, ritenta se necessario
    for attempt in range(5):
        try:
            client = RemoteAPIClient()
            sim = client.require('sim')
            return client, sim
        except Exception as e:
            print("[SimService] connessione fallita, retry:", attempt, e)
            time.sleep(1)
    raise RuntimeError("Impossibile connettersi a CoppeliaSim")

def start_sim(sim):
    try:
        sim.setStepping(True)
        if sim.getSimulationState() == 0:
            sim.startSimulation()
    except Exception:
        traceback.print_exc()
        raise

def stop_sim(client, sim):
    try:
        sim.stopSimulation()
    except Exception:
        pass
    try:
        client.exit()  # se il client supporta close/exit
    except Exception:
        pass

def sim_loop():
    shared["setup"] += [True]
    print("[SimService] avvio sim_loop")
    try:
        client, sim = get_sim()
        print("[SimService] connesso a CoppeliaSim")
        start_sim(sim)
        print("[SimService] simulazione avviata")
        while True:
            try:
                sim.step()
                time.sleep(0.05)
            except Exception as e:
                print("[SimService] errore step:", e)
                traceback.print_exc()
                time.sleep(1)
    finally:
        try:
            stop_sim(client, sim)
        except Exception:
            pass
