import time
import socketio
import json
import pyvisa

SWITCH_NAME = ""
class SwitchService:
    def __init__(self):
        rm = pyvisa.ResourceManager()
        resources = rm.list_resources()
        if SWITCH_NAME not in resources:
            print("switch not present")
            exit(1)
        else:
            self.inst = rm.open_resource(SWITCH_NAME)

    def check_online(self):
        print(self.inst.query("*IDN?"))

    def close_gate(self, gate: str):
        self.inst.query(f"ROUTe:CLOSe (@{gate})")

    def open_gate(self, gate: str):
        self.inst.query(f"ROUTe: OPEN (@{gate})")



