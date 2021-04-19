import time
import socketio
import json
import pyvisa
from rehamove import *

SWITCH_NAME = "USB0::0x0957::0x3D18::MY60260004::INSTR"


class Controller:
    def __init__(self, debug: bool):
        # self.stimulator = Stimulator(debug)
        self.switch = Switch(debug)


class Stimulator:
    def __init__(self, debug: bool):
        if not debug:
            # self.port_name = "/dev/tty.usbserial-14410"
            self.port_name = "COM15"
            self.rehamove = Rehamove(self.port_name)
            i = 0
            while self.rehamove.rehamove is None:
                # self.rehamove = Rehamove(f"COM{i}")
                self.rehamove = Rehamove(f"COM{15}")
                time.sleep(0.5)
                i += 1
                if i == 16:
                    i = 0
            self.stimulator_mode = 0  # low level
            self.rehamove.change_mode(self.stimulator_mode)
        else:
            self.rehamove = None


class Switch:
    def __init__(self, debug: bool):
        self.debug = debug
        rm = pyvisa.ResourceManager()
        resources = rm.list_resources()
        print(resources)
        if not debug:
            if SWITCH_NAME not in resources:
                print("switch not present")
                exit(1)
            else:
                self.inst = rm.open_resource(SWITCH_NAME)
                print("connected to the switch!")

    def check_online(self):
        if not self.debug:
            print(self.inst.query("*IDN?"))

    def close_gate(self, gate: str):
        if not self.debug:
            self.inst.write(f"ROUTe:CLOSe (@{gate})")

    def open_gate(self, gate: str):
        if not self.debug:
            self.inst.write(f"ROUTe:OPEN (@{gate})")


CONTROLLER = None
DEBUG = False
CONTROLLER = Controller(DEBUG)
JS_BACKEND_ENDPOINT = 'http://localhost:8000'
sio = socketio.Client()


@sio.on('message')
def message(data):
    print(data)


@sio.event
def connect():
    sio.emit('connect', 'controller backend connected')
    print('interface connected')
    CONTROLLER.switch.inst.write("*CLS; *RST")
    for row in range(1, 5):
        for col in range(1, 9):
            CONTROLLER.switch.open_gate(f"{row}0{col}")


""" ~~~ STIMULATOR ~~~ """


@sio.event
def stimulator_change_mode(data):
    data = json.loads(data[0])
    print(f'change_mode: {str(data["mode"])}')
    if not DEBUG:
        CONTROLLER.stimulator.rehamove.change_mode(int(data['mode']))


@sio.event
def stimulator_low_pulse(data):
    data = json.loads(data[0])
    print('low_pulse: ', data['channel'], data['current'], data['pulse_width'])
    if not DEBUG:
        CONTROLLER.stimulator.rehamove.pulse(data['channel'], int(data['current']), int(data['pulse_width']))


@sio.event
def stimulator_mid_pulse_timed(data):
    data = json.loads(data[0])
    print('mid_pulse: ', data['channel'], data['current'], data['pulse_width'], data['period'], data['ms'])
    if not DEBUG:
        CONTROLLER.stimulator.rehamove.set_pulse(int(data['current']), int(data['pulse_width']))
        CONTROLLER.stimulator.rehamove.run(data['channel'], int(data['period']), int(data['ms']))


""" ~~~ SWITCH ~~~ """


@sio.event
def switch_button(data):
    data = json.loads(data[0])
    # gate = f"{int(data['button'][0])+1}0{int(data['button'][2])+1}"
    gate = f"30{int(data['button'][2])+1}"
    print('button', data['button'], data['state'], gate)
    if data['state'] == 1:
        CONTROLLER.switch.close_gate(gate)
    else:
        CONTROLLER.switch.open_gate(gate)

@sio.event
def disconnect():
    print('disconnected from server')


sio.connect(JS_BACKEND_ENDPOINT)
sio.wait()
