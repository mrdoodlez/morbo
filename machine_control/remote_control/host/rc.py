from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services import Service
from adafruit_ble.uuid import VendorUUID
from adafruit_ble.characteristics import Characteristic
import sys
import struct

class GroveService(Service):
    uuid = VendorUUID("0000ffe0-0000-1000-8000-00805f9b34fb")

    _control = Characteristic(
        uuid=VendorUUID("0000ffe1-0000-1000-8000-00805f9b34fb"), max_length=20
    )

    def __init__(self, service=None):
        super().__init__(service=service)

    def __setitem__(self, index, value):
        self._control = value
    
    def __getitem__(self, index):
        return self._control

    def __len__(self):
        return 1

seq_num = 0

CODE_ACK    = 0
CODE_NACK   = 1

CODE_PING   = 2

CODE_STOP   = 3
CODE_SPEEDS = 4

def make_command(cmd, arg1 = 0, arg2 = 0):
    cmd_buff = b'mb'
    if cmd == CODE_PING:
        cmd_buff += struct.pack("<HBB",   seq_num, cmd, 0)
    elif cmd == CODE_SPEEDS:
        cmd_buff += struct.pack("<HBBBB", seq_num, cmd, 2, arg1, arg2)
    cmd_buff += b'\x00\x00'
    print(cmd_buff)
    return(cmd_buff)

radio = BLERadio()

print("scanning....")

connection = None
services = []

for entry in radio.start_scan(ProvideServicesAdvertisement, timeout=60, minimum_rssi=-80):
    addr = entry.address
    name = entry.complete_name
    if name == "HMSoft":        
        print("discovered:", name, "@", addr)
        if GroveService in entry.services:
            print("Grove service found!")
            connection = radio.connect(entry)
            break

radio.stop_scan()
print("scan done")

if connection and connection.connected:
    print("connected")
    grove = connection[GroveService]
    stop = False
    while not stop:
        cmd = input("> ")
        if cmd == "stop":            
            stop = True
        else:
            cmd = cmd.split()
            if cmd[0] == "s":
                speed_l = int(cmd[1])
                speed_r = int(cmd[2])
                grove[0] = make_command(CODE_SPEEDS, speed_l, speed_r)


    grove[0] = make_command(CODE_SPEEDS, 0, 0)

connection.disconnect()

