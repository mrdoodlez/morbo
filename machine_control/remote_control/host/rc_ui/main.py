# This Python file uses the following encoding: utf-8
import sys
import os
import time
import struct

from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services import Service
from adafruit_ble.uuid import VendorUUID
from adafruit_ble.characteristics import Characteristic

from PySide2.QtWidgets import QApplication, QWidget
from PySide2.QtCore import *
from PySide2.QtUiTools import QUiLoader
from PySide2 import QtGui

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
    elif cmd == CODE_STOP:
        cmd_buff += struct.pack("<HBB",   seq_num, cmd, 0)
    cmd_buff += b'\x00\x00'
    #print(cmd_buff)
    return(cmd_buff)

    def print(self, line):
        self.text += line + '\n'
        widget.ui.textBrowser.setText(self.text)

class rc_ui(QWidget):
    def __init__(self):
        super(rc_ui, self).__init__()

        self.text = ""
        self.value = 0
        self.lastcmd = 0
        self.running = False

        self.speed_scale = 0.8

        self.load_ui()
        self.setWindowTitle("MORBO RC")

        self.ui.dial.sliderPressed.connect(self.dial_pressed)
        self.ui.dial.sliderReleased.connect(self.dial_released)

        self.timer = QTimer()
        self.timer.setInterval(200)
        self.timer.timeout.connect(self.recurring_timer)
        self.timer.start()

        self.radio = BLERadio()

        print("scanning....")

        self.connection = None
        self.services = []

        for entry in self.radio.start_scan(ProvideServicesAdvertisement, timeout=60, minimum_rssi=-80):
            addr = entry.address
            name = entry.complete_name
            if name == "HMSoft":        
                print("discovered:", name, "@", addr)
                if GroveService in entry.services:
                    print("Grove service found!")
                    self.connection = self.radio.connect(entry)
                    break

        self.radio.stop_scan()
        print("scan done")

        if self.connection and self.connection.connected:
            print("connected")
            self.grove = self.connection[GroveService]
        
        self.show()

    def recurring_timer(self):
        value = self.ui.dial.value() // 5
        value *= 5
        value -= 180
        if value < 0:
            value += 360

        if value != self.value:
            self.value = value
            if self.running:
                self.set_speeds()
    
        if (time.time() - self.lastcmd) > 2:
            self.ping()

    def dial_pressed(self):
        self.set_speeds()
        self.running = True
            
    def dial_released(self):
        self.stop()
        self.running = False
        self.value = 0
        self.ui.dial.setValue(180)

    def set_speeds(self):
        v0 = 0
        l0 = 0
        l1 = 0
        r0 = 0
        r1 = 0        
        if self.value >= 0 and self.value < 90:
            v0 =  0
            l0 =  1
            l1 =  1
            r0 =  1
            r1 = -1
        elif self.value >= 90 and self.value < 180:
            v0 = 90
            l0 =  1
            l1 = -1
            r0 = -1
            r1 = -1
        elif self.value >= 180 and self.value < 270:
            v0 = 180
            l0 = -1
            l1 = -1
            r0 = -1
            r1 =  1
        else:
            v0 = 270
            l0 = -1
            l1 =  1
            r0 =  1
            r1 =  1

        scale = (self.value - v0) / 90

        l = l0 + (l1 - l0) * scale
        r = r0 + (r1 - r0) * scale

        l *= self.speed_scale
        r *= self.speed_scale

        l *= 127
        r *= 127

        if l < 0: l = 128 - l
        if r < 0: r = 128 - r

        l = int(l)
        r = int(r)

        self.print("> speeds: {:02x}; {:02x}".format(l, r))
        self.grove[0] = make_command(CODE_SPEEDS, l, r)
        self.lastcmd = time.time()

    def stop(self):
        self.print("> stop")
        self.grove[0] = make_command(CODE_STOP)
        self.lastcmd = time.time()

    def ping(self):
        self.print("> ping")
        self.grove[0] = make_command(CODE_PING)
        self.lastcmd = time.time()
    
    def print(self, line):
        self.text += line + '\n'
        self.ui.textBrowser.setText(self.text)
        self.ui.textBrowser.verticalScrollBar().setValue(10000)

    def load_ui(self):
        loader = QUiLoader()
        path = os.path.join(os.path.dirname(__file__), "form.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        self.ui = loader.load(ui_file, self)
        ui_file.close()

if __name__ == "__main__":
    app = QApplication([])
    widget = rc_ui()
    sys.exit(app.exec_())
