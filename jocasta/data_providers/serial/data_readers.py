"""Read data from serial port"""

import glob
import json
import serial
import os.path
import time

PORTS = ['ttyUSB0', 'ttyAMA0', 'ttyACM0']
OSX_PORTS = ['tty.usbserial-*']
HOST = 'pb'
PORT = 8888
SERIAL_PORT_PATH_ROOT = '/dev/'


class DataSensor:

    def __init__(self, port=None, debug=False):
        if port:
            self.serial_port = SERIAL_PORT_PATH_ROOT + port
        else:
            self.serial_port = self._detect_port()
        self.ser = serial.Serial(self.serial_port, 9600, timeout=1)

        self.debug = debug
        if self.debug:
            print("Using {} as serial port".format(self.serial_port))

    def read(self, json_data=True, timeout=10):

        if not self.serial_port:
            print("Unable to find the Arduino")
            return

        stop = time.time() + timeout
        i = 0

        while time.time() < stop:
            i = i + 1
            raw_serial = self.ser.readline()
            if self.debug:
                print("Attempt {0}: Received: {1}".format(i, raw_serial))

            if json_data and raw_serial:
                try:
                    return json.loads(raw_serial)
                except Exception as e:
                    if self.debug:
                        print(
                            "Failed to decode to JSON: {0}".format(raw_serial))
                        print(e.message)
                    pass
            elif raw_serial:
                try:
                    raw_serial = raw_serial.replace("\r\n", "")
                    return raw_serial.split(',')
                except Exception:
                    pass
            else:
                print("Failed to decode anything")

    def _detect_port(self):

        device_path = None

        for port in PORTS:
            device = SERIAL_PORT_PATH_ROOT + port
            if os.path.exists(device):
                device_path = device

        if not device_path:
            # lets try osx stuff
            for file_name in glob.glob1("/dev", "tty.usbserial-*"):
                device_path = SERIAL_PORT_PATH_ROOT + file_name
        return device_path
