#!/usr/bin/env python
"""
Handles serial communication with the zumo robot.
"""
import serial
import time

class SerialGateway(object):

    def __init__(self, port="/dev/ttyACM0", baudrate=115200, debug=False):
        self._port = port
        self._baudrate = baudrate
        self.debug = debug

        self.buffer = []

    def start(self):
        self._serial = serial.Serial(port = self._port, \
                                     baudrate = self._baudrate, \
                                     timeout = None)
            
    def stop(self):
        time.sleep(.1)
        self._serial.close()

    def clear_buffer(self):
        self.buffer = []

    def get_raw_buffer(self):
        #print("BUFFER: --->{}<---".format(self.buffer))
        return self.buffer

    def get_buffer_as_string(self):
        s = b''.join(self.buffer).decode().strip()
        return s

    def get_buffer_as_list(self):
        s = self.get_buffer_as_string()
        return s.split(' ')

    def wait_for_buffer_fill(self, n):
        """Keep testing until the buffer containts n entries, then return."""
        while len(self.buffer) < n:
            self.buffer.append( self._serial.read() )

    def wait_for_newline(self):
        """Keep testing until the buffer ends with a newline, then return."""
        while len(self.buffer) == 0 or self.buffer[-1] != b'\n':
            #print("BUFFER:")
            #print(self.buffer)
            self.buffer.append( self._serial.read() )
        if self.debug:
            print(self.get_buffer_as_string())

    def write(self, data):
        if self.debug:
            print(data)
        self._serial.write(data)
