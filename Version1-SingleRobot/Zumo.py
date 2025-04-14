"""
Definitions and utilities for interfacing with the Zumo robot via serial.  It
must be running the ZumoSerial sketch and be connected via USB.
"""

from SerialGateway import SerialGateway
from math import pi

class Zumo:

    def __init__(self):
        self.gateway = SerialGateway()
        self.gateway.start()
        self.heading = 0
        
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.send_speeds(0, 0)
        self.gateway.stop()

    def send_speeds(self, left_speed, right_speed):
        """Sends the given speeds to the left and right motors."""

        str_cmd = 's {} {}'.format(left_speed, right_speed)
        expected_response = '{} {}'.format(left_speed, right_speed)
        self.gateway.clear_buffer()
        self.gateway.write(bytes(str_cmd, encoding='utf-8'))

        self.gateway.wait_for_newline()
        s = self.gateway.get_buffer_as_string()
#        if s != expected_response:
 #           print("Expected '{}' but got '{}'.".format(expected_response, s))

    def get_line_sensors(self):
        self.gateway.clear_buffer()
        self.gateway.write(bytes('l', encoding='utf-8'))
        self.gateway.wait_for_newline()

        raw_result = list(map(int, self.gateway.get_buffer_as_list()))
        return raw_result

        # Inverting so that the results are proportional to brightness
        # result = [2000 - x for x in raw_result]
        # return result

    def reset_encoders(self):
        self.gateway.clear_buffer()
        self.gateway.write(bytes('r', encoding='utf-8'))
        self.gateway.wait_for_newline()

        s = self.gateway.get_buffer_as_string()
        expected_response = '0 0'
        if s != expected_response:
            print("Expected '{}' but got '{}'.".format(expected_response, s))

    def get_encoders(self):
        self.gateway.clear_buffer()
        self.gateway.write(bytes('e', encoding='utf-8'))
        self.gateway.wait_for_newline()

        result = list(map(int, self.gateway.get_buffer_as_list()))

        return result

    def get_battery_and_usb(self):
        self.gateway.clear_buffer()
        self.gateway.write(bytes('b', encoding='utf-8'))
        self.gateway.wait_for_newline()

        result = list(map(int, self.gateway.get_buffer_as_list()))

        return result
