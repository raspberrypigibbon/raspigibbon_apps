#!/usr/bin/env python
# coding:utf-8

"""
RS304MD.py

Copyright (c) 2017 Shota Hirama

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import serial
import struct


class RS304MD(object):
    def __init__(self):
        PORT = "/dev/ttyUSB0"
        BAUDRATE = 115200
        BYTESIZE = serial.EIGHTBITS
        PARITY = serial.PARITY_NONE
        STOPBIT = serial.STOPBITS_ONE
        TIMEOUT = 1
        self.ser = serial.Serial(PORT, BAUDRATE, BYTESIZE, PARITY, STOPBIT, TIMEOUT)

    def __del__(self):
        self.ser.close()

    def __bytecreateid(self, id):
        return [0xFA, 0xAF, id]

    def __checksum(self, checklist):
        sum = 0
        for i in range(2, len(checklist)):
            sum ^= checklist[i]
        checklist.append(sum)
        return checklist

    def __write(self, servolist):
        self.ser.write("".join(map(chr, servolist)))

    def __requestStatus(self, servo_id):
        a = self.__bytecreateid(servo_id)
        a.extend([0x09, 0x00, 0x00, 0x01])
        self.__write(self.__checksum(a))

    def setAngle(self, servo_id, set_angle):
        angle = max(-150.0, min(150.0, set_angle))
        angle = int(angle * 10)
        a = self.__bytecreateid(servo_id)
        a.extend([0x00, 0x1E, 0x02, 0x01, angle & 0xFF, (angle & 0xFF00) >> 8])
        self.__write(self.__checksum(a))

    def setTorque(self, servo_id, onoff):
        a = self.__bytecreateid(servo_id)
        a.extend([0x00, 0x24, 0x01, 0x01, int(onoff)])
        self.__write(self.__checksum(a))

    def readAngle(self, servo_id):
        self.__requestStatus(servo_id)
        b = self.ser.read(26)[7:9]
        return struct.unpack("<h", b)[0] / 10.0

    def readCurrent(self, servo_id):
        self.__requestStatus(servo_id)
        b = self.ser.read(26)[13:15]
        return struct.unpack("<h", b)[0]
