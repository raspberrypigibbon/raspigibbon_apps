#!/usr/bin/env python
# coding:utf-8

import time
import serial
import struct


class RS304MD(object):
    def __init__(self):
        SERIAL_BAUDRATE = 115200
        SERIAL_BYTESIZE = serial.EIGHTBITS
        SERIAL_PARITY = serial.PARITY_NONE
        SERIAL_STOPBIT = serial.STOPBITS_ONE
        SERIAL_TIMEOUT = 1
        self.ser = serial.Serial("/dev/ttyUSB0", SERIAL_BAUDRATE, SERIAL_BYTESIZE, SERIAL_PARITY, SERIAL_STOPBIT,
                                 SERIAL_TIMEOUT)

    def __del__(self):
        self.ser.close()

    def __bytecreateid(self, servo_id):
        return [0xFA, 0xAF, servo_id]

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
        time.sleep(0.001)

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
