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


class NekonoteEEF(object):
    def __init__(self):
        self.reso = rospy.get_param("~reso", 2.0)
        self.rs = RS304MD()
        sub = rospy.Subscriber("nekonote_eef", Float32, self.callback)
        self.s = rospy.Service("karaage_tsukamuzo", HandGrab, self.service_handle)
        self.service = rospy.Service("ganbaruzoi", CurrentLoad, self.handlehandle)
        self.rs.setTorque(1, True)
        time.sleep(2)
        rospy.spin()

    def handlehandle(self, req):
        # self.rs.setAngle(1, req.grab_angle)
        # time.sleep(2)
        # self.rs.setAngle(1, 0)
        # time.sleep(2)
        # angle = self.rs.readAngle(1)
        # if angle < 5.0:
        #     return hand_grabResponse(False)
        # else:
        #     return hand_grabResponse(True)
        time.sleep(1)
        current = self.rs.readCurrent(1)
        return CurrentLoadResponse(current)

    def service_handle(self, req):
        """
        :type req: HandGrabRequest
        :param req:
        :return: HandGrabResponse
        """
        self.rs.setAngle(1, req.hand_angle)
        time.sleep(2)
        i = req.hand_angle
        for j in range(int(req.hand_angle * self.reso), 0, -1):
            # while self.rs.readCurrent(1) < 300:
            k = j / self.reso
            self.rs.setAngle(1, k)
            time.sleep(0.001)
            if self.rs.readCurrent(1) > 400:
                time.sleep(1)
                return HandGrabResponse(True)
        time.sleep(1)
        return HandGrabResponse(False)

    def callback(self, msg):
        self.rs.setAngle(1, msg.data)
        time.sleep(0.01)

    def read(self):
        return self.rs.readCurrent(1)


if __name__ == '__main__':
    rs = RS304MD()

    for i in range(1,6):
        rs.setTorque(i, True)
        time.sleep(0.1)

    for i in range(1,6):
        rs.setAngle(i, 0)
        time.sleep(0.1)

    rs.setAngle(1,0)
    time.sleep(0.1)
    for i in range(1,8):
        rs.setAngle(2, i*10)
        time.sleep(0.1)
    time.sleep(0.1)
    for i in range(1,12):
        rs.setAngle(3, i*-10)
        time.sleep(0.1)
    time.sleep(0.1)
    for i in range(1,8):
        rs.setAngle(4, i*-10)
        time.sleep(0.1)
    time.sleep(0.1)
    rs.setAngle(5,0)
    time.sleep(0.1)
    

    for i in range(1,6):
        rs.setTorque(i, 0)
        time.sleep(0.1)
