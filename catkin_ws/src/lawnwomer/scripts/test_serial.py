#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author: Wuzf
@date: 2026.02.03
@brief Test RTK UM982 serial communication
"""
import serial
import rospy
from lawnwomer.msg import RTKData

SERIAL_NAME = "/dev/ttyUSB0"
BAUDRATE = 115200

def rtk_test():
    rospy.init_node("rtk_test_node", anonymous=True)
    pub = rospy.Publisher("/rtk_data", RTKData, queue_size=10)
    rate = rospy.Rate(10)

    ser = serial.Serial(SERIAL_NAME, BAUDRATE, timeout=1)
    if not ser.isOpen():
        rospy.logerr("Failed to open serial port")
        return

    while not rospy.is_shutdown():
        data = ser.readline()
        if data:
            # Parse NMEA data (simplified example)
            rtk_msg = RTKData()
            rtk_msg.latitude = 30.123456
            rtk_msg.longitude = 120.654321
            rtk_msg.altitude = 50.0
            rtk_msg.heading = 90.0
            rtk_msg.is_valid = True
            pub.publish(rtk_msg)
            rospy.loginfo("Published RTK data: lat=%f, lon=%f", rtk_msg.latitude, rtk_msg.longitude)
        rate.sleep()

    ser.close()

if __name__ == "__main__":
    try:
        rtk_test()
    except rospy.ROSInterruptException:
        pass