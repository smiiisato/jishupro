#!/usr/bin/env python
# -*- coding: utf-8 -*-  

import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import Imu
import numpy as np
import random
from std_msgs.msg import String


def motor_command_cb(msg):
    pub01 = rospy.Publisher('/motor/command01', Int64, queue_size=1)
    pub02 = rospy.Publisher('/motor/command02', Int64, queue_size=1)
    pub03 = rospy.Publisher('/motor/command03', Int64, queue_size=1)
    pub04 = rospy.Publisher('/motor/command04', Int64, queue_size=1)
    pub05 = rospy.Publisher('/motor/command05', Int64, queue_size=1)
    pub06 = rospy.Publisher('/motor/command06', Int64, queue_size=1)

    def clear():
        pub01.publish(6900)
        pub02.publish(6900)
        pub03.publish(6900)
        pub04.publish(6900)
        pub05.publish(6900)
        pub06.publish(6900)

    motor_command_msg01 = Int64()
    motor_command_msg02 = Int64()
    motor_command_msg03 = Int64()
    motor_command_msg04 = Int64()
    motor_command_msg05 = Int64()
    motor_command_msg06 = Int64()

    pose_number = msg.data

    n = 2
    sleep_time = 0.3
    if pose_number == 0:
        for i in range(n):
            motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg02.data = random.choice([6900, 9500])
            motor_command_msg03.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            pub05.publish(motor_command_msg05)
            pub02.publish(motor_command_msg02)
            pub03.publish(motor_command_msg03)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 2:
        for i in range(n):
            motor_command_msg01.data = random.choice([6900, 9500])
            motor_command_msg02.data = random.choice([6900, 9500])
            motor_command_msg03.data = random.choice([6900, 9500])
            motor_command_msg04.data = random.choice([6900, 9500])
            pub01.publish(motor_command_msg01)
            pub02.publish(motor_command_msg02)
            pub03.publish(motor_command_msg03)
            pub04.publish(motor_command_msg04)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 3:
        for i in range(n):
            motor_command_msg01.data = random.choice([6900, 9500])
            motor_command_msg03.data = random.choice([6900, 9500])
            motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            pub01.publish(motor_command_msg01)
            pub05.publish(motor_command_msg05)
            pub03.publish(motor_command_msg03)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)

    if pose_number == 4:
        for i in range(n):
            motor_command_msg03.data = random.choice([6900, 9500])
            motor_command_msg04.data = random.choice([6900, 9500])
            motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            pub03.publish(motor_command_msg03)
            pub04.publish(motor_command_msg04)
            pub05.publish(motor_command_msg05)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 5:
        for i in range(n):
            motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg02.data = random.choice([6900, 9500])
            motor_command_msg03.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            pub05.publish(motor_command_msg05)
            pub02.publish(motor_command_msg02)
            pub03.publish(motor_command_msg03)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 6:
        for i in range(n):
            motor_command_msg02.data = random.choice([6900, 9500])
            motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            motor_command_msg01.data = random.choice([6900, 9500])
            pub01.publish(motor_command_msg01)
            pub02.publish(motor_command_msg02)
            pub05.publish(motor_command_msg05)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 8:
        for i in range(n):
            motor_command_msg01.data = random.choice([6900, 9500])
            motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            motor_command_msg02.data = random.choice([6900, 9500])
            pub01.publish(motor_command_msg01)
            pub02.publish(motor_command_msg02)
            pub05.publish(motor_command_msg05)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 9:
        for i in range(n):
            motor_command_msg01.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            motor_command_msg03.data = random.choice([6900, 9500])
            motor_command_msg05.data = random.choice([6900, 9500])
            pub01.publish(motor_command_msg01)
            pub05.publish(motor_command_msg05)
            pub03.publish(motor_command_msg03)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 10:
        for i in range(n):
            motor_command_msg02.data = random.choice([6900, 9500])
            motor_command_msg04.data = random.choice([6900, 9500])
            #motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            pub02.publish(motor_command_msg02)
            pub04.publish(motor_command_msg04)
            #pub05.publish(motor_command_msg05)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 13:
        for i in range(n):
            motor_command_msg01.data = random.choice([6900, 9500])
            motor_command_msg02.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            motor_command_msg04.data = random.choice([6900, 9500])
            motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg03.data = random.choice([6900, 9500])
            pub01.publish(motor_command_msg01)
            pub02.publish(motor_command_msg02)
            pub03.publish(motor_command_msg03)
            pub04.publish(motor_command_msg04)
            pub05.publish(motor_command_msg05)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 15:
        for i in range(n):
            motor_command_msg02.data = random.choice([6900, 9500])
            motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            motor_command_msg01.data = random.choice([6900, 9500])
            pub02.publish(motor_command_msg02)
            pub05.publish(motor_command_msg05)
            pub06.publish(motor_command_msg06)
            pub01.publish(motor_command_msg01)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 16:
        for i in range(n):
            motor_command_msg02.data = random.choice([6900, 9500])
            motor_command_msg04.data = random.choice([6900, 9500])
            motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            pub02.publish(motor_command_msg02)
            pub04.publish(motor_command_msg04)
            pub05.publish(motor_command_msg05)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 17:
        for i in range(n):
            motor_command_msg01.data = random.choice([6900, 9500])
            motor_command_msg02.data = random.choice([6900, 9500])
            motor_command_msg03.data = random.choice([6900, 9500])
            motor_command_msg04.data = random.choice([6900, 9500])
            motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            pub01.publish(motor_command_msg01)
            pub02.publish(motor_command_msg02)
            pub03.publish(motor_command_msg03)
            pub04.publish(motor_command_msg04)
            pub05.publish(motor_command_msg05)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 18:
        for i in range(n):
            motor_command_msg01.data = random.choice([6900, 9500])
            motor_command_msg02.data = random.choice([6900, 9500])
            motor_command_msg03.data = random.choice([6900, 9500])
            motor_command_msg04.data = random.choice([6900, 9500])
            motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            pub01.publish(motor_command_msg01)
            pub02.publish(motor_command_msg02)
            pub03.publish(motor_command_msg03)
            pub04.publish(motor_command_msg04)
            pub05.publish(motor_command_msg05)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 19:
        for i in range(n):
            motor_command_msg01.data = random.choice([6900, 9500])
            motor_command_msg02.data = random.choice([6900, 9500])
            #motor_command_msg03.data = random.choice([6900, 9500])
            #motor_command_msg04.data = random.choice([6900, 9500])
            #motor_command_msg05.data = random.choice([6900, 9500])
            motor_command_msg06.data = random.choice([6900, 9500])
            pub01.publish(motor_command_msg01)
            pub02.publish(motor_command_msg02)
            #pub03.publish(motor_command_msg03)
            #pub04.publish(motor_command_msg04)
            #pub05.publish(motor_command_msg05)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()




def main():
    rospy.init_node('motor_command', anonymous=False)
    rospy.Subscriber('/pose/number', Int64, motor_command_cb)
    # rospy.Subscriber('imu', Imu, motor_command_cb)                           
    rate =  rospy.Rate(5) ##change                                             
    rospy.spin()

if __name__ == '__main__':
    main()

