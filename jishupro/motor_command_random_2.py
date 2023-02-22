#!/usr/bin/env python
# -*- coding: utf-8 -*-  

import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import Imu
import numpy as np
import random
from std_msgs.msg import String

pub01 = rospy.Publisher('/motor/command01', Int64, queue_size=1)
pub02 = rospy.Publisher('/motor/command02', Int64, queue_size=1)
pub03 = rospy.Publisher('/motor/command03', Int64, queue_size=1)
pub04 = rospy.Publisher('/motor/command04', Int64, queue_size=1)
pub05 = rospy.Publisher('/motor/command05', Int64, queue_size=1)
pub06 = rospy.Publisher('/motor/command06', Int64, queue_size=1)

def motor_command_cb(msg):
#    pub01 = rospy.Publisher('/motor/command01', Int64, queue_size=1)
#    pub02 = rospy.Publisher('/motor/command02', Int64, queue_size=1)
#    pub03 = rospy.Publisher('/motor/command03', Int64, queue_size=1)
#    pub04 = rospy.Publisher('/motor/command04', Int64, queue_size=1)
#    pub05 = rospy.Publisher('/motor/command05', Int64, queue_size=1)
#    pub06 = rospy.Publisher('/motor/command06', Int64, queue_size=1)

    def clear():
        pub01.publish(7500)
        pub02.publish(7500)
        pub03.publish(7500)
        pub04.publish(7500)
        pub05.publish(7500)
        pub06.publish(7500)

    def clear3():
        pub01.publish(8000)
        pub02.publish(8000)
        pub03.publish(8000)
        pub04.publish(8000)
        pub05.publish(8000)
        pub06.publish(8000)

    def All():
        pub01.publish(9000)
        pub02.publish(9000)
        pub03.publish(9000)
        pub04.publish(9000)
        pub05.publish(9000)
        pub06.publish(9000)

    motor_command_msg01 = Int64()
    motor_command_msg02 = Int64()
    motor_command_msg03 = Int64()
    motor_command_msg04 = Int64()
    motor_command_msg05 = Int64()
    motor_command_msg06 = Int64()

    pose_number = msg.data
    print(pose_number)

    n = 1
    sleep_time = 0.3
    if pose_number == 0:
        for i in range(n):
            motor_command_msg05.data = random.choice([7500, 9500])
            motor_command_msg02.data = random.choice([7500, 9500])
            motor_command_msg03.data = random.choice([7500, 9500])
            motor_command_msg06.data = random.choice([7500, 9500])
            pub05.publish(motor_command_msg05)
            pub02.publish(motor_command_msg02)
            pub03.publish(motor_command_msg03)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 1: #123
        for i in range(n):
            motor_command_msg05.data = random.choice([7500, 9500])
            motor_command_msg02.data = random.choice([7500, 9500])
            motor_command_msg03.data = random.choice([7500, 9500])
            motor_command_msg06.data = random.choice([7500, 9500])
            pub05.publish(motor_command_msg05)
            pub02.publish(motor_command_msg02)
            pub03.publish(motor_command_msg03)
            pub06.publish(motor_command_msg06)
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 2:
        for i in range(n):
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 3: #1a-3a-7a
        for i in range(n):
	    pub01.publish(9000)
	    pub02.publish(random.choice([9000,7500]))
	    pub03.publish(9000)
	    pub05.publish(7500)
            #pub04.publish(9000)                                               
	    rospy.sleep(sleep_time)
            pub01.publish(random.choice([9000,7500]))
            pub02.publish(random.choice([9000,7500]))
            pub03.publish(random.choice([9000,7500]))
            pub05.publish(9000)
            rospy.sleep(sleep_time)
            pub01.publish(7500)
            pub02.publish(random.choice([9000,7500]))
            pub03.publish(7500)
            #pub03.publish(9000)                                               
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 4: #3a-4b-7a
        for i in range(n):
            pub03.publish(random.choice([9000,7500]))
            pub04.publish(random.choice([9000,7500]))
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 5: #2b-3b-8a #3b-4a-8b
        for i in range(n):
            pub05.publish(9000)
            pub02.publish(random.choice([9000,7500]))
            pub06.publish(9000)
            pub03.publish(7500)
            #pub02.publish(9000)                                               
            rospy.sleep(sleep_time)
            pub05.publish(random.choice([9000,7500]))
            pub02.publish(random.choice([9000,7500]))
            pub06.publish(random.choice([9000,7500]))
            pub03.publish(9000)
            rospy.sleep(sleep_time)
            pub05.publish(7500)
            pub02.publish(random.choice([9000,7500]))
            pub06.publish(7500)
            #pub03.publish(9000)                                               
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 6: #2a-7b-8a #2b-7b-8a
        for i in range(n):
            pub05.publish(random.choice([9000,7500]))
            pub06.publish(random.choice([9000,7500]))
            rospy.sleep(sleep_time)
            pub05.publish(random.choice([9000,7500]))
            pub06.publish(random.choice([9000,7500]))
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 9: #1a-3b-8b
        for i in range(n):
            pub01.publish(9000)
            pub02.publish(random.choice([9000,7500]))
            pub06.publish(9000)
            pub03.publish(7500)
            #pub02.publish(9000)                                               
            rospy.sleep(sleep_time)
            pub01.publish(random.choice([9000,7500]))
            pub02.publish(random.choice([9000,7500]))
            pub06.publish(random.choice([9000,7500]))
            pub03.publish(9000)
            rospy.sleep(sleep_time)
            pub01.publish(7500)
            pub02.publish(random.choice([9000,7500]))
            pub06.publish(7500)
            #pub03.publish(9000)                                               
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 10: #1b-2a-4a #2a-4a-8b
        for i in range(n):
            pub04.publish(9000)
            pub05.publish(random.choice([9000,7500]))
            pub06.publish(9000)
            pub02.publish(7500)
            pub03.publish(random.choice([9000,7500]))
            rospy.sleep(sleep_time)
            pub04.publish(random.choice([9000,7500]))
            pub05.publish(random.choice([9000,7500]))
            pub06.publish(random.choice([9000,7500]))
	    pub02.publish(9000)
            pub03.publish(random.choice([9000,7500]))
            rospy.sleep(sleep_time)
            pub04.publish(7500)
            pub05.publish(random.choice([9000,7500]))
            pub06.publish(7500)
            pub03.publish(random.choice([9000,7500]))
            #pub03.publish(9000)                                              
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 13: #1b-4a-8b
        for i in range(n):
            pub01.publish(9000)
	    pub03.publish(random.choice([9000,7500]))
            pub05.publish(random.choice([9000,7500]))
	    pub06.publish(9000)
	    pub04.publish(7500)
	    rospy.sleep(sleep_time)
	    pub01.publish(random.choice([9000,7500]))
	    pub03.publish(random.choice([9000,7500]))
            pub05.publish(random.choice([9000,7500]))
	    pub06.publish(random.choice([9000,8000]))
	    pub04.publish(9000)
	    rospy.sleep(sleep_time)
	    pub01.publish(7500)
	    pub03.publish(random.choice([9000,7500]))
            pub05.publish(random.choice([9000,7500]))
	    pub06.publish(7500)
	    #pub03.publish(9000)                                               
	    rospy.sleep(sleep_time)
        clear()

    if pose_number == 15:
        for i in range(n):
            pub05.publish(random.choice([9000,6900]))
            pub03.publish(random.choice([9000,6900]))
            pub04.publish(random.choice([9000,6900]))
            rospy.sleep(sleep_time)
	clear()


    if pose_number == 16: #2a-4b-7b #1b-2a-4b
        for i in range(n):
            pub05.publish(9000)
            pub03.publish(random.choice([9000,7500]))
            pub02.publish(9000)
            pub04.publish(7500)
            #pub02.publish(9000)                                               
            rospy.sleep(sleep_time)
            pub05.publish(random.choice([9000,7500]))
            pub03.publish(random.choice([9000,7500]))
            pub02.publish(random.choice([9000,7500]))
            pub04.publish(9000)
            rospy.sleep(sleep_time)
            pub05.publish(7500)
            pub03.publish(random.choice([9000,7500]))
            pub02.publish(7500)
            #pub03.publish(9000)                                               
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 17: #1b-4b-7a
        for i in range(n):
            pub01.publish(9000)
            pub06.publish(random.choice([9000,6900]))
            pub05.publish(9000)
            pub04.publish(6900)
            #pub02.publish(9000)                                               
            rospy.sleep(sleep_time)
            pub01.publish(random.choice([9000,6900]))
	    pub06.publish(random.choice([9000,6900]))
	    pub05.publish(random.choice([9000,6900]))
            pub04.publish(9000)
            rospy.sleep(sleep_time)
            pub01.publish(6900)
            pub06.publish(random.choice([9000,6900]))
            pub05.publish(6900)
            #pub03.publish(9000)                                               
            rospy.sleep(sleep_time)
        clear()

    if pose_number == 18:
	for i in range(n):
            pub05.publish(random.choice([9000,6900]))
            pub06.publish(random.choice([9000,6900]))
            rospy.sleep(sleep_time)
	clear()

    if pose_number == 19:
        for i in range(n):
            pub03.publish(random.choice([9000,6900]))
            pub04.publish(random.choice([9000,6900]))
            rospy.sleep(sleep_time)
        clear()




def main():
    rospy.init_node('motor_command', anonymous=False)
    rospy.Subscriber('/pose/number', Int64, motor_command_cb)
    # rospy.Subscriber('imu', Imu, motor_command_cb)                           
    rate =  rospy.Rate(4) ##change                                            
    rospy.spin()

if __name__ == '__main__':
    main()

