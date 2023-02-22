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

    motor_command_msg01 = Int64()
    motor_command_msg02 = Int64()
    motor_command_msg03 = Int64()
    motor_command_msg04 = Int64()
    motor_command_msg05 = Int64()
    motor_command_msg06 = Int64()

    pose_number = msg.data
    print(pose_number)

    n = 1
    sleep_time = 0.2

                              
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

    def all():
        pub01.publish(9000)
        pub02.publish(9000)
        pub03.publish(9000)
        pub04.publish(9000)
        pub05.publish(9000)
	pub06.publish(9000)


    if pose_number == 0:
	for i in range(n):
            pub05.publish(9000)
            pub03.publish(9000)
            pub02.publish(7500)
            pub01.publish(7500)
            #pub02.publish(9000)                                               
            rospy.sleep(sleep_time)
            #pub06.publish(7000)                                               
            pub05.publish(7500)
            pub03.publish(7500)
            pub02.publish(9000)
            pub01.publish(9000)
            #pub02.publish(7000)                                               
            rospy.sleep(sleep_time)
	clear3()

    elif pose_number == 3:
        for i in range(n):
            pub05.publish(9000)
            pub03.publish(9000)
            pub01.publish(7500)
            #pub01.publish(7500)                                               
            #pub02.publish(9000)                                               
            rospy.sleep(sleep_time)
            #pub06.publish(7000)                                               
            pub05.publish(7500)
            pub03.publish(7500)
            pub01.publish(9000)
            #pub01.publish(7500)                                               
            #pub02.publish(7000)                                               
            rospy.sleep(sleep_time)
        clear3()

    elif pose_number == 5:
        for i in range(n):
            pub03.publish(9000)
            pub02.publish(9000)
            pub06.publish(7500)
            pub05.publish(7500)
            #pub02.publish(9000)                                               
            rospy.sleep(sleep_time)
            #pub06.publish(7000)                                               
            pub03.publish(7500)
            pub02.publish(7500)
            pub06.publish(9000)
            pub05.publish(9000)
            #pub02.publish(7000)                                               
            rospy.sleep(sleep_time)
        clear3()
        
    elif pose_number == 9:
	for i in range(n):
            pub06.publish(9000)
            pub03.publish(9000)
            pub01.publish(7500)
            #pub01.publish(7500)                                               
            #pub02.publish(9000)                                               
            rospy.sleep(sleep_time)
            #pub06.publish(7000)                                               
            pub06.publish(7500)
            pub03.publish(7500)
            pub01.publish(9000)
            #pub01.publish(7500)                                               
            #pub02.publish(7000)                                               
            rospy.sleep(sleep_time)
        clear3()

    elif pose_number == 10: #2-4-8
	for i in range(n):
            pub06.publish(9000)
            pub04.publish(9000)
            pub02.publish(7500)
            #pub01.publish(7500)                                               
            #pub02.publish(9000)                                               
            rospy.sleep(sleep_time)
            #pub06.publish(7000)                                               
            pub06.publish(7500)
            pub04.publish(7500)
            pub02.publish(9000)
            #pub01.publish(7500)                                               
            #pub02.publish(7000)                                               
            rospy.sleep(sleep_time)
        clear3()

    elif pose_number == 13:
	for i in range(n):  
            pub06.publish(9000)
            pub04.publish(9000)
            pub01.publish(7500)
            #pub01.publish(7500)                                               
            #pub02.publish(9000)                                               
            rospy.sleep(sleep_time)
            #pub06.publish(7000)                                               
            pub06.publish(7500)
            pub04.publish(7500)
            pub01.publish(9000)
            #pub01.publish(7500)                                               
            #pub02.publish(7000)                                               
            rospy.sleep(sleep_time)
        clear3()

    elif pose_number == 16:
        for i in range(n):
            pub05.publish(9000)
            pub04.publish(9000)
            pub02.publish(7500)
            #pub01.publish(7500)                                               
            #pub02.publish(9000)                                               
            rospy.sleep(sleep_time)
            #pub06.publish(7000)                                               
            pub05.publish(7500)
            pub04.publish(7500)
            pub02.publish(9000)
            #pub01.publish(7500)                                               
            #pub02.publish(7000)                                               
            rospy.sleep(sleep_time)
        clear3()

    elif pose_number == 17:
	for i in range(n):
            pub05.publish(9000)
            pub04.publish(9000)
            pub01.publish(7500)
            #pub01.publish(7500)                                               
            #pub02.publish(9000)                                               
            rospy.sleep(sleep_time)
            #pub06.publish(7000)                                               
            pub05.publish(7500)
            pub04.publish(7500)
            pub01.publish(9000)
            #pub01.publish(7500)                                               
            #pub02.publish(7000)                                               
            rospy.sleep(sleep_time)
        clear3()
        
    else:
        all()
        rospy.sleep(sleep_time)
        clear3()

def main():
    rospy.init_node('motor_command', anonymous=False)
    rospy.Subscriber('/pose/number', Int64, motor_command_cb)
    # rospy.Subscriber('imu', Imu, motor_command_cb)                           
    rate =  rospy.Rate(4) ##change                                             
    rospy.spin()

if __name__ == '__main__':
    main()

            
        
