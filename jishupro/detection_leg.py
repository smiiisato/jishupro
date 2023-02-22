#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
from sensor_msgs.msg import Imu

from skrobot.coordinates.math import quaternion2matrix

pose_1 = [1, 1, 1, 1]
pose_2 = [2, 2, 2, 2]
pose_3 = [3, 3, 3, 3]
pose_4 = [4, 4, 4, 4]
pose_5 = [5, 5, 5, 5]
pose_6 = [6, 6, 6, 6]
pose_7 = [7, 7, 7, 7]
pose_8 = [8, 8, 8, 8]
pose_9 = [9, 9, 9, 9]
pose_10 = [0, 0, 0, 0]
pose_11 = [1, 1, 1, 1]
pose_12 = [2, 2, 2, 2]
pose_13 = [3, 3, 3, 3]
pose_14 = [4, 4, 4, 4]
pose_15 = [5, 5, 5, 5]
pose_16 = [6, 6, 6, 6]
pose_17 = [7, 7, 7, 7]
pose_18 = [8, 8, 8, 8]
pose_19 = [9, 9, 9, 9]
pose_20 = [0, 0, 0, 0]

pose = [pose_1,
        pose_2,
        pose_3,
        pose_4,
        pose_5,
        pose_6,
        pose_7,
        pose_8,
        pose_9,
        pose_10,
        pose_11,
        pose_12,
        pose_13,
        pose_14,
        pose_15,
        pose_16,
        pose_17,
        pose_18,
        pose_19,
        pose_20]

#define callback function
def motor_command_cb(msg):
    pub = rospy.Publisher('motor/command', Int64, queue_size=1)
    motor_command_msg = Int64()
    min_num = -1
    min_error = 1000

    orientation = msg.orientation
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w
    rotation_matrix = quaternion2matrix([w, x, y, z])
    x_axis = rotation_matrix[:, 0].T
    #calculate error
    for i in range(20):
        x_ref = pose[i][0]
        y_ref = pose[i][1]
        z_ref = pose[i][2]
        w_ref = pose[i][3]
        error = (x-x_ref)**2 + (y-y_ref)**2 +(z-z_ref)**2 +(w-w_ref)**2
        # detect minimum number
        if min_error > error:
            min_num = i+1
            min_error = error
    if min_num == -1:
        print "error"
    else:
        motor_command_msg.data = min_num
        print("pose number = {}\n" .format(min_num))
        pub.publish(motor_command_msg)
                 


def main():
    rospy.init_node('motor_command', anonymous=True)
    rospy.Subscriber('/avg/quaternion', Imu, motor_command_cb)
    rate =  rospy.Rate(5) ##change
    
    rospy.spin()
    rate.sleep()
    
if __name__ == '__main__':
    main()
