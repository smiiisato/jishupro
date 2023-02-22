#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
import trimesh
from sensor_msgs.msg import Imu
import numpy as np

from skrobot.coordinates.math import quaternion2matrix
from skrobot.coordinates.math import quaternion_normalize
from skrobot.coordinates.math import quaternion_distance
from skrobot.coordinates.math import normalize_vector
from skrobot.coordinates import Coordinates

icosahedron = trimesh.creation.icosahedron()
# prev_q = None

pub = rospy.Publisher('/pose/number', Int64, queue_size=1)

def pose_number_cb(msg):
    # global prev_q
    # orientation = msg.orientation
    # x = orientation.x
    # y = orientation.y
    # z = orientation.z
    # w = orientation.w
    # q_wxyz = quaternion_normalize(np.array([w, x, y, z]))
    # if prev_q is not None:
    #     theta = quaternion_distance(prev_q, q_wxyz)
    #     print(np.rad2deg(theta))
    # prev_q = q_wxyz
    # rotation_matrix = quaternion2matrix(
    #     quaternion_normalize(q_wxyz))
    # x_axis = rotation_matrix[:, 0].T
    # print(quaternion_normalize(q_wxyz))
    # print(x_axis)
#    pub = rospy.Publisher('/pose/number', Int64, queue_size=1)
    pose_number_msg = Int64()
    
    x_axis = np.array([
        msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z])
    #ind = np.argmax(x_axis)
    #x_axis[ind] = x_axis[ind] + 1.2
    x_axis = normalize_vector(x_axis)
    locations, index_ray, index_tri = icosahedron.ray.intersects_location(
        ray_origins=np.array([0, 0.1, 0.10]).reshape(1, 3),
        ray_directions=x_axis.reshape(1, 3))
    print(int(index_tri[0]))
    pose_number_msg.data = int(index_tri[0])
    pub.publish(pose_number_msg)


def main():
    rospy.init_node('get_pattern', anonymous=False)
    rospy.Subscriber('/avg/linear/acceleration', Imu, pose_number_cb)
    # rospy.Subscriber('imu', Imu, motor_command_cb)
    rate =  rospy.Rate(1) ##change
    rospy.spin()
    rate.sleep()

    
if __name__ == '__main__':
    main()
