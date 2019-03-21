#!/usr/bin/env python
# coding: utf-8
# license removed for brevity

from math import *
from time import time, sleep
import rospy
import numpy as np
import tf2_ros
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

def pqr_to_phidot_thetadot_psidot(p, q, r, theta, phi):
    phi_dot = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta)
    theta_dot = q * cos(phi) - r * sin(phi)
    psi_dot = q * sin(phi) / cos(theta) + r * cos(phi) / cos(theta)
    return phi_dot, theta_dot, psi_dot


def toEulerAngle(i, j, k, one):
    q_w, q_x, q_y, q_z = one, i, j, k  # modified to match quaternion ordrer given by flightGoggle

    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (q_w * q_x + q_y * q_z);
    cosr_cosp = 1.0 - 2.0 * (q_x * q_x + q_y * q_y);
    roll = atan2(sinr_cosp, cosr_cosp);

    # pitch (y-axis rotation)
    sinp = +2.0 * (q_w * q_y - q_z * q_x);
    if (fabs(sinp) >= 1):
        pitch = copysign(pi / 2, sinp);  # use 90 degrees if out of range
    else:
        pitch = asin(sinp);

    # yaw (z-axis rotation)
    siny_cosp = +2.0 * (q_w * q_z + q_x * q_y);
    cosy_cosp = +1.0 - 2.0 * (q_y * q_y + q_z * q_z);
    yaw = atan2(siny_cosp, cosy_cosp);

    return (pitch, roll, yaw)


def euler_to_quaternions(roll, pitch, yaw):
    # To see if use more CPU or ram is better (time)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)

    q1 = cy * cp * cr + sy * sp * sr
    q2 = cy * cp * sr - sy * sp * cr
    q3 = sy * cp * sr + cy * sp * cr
    q4 = sy * cp * cr - cy * sp * sr
    return q1, q2, q3, q4


# Get the angles from gyro that provides the variation of angles
def get_angle_gyro(roll, pitch, yaw, gyro_x, gyro_y, gyro_z, dt):
    roll_dot = gyro_x + sin(roll) * tan(pitch) * gyro_y + cos(roll) * tan(pitch) * gyro_z
    pitch_dot = cos(roll) * gyro_y - sin(roll) * gyro_z
    yaw_dot = sin(roll) / cos(pitch) * gyro_y + cos(roll) / cos(pitch) * gyro_z

    #roll_dot, pitch_dot, yaw_dot = pqr_to_phidot_thetadot_psidot(roll_dot, pitch_dot,
    #                                                             yaw_dot, pitch, roll)
    new_roll = roll + roll_dot * dt[0]
    new_pitch = pitch + pitch_dot * dt[0]
    new_yaw = yaw + yaw_dot * dt[0]
    return new_roll, new_pitch, new_yaw


# Get the angles from the accelerometer
def get_angle_accelerometer(accelerometer_x, accelerometer_y, accelerometer_z, type='radians'):
    g = 9.80665
    pi = 3.141592
    if type == 'degre':
        roll_accelerometer = atan2(accelerometer_y, accelerometer_x) * 180 / pi
        pitch_accelerometer = atan2(accelerometer_x,
                                         sqrt((accelerometer_z + g) ^ 2 + accelerometer_y ^ 2)) * 180 / pi
        yaw_accelerometer = 0
    if type == 'radians':
        roll_accelerometer = atan2(accelerometer_y, accelerometer_x)
        pitch_accelerometer = atan2(accelerometer_x,
                                         sqrt((accelerometer_z + g) ** 2 + accelerometer_y ** 2))
        yaw_accelerometer = 0

    else:
        print('type error in get_angle_acc')
    return roll_accelerometer, pitch_accelerometer, yaw_accelerometer


def callback(data, dt):
    alpha = 0.1  # to choose between 0 and 1 preferably [0.02, 0.1] hkaya hna pk
    pitch, roll, yaw = toEulerAngle(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    roll_acc, pitch_acc, yaw_acc = get_angle_accelerometer(data.linear_acceleration.x, data.linear_acceleration.y,
                                                           data.linear_acceleration.z)

    roll_gyro, pitch_gyro, yaw_gyro = get_angle_gyro(roll, pitch, yaw, data.angular_velocity.x, data.angular_velocity.y,
                                                     data.angular_velocity.z, dt)
    roll_new = (1 - alpha) * roll_gyro + alpha * roll_acc
    pitch_new = (1 - alpha) * pitch_gyro + alpha * pitch_gyro
    # For the yaw angle we will use only the gyro so (a = 1)
    yaw_new = yaw_gyro

    print('new', roll_new, pitch_new, yaw_new)
    print('old', roll, pitch, yaw)
    sender = TFMessage()
    sender.header.stamp = rospy.Time.now()
    sender.header.frame_id = "uav/imu"

    q1, q2, q3, q4 = euler_to_quaternions(roll_new, pitch_new, yaw_new)
    sender.transform.rotation.x = q1
    sender.transform.rotation.y = q2
    sender.transform.rotation.z = q3
    sender.transform.rotation.w = q4

    #pub.publish(sender)


if __name__ == '__main__':
    rospy.init_node('data_fusion', anonymous=True)
    pub = rospy.Publisher('fused_data', TFMessage, queue_size=5)


    rate = rospy.Rate(1000)
    dt = 0.001
    while not rospy.is_shutdown():
        try:
            rospy.Subscriber("/uav/sensors/imu", Imu, callback, (dt,))
        except rospy.ROSInterruptException:
            pass

        rate.sleep()