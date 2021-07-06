#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion, Vector3
from typing import List

ODOM_MSG = Odometry()


def euler_from_quaternion(quat: Quaternion) -> Vector3:
    angles = Vector3()

    # roll (x-axis rotation)
    sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z)
    cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)
    angles.x = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (quat.w * quat.y - quat.z * quat.x)
    if abs(sinp) >= 1:
        angles.y = math.copysign(math.pi / 2, sinp)
    else:
        angles.y = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
    angles.z = math.atan2(siny_cosp, cosy_cosp)

    return angles


def quaternion_from_euler(euler: Vector3) -> Quaternion:
    quat = Quaternion()

    cy = math.cos(euler.z * 0.5)
    sy = math.sin(euler.z * 0.5)
    cp = math.cos(euler.y * 0.5)
    sp = math.sin(euler.y * 0.5)
    cr = math.cos(euler.x * 0.5)
    sr = math.sin(euler.x * 0.5)

    quat.w = cr * cp * cy + sr * sp * sy
    quat.x = sr * cp * cy - cr * sp * sy
    quat.y = cr * sp * cy + sr * cp * sy
    quat.z = cr * cp * sy - sr * sp * cy

    return quat


def cmd_vel_sub(vel: Twist):
    """
    just assign the output of the Twist to odom, making the assumption that
    all moves take place exactly as requested
    """
    global ODOM_MSG
    # we'll need this info later
    last_time = ODOM_MSG.header.stamp
    ODOM_MSG.header.stamp = rospy.Time.now()

    # vel is easy
    ODOM_MSG.twist.twist = vel

    # change in position is trickier-- first we need to know how long since the
    # last move request, thats our delta-t
    duration = (ODOM_MSG.header.stamp - last_time).to_sec()

    # we can use that to compute change in angular position
    vector_angle = euler_from_quaternion(ODOM_MSG.pose.pose.orientation)
    vector_angle.x += vel.angular.x * duration
    vector_angle.y += vel.angular.y * duration
    vector_angle.z += vel.angular.z * duration

    # that's useful in computing our change in position, which is change in
    # position split into x and y components
    x_component = math.cos(vector_angle.z) * (vel.linear.x * duration)
    y_component = math.cos(vector_angle.z) * (vel.linear.x * duration)

    # and now we can make use of all that information to update our position.
    ODOM_MSG.pose.pose.position.x += x_component
    ODOM_MSG.pose.pose.position.y += y_component
    ODOM_MSG.pose.pose.orientation = quaternion_from_euler(vector_angle)


def spin():
    rate = rospy.Rate(10)  # todo: not hardcode rate
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_sub)
    while not rospy.is_shutdown():
        odom_pub.publish(ODOM_MSG)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("simbot_controller")
    rospy.loginfo("Velocity manager is running.")
    spin()
    rospy.loginfo("Velocity manager is closed.")
