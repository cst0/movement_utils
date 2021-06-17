#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

ODOM_MSG = Odometry()


def cmd_vel_sub(vel: Twist):
    global ODOM_MSG
    ODOM_MSG.twist.twist = vel


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
