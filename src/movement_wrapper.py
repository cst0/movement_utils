#!/usr/bin/env python3

import sys
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3, Pose2D
from std_msgs.msg import Float32, Bool, Empty

from movement_utils.srv import ResetOdom, ResetOdomRequest, ResetOdomResponse
from movement_utils.srv import GetPosition, GetPositionRequest, GetPositionResponse
from movement_utils.srv import GoToRelative, GoToRelativeRequest, GoToRelativeResponse

from typing import Tuple, Union

from math import asin, atan2, degrees, sqrt

from sys import version_info

if version_info.major is 2:
    raise Exception(
        "We're using function annotations here, which prevents the code from running in Python2."
    )

# set these via rosparam
SCALING_FACTOR = 1.0
LINEAR_TRAVEL_PER_STEP = 0.0
LINEAR_TRAVEL_THRESHOLD = 0.0
LINEAR_VEL = 0.0
ANGULAR_TRAVEL_PER_STEP = 0.0
ANGULAR_TRAVEL_THRESHOLD = 0.0
ANGULAR_VEL = 0.0
TWIST_CCW = Twist()
TWIST_CW = Twist()
TWIST_FWD = Twist()
START_POSITION = (Point(), 0.0)
CURRENT_ROS_POSITION = (Point(), 0.0)

HZ = 20

PUB_CMDVEL = None
PUB_ODOM_RESET = None

# handy constants
_X = 0
_Y = 1
_Z = 2
_POINT = 0
_DEG = 1


def euler_from_quaternion(q: Quaternion):
    # standard euler to quat function
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = atan2(t0, t1)

    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)

    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def pythag(start: Point, end: Point):
    d_x = start.x - end.x
    d_y = start.y - end.y
    return sqrt(d_x * d_x + d_y * d_y)


def get_position() -> Tuple[Point, float]:
    point = Point()
    point.x = (
        CURRENT_ROS_POSITION[_POINT].x - START_POSITION[_POINT].x
    ) * SCALING_FACTOR
    point.y = (
        CURRENT_ROS_POSITION[_POINT].y - START_POSITION[_POINT].y
    ) * SCALING_FACTOR
    point.z = (
        CURRENT_ROS_POSITION[_POINT].z - START_POSITION[_POINT].z
    ) * SCALING_FACTOR
    # scaling only affects linear movement, otherwise we don't scale properly
    deg = CURRENT_ROS_POSITION[_DEG]
    return (point, deg)


def reset_odom():
    global START_POSITION
    global CURRENT_ROS_POSITION

    PUB_ODOM_RESET.publish(Empty())
    START_POSITION = get_position()
    CURRENT_ROS_POSITION = (Point(), 0.0)


def update_angle(amount):
    global CURRENT_ROS_POSITION
    CURRENT_ROS_POSITION = (
        CURRENT_ROS_POSITION[_POINT],
        CURRENT_ROS_POSITION[_DEG] + amount,
    )


def handle_service_reset_odom(req: ResetOdomRequest):
    req.empty  # ignore empty request
    reset_odom()
    return ResetOdomResponse()


def handle_service_get_position(req: GetPositionRequest):
    req.empty  # ignore empty request
    position = get_position()
    reply = GetPositionResponse()
    reply.point = position[_POINT]
    reply.degrees = Float32(position[_DEG])
    return reply


def handle_service_goto_relative(req: GoToRelativeRequest):
    if PUB_CMDVEL is None:
        return GoToRelativeResponse(False)

    if req.movement.val == req.movement.STOP:
        # pub message with all 0's
        PUB_CMDVEL.publish(Twist())

    if req.movement.val == req.movement.FORWARD:
        rate = rospy.Rate(20)
        start_point = get_position()[_POINT]
        traveled_distance = 0
        while traveled_distance < abs(LINEAR_TRAVEL_PER_STEP - LINEAR_TRAVEL_THRESHOLD):
            traveled_distance = abs(pythag(start_point, get_position()[_POINT]))
            PUB_CMDVEL.publish(TWIST_FWD)
            rate.sleep()

    if req.movement.val == req.movement.CCWISE or req.movement.val == req.movement.CWISE:
        rate = rospy.Rate(HZ)
        start_deg = get_position()[_DEG]
        traveled_degs = 0
        while traveled_degs < ANGULAR_TRAVEL_PER_STEP - ANGULAR_TRAVEL_THRESHOLD:
            traveled_degs = abs(start_deg - get_position()[_DEG])
            msg = TWIST_CCW if req.movement.val == req.movement.CCWISE else TWIST_CW
            PUB_CMDVEL.publish(msg)
            # we're going to make the ok-ish assumption that the rate takes exactly the time specified.
            # it's not true, but we're doing things low-precision enough that who cares.
            if not USE_POSE2D:
                update_angle(degrees(msg.angular.z) / 2 * (1 / HZ))
            rate.sleep()

    resp = GoToRelativeResponse()
    resp.state = Bool(True)
    return resp


def handle_sub_odom(data):  # data may be a Pose2D or Odometry type
    global CURRENT_ROS_POSITION
    point = Point()
    yaw = 0
    if type(data) == Odometry:
        point.x = data.pose.pose.position.x
        point.y = data.pose.pose.position.y
        yaw = CURRENT_ROS_POSITION[_DEG]
        #yaw = degrees(euler_from_quaternion(data.pose.pose.orientation)[_Z])
    elif type(data) == Pose2D:
        point.x = data.x
        point.y = data.y
        yaw = degrees(data.theta)

    CURRENT_ROS_POSITION = (point, yaw)


def setup_parameters():
    global SCALING_FACTOR
    global LINEAR_TRAVEL_PER_STEP
    global LINEAR_TRAVEL_THRESHOLD
    global LINEAR_VEL
    global ANGULAR_TRAVEL_PER_STEP
    global ANGULAR_TRAVEL_THRESHOLD
    global ANGULAR_VEL
    global TWIST_FWD
    global TWIST_CCW
    global TWIST_CW
    global USE_POSE2D

    SCALING_FACTOR = rospy.get_param("movement_utils/scaling_factor", 1)
    LINEAR_TRAVEL_PER_STEP = rospy.get_param(
        "movement_utils/linear_travel_per_step", 0.25
    )
    LINEAR_TRAVEL_THRESHOLD = rospy.get_param(
        "movement_utils/linear_travel_threshold", 0.01
    )
    LINEAR_VEL = rospy.get_param("movement_utils/linear_vel", 0.2)
    ANGULAR_TRAVEL_PER_STEP = rospy.get_param(
        "movement_utils/angular_travel_per_step", 20
    )
    ANGULAR_TRAVEL_THRESHOLD = rospy.get_param(
        "movement_utils/angular_travel_threshold", 2
    )
    ANGULAR_VEL = rospy.get_param("movement_utils/angular_vel", 0.35)

    TWIST_FWD = Twist(Vector3(LINEAR_VEL, 0, 0), Vector3(0, 0, 0))
    TWIST_CCW = Twist(Vector3(0, 0, 0), Vector3(0, 0, ANGULAR_VEL))
    TWIST_CW = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ANGULAR_VEL))

    USE_POSE2D = rospy.get_param("movement_utils/use_pose2d", False)


def movement_wrapper_node():
    rospy.init_node("movement_wrapper")
    setup_parameters()

    service_reset_odom = rospy.Service(
        "/movement_wrapper/reset_odom", ResetOdom, handle_service_reset_odom
    )
    service_get_position = rospy.Service(
        "/movement_wrapper/get_position", GetPosition, handle_service_get_position
    )
    service_goto_position = rospy.Service(
        "/movement_wrapper/goto_relative", GoToRelative, handle_service_goto_relative
    )

    if USE_POSE2D:
        sub_current_position = rospy.Subscriber(
            "pose2D", Pose2D, handle_sub_odom, queue_size=1
        )
    else:
        sub_current_position = rospy.Subscriber(
            "odom", Odometry, handle_sub_odom, queue_size=1
        )

    global PUB_CMDVEL
    PUB_CMDVEL = rospy.Publisher("cmd_vel", Twist, queue_size=1)
    global PUB_ODOM_RESET
    PUB_ODOM_RESET = rospy.Publisher("reset_odometry", Empty, queue_size=1)

    # zero the node while we're starting up
    handle_service_reset_odom(ResetOdomRequest())
    rospy.loginfo("movement_wrapper node ready to go!")
    rospy.spin()

    rospy.loginfo("Told to shut down, closing subs/pubs/services.")
    service_reset_odom.shutdown()
    service_get_position.shutdown()
    service_goto_position.shutdown()
    PUB_CMDVEL.unregister()
    sub_current_position.unregister()
    rospy.loginfo("Goodbye!")


if __name__ == "__main__":
    movement_wrapper_node()
