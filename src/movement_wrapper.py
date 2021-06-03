#!/usr/bin/env python3

import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Vector3
from std_msgs.msg import Float32

from movement_utils.srv import ResetOdom, ResetOdomRequest, ResetOdomResponse
from movement_utils.srv import GetPosition, GetPositionRequest, GetPositionResponse
from movement_utils.srv import GoToRelative, GoToRelativeRequest, GoToRelativeResponse

from typing import Tuple

from math import asin, atan2, degrees, sqrt

# set these via rosparam
SCALING_FACTOR:int           = 1
LINEAR_TRAVEL_PER_STEP:int   = 0
LINEAR_TRAVEL_THRESHOLD:int  = 0
LINEAR_VEL:int               = 0
ANGULAR_TRAVEL_PER_STEP:int  = 0
ANGULAR_TRAVEL_THRESHOLD:int = 0
ANGULAR_VEL:int              = 0
TWIST_CCW:Twist              = Twist()
TWIST_CW:Twist               = Twist()
TWIST_FWD:Twist              = Twist()

# handy constants
_X=0
_Y=1
_Z=2
_POINT=0
_DEG=1

START_POSITION:Tuple[Point, Float32] = (Point(), Float32())
CURRENT_ROS_POSITION:Tuple[Point, Float32] = (Point(), Float32())
PUB_CMDVEL = None


def euler_from_quaternion(q:Quaternion):
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


def pythag(start:Point, end:Point):
    d_x = start.x - end.x
    d_y = start.y - end.y
    return sqrt(d_x*d_x + d_y*d_y)

def get_position():
    point = Point()
    point.x = (CURRENT_ROS_POSITION[_POINT].x - START_POSITION[_POINT].x) * SCALING_FACTOR
    point.y = (CURRENT_ROS_POSITION[_POINT].y - START_POSITION[_POINT].y) * SCALING_FACTOR
    point.z = (CURRENT_ROS_POSITION[_POINT].z - START_POSITION[_POINT].z) * SCALING_FACTOR
    # scaling only affects linear movement, otherwise we don't scale properly
    deg = CURRENT_ROS_POSITION[_DEG].data - START_POSITION[_DEG].data
    return (point, Float32(deg))


def handle_service_reset_odom(req:ResetOdomRequest):
    req.empty # ignore empty request
    global START_POSITION
    START_POSITION = get_position()
    return ResetOdomResponse()


def handle_service_get_position(req:GetPositionRequest):
    req.empty # ignore empty request
    position = get_position()
    reply = GetPositionResponse()
    reply.point = position[_POINT]
    reply.degrees = Float32(position[_DEG])
    return reply


def handle_service_goto_relative(req:GoToRelativeRequest):
    if PUB_CMDVEL is None:
        return GoToRelativeRequest(False)

    if req.movement == req.STOP:
        # pub message with all 0's
        PUB_CMDVEL.publish(Twist())

    if req.movement == req.FORWARD:
        rate = rospy.Rate(20)
        start_point = get_position()[_POINT]
        traveled_distance = 0
        while traveled_distance < LINEAR_TRAVEL_PER_STEP - LINEAR_TRAVEL_THRESHOLD:
            traveled_distance = pythag(start_point, get_position()[_POINT])
            PUB_CMDVEL.publish(TWIST_FWD)
            rate.sleep()

    if req.movement == req.CCWISE or req.movement == req.CWISE:
        rate = rospy.Rate(20)
        start_deg = get_position()[_DEG]
        traveled_degs = 0
        while traveled_degs < ANGULAR_TRAVEL_PER_STEP - ANGULAR_TRAVEL_THRESHOLD:
            traveled_degs = abs(start_deg.data - get_position()[_DEG].data)
            msg = TWIST_CCW if req.movement == req.CCWISE else TWIST_CW
            PUB_CMDVEL.publish(msg)
            rate.sleep()

    return GoToRelativeResponse(True)


def handle_sub_odom(data:Odometry):
    point = Point()
    point.x = data.pose.pose.position.x
    point.y = data.pose.pose.position.y
    point.z = data.pose.pose.position.z

    yaw = degrees(euler_from_quaternion(data.pose.pose.orientation)[_Z])

    global CURRENT_ROS_POSITION
    CURRENT_ROS_POSITION = (point, Float32(yaw))


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

    SCALING_FACTOR           = rospy.get_param("movement_utils/scaling_factor", 1)
    LINEAR_TRAVEL_PER_STEP   = rospy.get_param("movement_utils/linear_travel_per_step", 1)
    LINEAR_TRAVEL_THRESHOLD  = rospy.get_param("movement_utils/linear_travel_threshold", 0.01)
    LINEAR_VEL               = rospy.get_param("movement_utils/linear_vel", 0.2)
    ANGULAR_TRAVEL_PER_STEP  = rospy.get_param("movement_utils/angular_travel_per_step", 20)
    ANGULAR_TRAVEL_THRESHOLD = rospy.get_param("movement_utils/angular_travel_threshold", 2)
    ANGULAR_VEL              = rospy.get_param("movement_utils/angular_vel", 0.35)

    TWIST_FWD = Twist(Vector3(LINEAR_VEL, 0, 0), Vector3(0, 0, 0))
    TWIST_CCW = Twist(Vector3(0, 0, 0), Vector3(0, 0, ANGULAR_VEL))
    TWIST_CW = Twist(Vector3(0, 0, 0), Vector3(0, 0, -ANGULAR_VEL))

def movement_wrapper_node():
    rospy.init_node('movement_wrapper')
    setup_parameters()

    service_reset_odom    = rospy.Service('reset_odom',    ResetOdom,    handle_service_reset_odom)
    service_get_position  = rospy.Service('get_position',  GetPosition,  handle_service_get_position)
    service_goto_position = rospy.Service('goto_relative', GoToRelative, handle_service_goto_relative)

    sub_current_position  = rospy.Subscriber('odom',       Odometry,     handle_sub_odom, queue_size=1)
    global PUB_CMDVEL
    PUB_CMDVEL            = rospy.Publisher('cmd_vel',     Twist, queue_size=1)

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
