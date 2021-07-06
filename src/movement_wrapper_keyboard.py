#!/usr/bin/env python3
from enum import Enum
import rospy
import termios, select, sys, tty

from nav_msgs.msg import Odometry
from movement_utils.srv import GoToRelative, GoToRelativeRequest
from movement_utils.srv import LinearVel, LinearVelRequest
GOTO_RELATIVE_CLIENT
LINEAR_VEL_CLIENT
ODOM_SUB
CURRENT_ODOM
USAGE = """
Make and send service calls to the movement wrapper utility.
Q/A: Increase/Decrease linear speed
W/S: Increase/Decrease angular speed
E/D: Increase/Decrease linear travel
R/F: Increase/Decrease angular travel

Z:   Send GoToRelative of 0
X:   Send LinearVel of 0
J:   Send GoToRelative request based off of current settings
K:   Send LinearVel request based off of current settings
U:   Send GoToRelative request based off of default parameters
I:   Send LinearVel request based off of default parameters
"""
RUNNING_CMD = "Ready"

class validRequests(Enum):
    GoToRelativeZero=0
    LinearVelZero=1
    GoToRelativeCustom=2
    LinearVelCustom=3
    GoToRelativeDefault=4
    LinearVelDefault=5

messageSendBindings = {
        'z':validRequests.GoToRelativeZero,
        'x':validRequests.LinearVelZero,
        'j':validRequests.GoToRelativeCustom,
        'k':validRequests.LinearVelCustom,
        'u':validRequests.GoToRelativeDefault,
        'i':validRequests.LinearVelDefault,
    }

messageModLegends ={
        'q':"increase linear speed",
        'a':"decrease linear speed",
        'w':"increase angular speed",
        's':"decrease angular speed",
        'e':"increase linear travel",
        'd':"decrease linear travel",
        'r':"increase angular travel",
        'f':"decrease angular travel",
    }

messageModBindings={
        'q': (1.1 , 0   , 0   , 0   , 0   , 0   , 0   , 0 )   ,
        'a': (0   , 0.9 , 0   , 0   , 0   , 0   , 0   , 0 )   ,
        'w': (0   , 0   , 1.1 , 0   , 0   , 0   , 0   , 0 )   ,
        's': (0   , 0   , 0   , 0.9 , 0   , 0   , 0   , 0 )   ,
        'e': (0   , 0   , 0   , 0   , 1.1 , 0   , 0   , 0 )   ,
        'd': (0   , 0   , 0   , 0   , 0   , 0.9 , 0   , 0 )   ,
        'r': (0   , 0   , 0   , 0   , 0   , 0   , 1.1 , 0 )   ,
        'f': (0   , 0   , 0   , 0   , 0   , 0   , 0   , 0.9 ) ,
    }

def getKey(key_timeout):
    """ stolen straight from teleop_twist_keyboard """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def odom_callback(msg: Odometry):
    CURRENT_ODOM = msg

def main():
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None
    global GOTO_RELATIVE_CLIENT
    GOTO_RELATIVE_CLIENT = rospy.ServiceProxy(
        "movement_wrapper/goto_relative", GoToRelative
    )
    global LINEAR_VEL_CLIENT
    LINEAR_VEL_CLIENT = rospy.ServiceProxy(
        "movement_wrapper/linear_vel", LinearVel
    )
    global ODOM_SUB
    global CURRENT_ODOM
    ODOM_SUB = rospy.Subscriber("odom", Odometry, odom_callback)
    CURRENT_ODOM = Odometry()
    try:
        print(USAGE)
        print(RUNNING_CMD)
        while not rospy.is_shutdown():
            key = getKey(key_timeout)
            print(key)
            if key.lower() in messageSendBindings.keys():

                pass
            elif key.lower() in messageModBindings.keys():
                pass

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


    pass


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    main()
