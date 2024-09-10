#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

# Global variables for wheel parameters
WHEEL_SEPARATION_WIDTH = None
WHEEL_SEPARATION_LENGTH = None
WHEEL_GEOMETRY = None
WHEEL_RADIUS = None

# Publishers for motor speeds
pub_mfl = None
pub_mfr = None
pub_mbl = None
pub_mbr = None

# Global variables for position
global_position = (0.0, 0.0, 0.0)  # (x, y, theta)

def odometry_callback(odom_msg):
    global global_position
    position = odom_msg.pose.pose.position
    orientation = odom_msg.pose.pose.orientation

    # Extracting local position (x, y)
    x = position.x
    y = position.y

    # Extracting theta from quaternion
    _, _, theta = quaternion_to_euler(orientation)

    global_position = (x, y, theta)

def quaternion_to_euler(orientation):
    """Convert quaternion to Euler angles."""
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    # Roll (x-axis rotation)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    # Pitch (y-axis rotation)
    pitch = math.asin(2.0 * (w * y - z * x))
    # Yaw (z-axis rotation)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    return roll, pitch, yaw

def convert(move):
    x = move.linear.x
    y = move.linear.y
    rot = move.angular.z

    # Calculate wheel speeds
    front_left = (x - y - rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    front_right = (x + y + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_left = (x + y - rot * WHEEL_GEOMETRY) / WHEEL_RADIUS
    back_right = (x - y + rot * WHEEL_GEOMETRY) / WHEEL_RADIUS

    # Publish wheel speeds
    pub_mfl.publish(front_left)
    pub_mfr.publish(front_right)
    pub_mbl.publish(back_left)
    pub_mbr.publish(back_right)

if __name__ == '__main__':
    try:
        rospy.init_node('mecanum_controller', anonymous=True)

        # Get parameters related to wheel geometry
        WHEEL_SEPARATION_WIDTH = rospy.get_param("/wheel/separation/horizontal", 0.3)  
        WHEEL_SEPARATION_LENGTH = rospy.get_param("/wheel/separation/vertical", 0.3)  
        WHEEL_GEOMETRY = (WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH) / 2
        WHEEL_RADIUS = rospy.get_param("/wheel/diameter", 0.04) / 2  

        # Initialize publishers for each motor
        pub_mfl = rospy.Publisher('motor/front/left', Float32, queue_size=1)
        pub_mfr = rospy.Publisher('motor/front/right', Float32, queue_size=1)
        pub_mbl = rospy.Publisher('motor/rear/left', Float32, queue_size=1)
        pub_mbr = rospy.Publisher('motor/rear/right', Float32, queue_size=1)

        # Subscribe to cmd_vel topic
        sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, convert)

        # Subscribe to odometry topic
        sub_odom = rospy.Subscriber('odom', Odometry, odometry_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass