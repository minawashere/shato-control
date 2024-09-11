#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray  # Assuming target position is sent as an array
import tf.transformations
import math

class Robot(object):
    def __init__(self):
        rospy.init_node("motion_planner", anonymous=True)

        # Publishers
        self.velocity_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Subscribers
        self.odom_subscriber = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        
        self.target_subscriber = rospy.Subscriber("/target_position", Float64MultiArray, self.target_callback)

        # Initialization
        self.current_position = [0, 0, 0]  # x, y, theta
        self.final_position = [2.59, 0.7, 0]  # Default target position if not yet updated
        self.wheel_radius = 0.033  # meters
        self.robot_radius = 0.160  # meters

    def odom_callback(self, data):
        # Extract position
        self.current_position[0] = data.pose.pose.position.x   # to get the position
        self.current_position[1] = data.pose.pose.position.y

        # Extract quaternion
               #quaternion is w as scalar value and x and y and z vectors
                # to get the direction of robot as in directional as x and y and z and w as scalar part
        quaternion = [
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        ]

        # this to make quaternion instead of eular that use [roll , pitch , Yaw]
        
        # Convert quaternion to Euler angles
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.current_position[2] = euler[2]  # Yaw angle

    def target_callback(self, data):
        # Update final_position from received message
        if len(data.data) == 3:
            self.final_position = data.data
        else:
            rospy.logwarn("Received incorrect data length for target position")

    def set_velocities(self, linear_velocity, angular_velocity):
        velocity_message = Twist()
        velocity_message.linear.x = linear_velocity
        velocity_message.angular.z = angular_velocity
        self.velocity_publisher.publish(velocity_message)

    def feedback_control(self):
        # Current position x, y, theta
        current_x, current_y, current_theta = self.current_position

        # Target position
        target_x, target_y, target_theta = self.final_position

        # Calculate angle to the target
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        angle_difference = angle_to_target - current_theta

        # Normalize the angle difference
        angle_difference = (angle_difference + math.pi) % (2 * math.pi) - math.pi

        # Calculate the distance to the target
        distance_to_target = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

        # Set control gains
        angular_gain = 1.0
        linear_gain = 0.5

        # Maximum velocities
        max_linear_velocity = 0.22
        max_angular_velocity = 2.84

        # Initialize velocities
        linear_velocity = 0
        angular_velocity = 0

        # Rotate first to align with the target
        if abs(angle_difference) > 0.05:
            angular_velocity = angular_gain * angle_difference
            angular_velocity = max(min(angular_velocity, max_angular_velocity), -max_angular_velocity)
        # Move forward after alignment
        elif distance_to_target > 0.1:
            linear_velocity = linear_gain * distance_to_target
            linear_velocity = max(min(linear_velocity, max_linear_velocity), 0)
        else:
            # Stop the robot when goal is reached
            linear_velocity = 0
            angular_velocity = 0
            rospy.loginfo("Goal Reached Wohooo")
        
        # Publish velocities
        self.set_velocities(linear_velocity, angular_velocity)

        # Stop the robot if it's close enough to the target
        if distance_to_target <= 0.1 and abs(angle_difference) <= 0.05:
            self.set_velocities(0, 0)
            rospy.signal_shutdown("Goal Reached")

if __name__ == '__main__':
    try:
        rospy.wait_for_service("/gazebo/reset_simulation")
        robot_instance = Robot()

        while not rospy.is_shutdown():
            robot_instance.feedback_control()
            print("Current pos:", robot_instance.current_position)
            rospy.sleep(0.1)

    except rospy.ROSInterruptException:
        rospy.loginfo("We have Arrived")
        rospy.signal_shutdown("It's Done")