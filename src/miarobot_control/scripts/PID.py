
import rospy
from geometry_msgs.msg import Pose, Point, Twist, Vector3, Pose2D
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

MAX_SPEED = 2

class FetchData:
    def __init__(self):
        self.q = (0, 0, 0, 1)  # Quaternion (x, y, z, w)
        self.p = Point()
        self.linear = Vector3()
        self.angular = Vector3()

    def fetch_data(self, msg):
        self.q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.p = msg.pose.pose.position
        self.linear = msg.twist.twist.linear
        self.angular = msg.twist.twist.angular

class PointData:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

    def fetch_point(self, msg):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

class PidParam:
    def __init__(self):
        self.kp = 0
        self.ki = 0
        self.kd = 0

    def fetch_param(self, msg):
        self.kp = msg.x
        self.ki = msg.y
        self.kd = msg.z

class Pid:
    def __init__(self):
        self.accum = 0
        self.prev_time_i = rospy.get_time()
        self.prev_time_d = rospy.get_time()
        self.prev_point = 0
        self.prev_output = 0
        self.prev_error = 0

    def integrate(self, current):
        curr_time = rospy.get_time()
        delta_time = curr_time - self.prev_time_i
        if delta_time != 0:  
            self.accum += current * delta_time
        self.prev_time_i = curr_time

    def differentiate(self, last_point):
        curr_time = rospy.get_time()
        delta_time = curr_time - self.prev_time_d
        d = (self.prev_point - last_point) / delta_time if delta_time > 0 else 0
        self.prev_time_d = curr_time
        self.prev_point = last_point
        return d

    def check_clamping(self):
        if self.prev_error != 0 and self.prev_output != 0:
            if self.prev_output > MAX_SPEED and self.prev_error / self.prev_output > 0:
                return True
        return False

    def get_output(self, target, current_point):
        error = target - current_point
        p = param.kp * error
        d = param.kd * self.differentiate(error)
        self.integrate(error)
        self.prev_error = error
        
        if self.check_clamping():
            self.prev_output = p + d
            return self.prev_output
        else:
            self.prev_output = p + d + param.ki * self.accum
            return min(MAX_SPEED, self.prev_output)

def publish_output(msg, pub):
    pub.publish(msg)

def main():
    rospy.init_node('PID')

    global param
    data = FetchData()
    set_point = PointData()
    param = PidParam()

    rospy.Subscriber('/odom', Odometry, data.fetch_data)
    rospy.Subscriber('/setpoint', Pose2D, set_point.fetch_point)
    rospy.Subscriber('/set_param', Vector3, param.fetch_param)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1000)

    rate = rospy.Rate(800)

    x_pid = Pid()
    y_pid = Pid()
    theta_pid = Pid()

    while not rospy.is_shutdown():
        msg = Twist()
        rot_rpy = euler_from_quaternion(data.q)
        x_out = x_pid.get_output(set_point.x, data.linear.x)
        y_out = y_pid.get_output(set_point.y, data.linear.y)
        theta_out = theta_pid.get_output(set_point.theta, rot_rpy[2])

        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = theta_out
        msg.linear.x = x_out
        msg.linear.y = y_out
        msg.linear.z = 0

        publish_output(msg, pub)

        rospy.spin_once()
        rate.sleep()

if __name__ == '__main__':
    main()
