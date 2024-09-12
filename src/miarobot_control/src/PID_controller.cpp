#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>
#include <algorithm> 

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define clamp(v, l, h) min(max(v, l), h)

namespace gm = geometry_msgs;
namespace stdm = std_msgs;
namespace nav = nav_msgs;
namespace chrono = std::chrono;

const float MAX_SPEED = 2;

enum Clamp { NO_CLAMP, CLAMP };

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;


struct AxisRot {
    double yaw = 0, pitch = 0, roll = 0;
    AxisRot() {};
    AxisRot(double yaw, double pitch, double roll) : yaw(yaw), pitch(pitch), roll(roll) {}
    AxisRot(tf2::Quaternion q) {
        tf2::Matrix3x3 matrix(q);
        matrix.getRPY(roll, pitch, yaw);
    }
};




struct FetchData {
    float set_x, set_y, set_theta;
    tf2::Quaternion q;
    gm::Point p;
    gm::Vector3 linear;
    gm::Vector3 angular;

    void fetch_data(const nav::Odometry::ConstPtr& msg) {
        q.setX(msg->pose.pose.orientation.x);
        q.setY(msg->pose.pose.orientation.y);
        q.setZ(msg->pose.pose.orientation.z);
        q.setW(msg->pose.pose.orientation.w);

        p.x = msg->pose.pose.position.x;
        p.y = msg->pose.pose.position.y;
        p.z = msg->pose.pose.position.z;

        linear.x = msg->twist.twist.linear.x;
        linear.y = msg->twist.twist.linear.y;
        linear.z = msg->twist.twist.linear.z;

        angular.x = msg->twist.twist.angular.x;
        angular.y = msg->twist.twist.angular.y;
        angular.z = msg->twist.twist.angular.z;


    }
    void FetchPoint(const gm::Pose2D::ConstPtr& msg){
        set_x = msg->x;
        set_y = msg->y;
        set_theta = msg->theta;
    }
};




struct Pid {
private:
    float kp, ki, kd;
    float accum;
    float prev_err;
    TimePoint prev_time;

public:

    // void fetch(const gm::Vector3::ConstPtr& msg) {
    //     kp = msg->x;
    //     ki = msg->y;
    //     kd = msg->z;
    // }
    Pid(float kp, float ki, float kd): kp(kp), ki(ki), kd(kd) {}

    float get_output(const float target, const float current) {
        float err = target - current;
        auto dt = (float)(Clock::now() - prev_time).count();

        float p = kp * err;
        float d = kd * (prev_err - err) / dt;
        accum += err * dt;
        accum = clamp(accum, -MAX_SPEED, MAX_SPEED);

        float i = ki * accum;
       
        prev_err = err;
        return p + d + i;
    }
};

void publish_output(const gm::Twist& msg, ros::Publisher& pub) {
    pub.publish(msg);
}

int main(int argc, char **argv) {
    FetchData data;
    Pid x_pid(0.5,0,0.01);
    Pid y_pid(0.5,0,0.01);
    Pid theta_pid(0.5,0,0.01);

    ros::init(argc, argv, "PID");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom", 1000, &FetchData::fetch_data, &data);
    ros::Subscriber sub2 = nh.subscribe("/setpoint", 100, &FetchData::FetchPoint, &data);
    // ros::Subscriber sub3 = nh.subscribe("/set_param", 100, &Pid::fetch, &x_pid);
    ros::Publisher pub = nh.advertise<gm::Twist>("/cmd_vel", 1000);

    ros::Rate loop_rate(50);

    while (ros::ok()) {
        gm::Twist msg;
        auto rot_rpy = AxisRot(data.q);
        auto x_out = min(MAX_SPEED, x_pid.get_output(data.set_x, data.linear.x));
        auto y_out = min(MAX_SPEED, y_pid.get_output(data.set_y, data.linear.y));
        auto theta_out = min(MAX_SPEED, theta_pid.get_output(data.set_theta, rot_rpy.yaw));

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = theta_out;
        msg.linear.x = x_out;
        msg.linear.y = y_out;
        msg.linear.z = 0.0;

        publish_output(msg, pub);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
