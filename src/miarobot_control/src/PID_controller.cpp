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

const int max_x = 2;
const int max_y = 2;
const int max_r = 2;

float get_time(){
    ros::Time time = ros::Time::now();
    return time.toSec();
}

struct AxisRot {
    double yaw =0, pitch=0, roll=0;
    AxisRot() {};
    AxisRot(double yaw, double pitch, double roll): yaw(yaw), pitch(pitch), roll(roll) {}
    AxisRot(tf2::Quaternion q) {
        tf2::Matrix3x3 matrix(q);
        matrix.getRPY(roll, pitch, yaw);
    }
};

class FetchData {
    tf2::Quaternion q;
    geometry_msgs::Point p;
    geometry_msgs::Vector3 linear;
    geometry_msgs::Vector3 angular;

public:
    tf2::Quaternion get_q() {return q;}
    geometry_msgs::Point get_p(){return p;}
    geometry_msgs::Vector3 get_linear(){return linear;}
    geometry_msgs::Vector3 get_angular(){return angular;}

    void fetch_data(const nav_msgs::Odometry::ConstPtr& msg) {
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

};

struct SetPoint{
    float set_x, set_y, set_theta;
    void FetchSetPoint(const geometry_msgs::Pose2D::ConstPtr& msg){
        set_x = msg->x;
        set_y = msg->y;
        set_theta = msg->theta;
    }
};

struct PID_param {
    float kp, ki, kd;
    void FetchParam(const geometry_msgs::Vector3::ConstPtr& msg){
        kp = msg->x;
        ki = msg->y;
        kd = msg->z;
    }
};

struct PID{};

int main(int argc, char **argv) {
    FetchData data;
    SetPoint setPoint;
    PID_param param;

    //x
    //y
    //theta
    
    
    
    ros::init(argc, argv, "PID");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom", 1000, &FetchData::fetch_data, &data);
    ros::Subscriber sub2 = nh.subscribe("/setpoint", 100, &SetPoint::FetchSetPoint, &setPoint);
    ros::Subscriber sub3 = nh.subscribe("/set_param", 100, &PID_param::FetchParam, &param);
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/cmd_vel", 1000);

    ros::Rate loop_rate(800);

    while (ros::ok()) {
        // publish_rot(AxisRot(fq.get()), pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
}