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

namespace gm = geometry_msgs;
namespace stdm = std_msgs;
namespace nav = nav_msgs;


const int max_x = 2;
const int max_y = 2;
const int max_r = 2;


float get_time(){
    ros::Time time = ros::Time::now();
    return (float)time.toSec();
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

struct FetchData {
    tf2::Quaternion q;
    gm::Point p;
    gm::Vector3 linear;
    gm::Vector3 angular;
    


    tf2::Quaternion get_q() {return q;}
    gm::Point get_p(){return p;}
    gm::Vector3 get_linear(){return linear;}
    gm::Vector3 get_angular(){return angular;}

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

};

struct Point{
    float x, y, theta;
    // Pid x_pid, y_pid, theta_pid;

    void FetchPoint(const gm::Pose2D::ConstPtr& msg){
        x = msg->x;
        y = msg->y;
        theta = msg->theta;
    }
};

struct PidParam{

    float kp, ki, kd;

    void FetchParam(const gm::Vector3::ConstPtr& msg){
        kp = msg->x;
        ki = msg->y;
        kd = msg->z;
    }
} param;

struct Pid { 
private:
    float accum = 0;
    float prev_time_i = 0;
    float prev_time_d = 0;
    float prev_point = 0;

    void integrate(const float current){
        float curr_time = get_time();
        this->accum += current*(curr_time - prev_time_i);
        prev_time_i = curr_time;
    }
    float differentiate(const float current,const float last){
        float curr_time = get_time();
        float d = (float)((current - last)/(curr_time - prev_time_d));
        prev_time_d = curr_time;
        return d;
    }
public:
 


};

void publish_output(gm::Twist msg, ros::Publisher pub){
    pub.publish(msg);
}

int main(int argc, char **argv) {
    FetchData data;
    Point setPoint;
    // Pid 
    //x
    //y
    //theta

    
    
    ros::init(argc, argv, "PID");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom", 1000, &FetchData::fetch_data, &data);
    ros::Subscriber sub2 = nh.subscribe("/setpoint", 100, &Point::FetchPoint, &setPoint);
    ros::Subscriber sub3 = nh.subscribe("/set_param", 100, &PidParam::FetchParam, &param);
    ros::Publisher pub = nh.advertise<gm::Twist>("/cmd_vel", 1000);

    ros::Rate loop_rate(800);

    while (ros::ok()) {
        publish_output(AxisRot(f.get()), pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
}