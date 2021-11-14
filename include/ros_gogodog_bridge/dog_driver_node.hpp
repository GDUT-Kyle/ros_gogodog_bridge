#ifndef DOG_DRIVER_NODE_H
#define DOG_DRIVER_NODE_H

#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <string>
#include <mutex>
#include <tf/transform_broadcaster.h>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "CppLinuxSerial/SerialPort.hpp"

using namespace mn::CppLinuxSerial;

namespace dog_driver
{
#define CONTROL_VEL std::string("c")
#define READ_VEL    std::string("v")

class DogDriverNode
{
protected:
    ros::NodeHandle n_;

    ros::Publisher pub_odom;
    ros::Subscriber sub_vel;
    ros::Subscriber sub_reset;

    SerialPort serialPort;
    std::string dog_device_;
    int baud_;
    int framerate_;
    bool isVerif_;

    std::mutex mtx;

    std::vector<double> recVelocity;
    std::vector<double> lastRecVelocity;
    Eigen::Matrix<double, 6, 1> dMotion;

    Eigen::Vector3d curPosition;
    Eigen::Quaterniond curPose;
    double lastTime = 0.0;
    double curTime = 0.0;
    double dt = 0.0;
    bool initTime = false;

    double lastMovingTime = 0.0;
    bool isMoving = false;

    nav_msgs::Odometry odomMsg;

public:
    DogDriverNode();
    virtual ~DogDriverNode();
    void config();
    void connect();
    void disconnect();
    void stop();
    void spin();
    void checkPort();
    void parseOdometry();
    void PublishOdometryToROS();
    void PublishTF();
    void setVelocity(double vX, double vY, double vZ, double vYaw, double vPitch, double vRoll);
    void cmdVelHandler(const geometry_msgs::Twist::ConstPtr cmdVel);
    void ResetOdomIntegratorCallback(const std_msgs::Bool::ConstPtr& msg);
};
} // namespace name


#endif