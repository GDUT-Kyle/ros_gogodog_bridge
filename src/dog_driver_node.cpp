#include "CppLinuxSerial/SerialPort.hpp"
#include "ros_gogodog_bridge/dog_driver_node.hpp"
#include "ros_gogodog_bridge/util.h"

using namespace mn::CppLinuxSerial;
using namespace dog_driver;

DogDriverNode::DogDriverNode() : n_("~")
{
	n_.param("dog_device", dog_device_, std::string("/dev/ttyUSB0"));
	n_.param("baud", baud_, 115200);
	n_.param("framerate", framerate_, 100);
	n_.param("isVerif", isVerif_, false);

	recVelocity.resize(6);
	lastRecVelocity.resize(6);
	dMotion.resize(6);
	curPosition.setZero();
	curPose.setIdentity();

	pub_odom = n_.advertise<nav_msgs::Odometry>("/dog/odometry", 5);
	// sub_vel = n_.subscribe<geometry_msgs::Twist>("/cmd_vel", 5, &DogDriverNode::cmdVelHandler, this);
	sub_vel = 
			n_.subscribe<geometry_msgs::Twist>("/cmd_vel", 5, boost::bind(&DogDriverNode::cmdVelHandler, this, _1));
	sub_reset =
			n_.subscribe<std_msgs::Bool>("/dog/reset", 1, boost::bind(&DogDriverNode::ResetOdomIntegratorCallback, this, _1));

	connect();
}

DogDriverNode::~DogDriverNode()
{
	disconnect();
}

void DogDriverNode::connect()
{
	serialPort.SetDevice(dog_device_);
	serialPort.SetBaudRate((unsigned int)baud_);
	serialPort.SetTimeout(0.1);
	serialPort.Open();

	setVelocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	lastMovingTime = ros::Time::now().toSec();
	isMoving = false;

	// check conection parameter
	ROS_INFO("Starting \033[1;32;40m'%s'\033[0m at \033[1;32;40m%u\033[0m, Verif: %d", dog_device_.c_str(), (unsigned int)baud_, isVerif_);
}

void DogDriverNode::disconnect()
{
	stop();
	serialPort.Close();
}

void DogDriverNode::stop()
{
	setVelocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
}

void DogDriverNode::checkPort()
{
	// mtx.lock();
	std::string readData;
	// serialPort.Read(readData);
	serialPort.ReadLine(readData);
	// unsigned char chardata = 0;
	// try
    // {
    //     // Read a single byte of data from the serial ports.
    //     serialPort.ReadByte(chardata, 25) ;
    // }
    // catch (const std::system_error&)
    // {
    //     std::cerr << "Read was unsuccessful." << std::endl ;
    // }
	// if(chardata) std::cout<<chardata<<std::endl;
	if(readData.empty()) return;
	std::vector<std::string> vecData;
	ROS_INFO("Read data from serial :\n %s", readData.c_str());
	ROS_INFO("The data has [%d] byte", readData.size());
	split(readData, vecData, ' ');
	if(vecData[0] == READ_VEL)
	{
		if(vecData.size() != 7){
			ROS_WARN("Serial communi occur error!");
			return;
		}
		for(size_t i=0; i<6; i++)
		{
			try
			{
				recVelocity[i] = std::stod(vecData[i+1]);
			}
			catch (const std::invalid_argument& ia) {
	  			std::cerr << "Invalid argument: " << ia.what() << "src/ros_gogodog_bridge/src/dog_driver_node.cpp--line 82" << '\n';
				return;
			}
			catch (const std::out_of_range& oor) {
				std::cerr << "Out of Range error: " << oor.what() << "src/ros_gogodog_bridge/src/dog_driver_node.cpp--line 82" << '\n';
				return;
			}
		}
		parseOdometry();
		ROS_INFO("Receive velocity value: (%f, %f, %f, %f, %f, %f)", recVelocity[0], recVelocity[1], recVelocity[2], recVelocity[3],recVelocity[4],recVelocity[5]);
	}
	else
	{
		ROS_WARN("Serial communi occur error!");
		return;
	}
	if(isMoving)
	{
		if(ros::Time::now().toSec()-lastMovingTime>0.2) // timeout
		{
			setVelocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
			isMoving = false;
		}
	}
	// mtx.unlock();
}

void DogDriverNode::parseOdometry()
{
	if(!initTime)
	{
		initTime = true;
		curTime = ros::Time::now().toSec();
		lastTime = curTime;
		for(size_t i=0; i<recVelocity.size(); i++)
			lastRecVelocity[i] = recVelocity[i];
		return;
	}

	curTime = ros::Time::now().toSec();
	dt = curTime - lastTime;
	// 中值积分
	for(size_t i=0; i<recVelocity.size(); i++)
	{
		dMotion(i, 0) = 0.5*(lastRecVelocity[i]+recVelocity[i])*dt;
	}

	Eigen::AngleAxisd dYaw(dMotion(3,0),Eigen::Vector3d(0, 0, 1));
	Eigen::AngleAxisd dPitch(dMotion(4,0),Eigen::Vector3d(0, 1, 0));
	Eigen::AngleAxisd dRoll(dMotion(5,0),Eigen::Vector3d(1, 0, 0));
	Eigen::Quaterniond dPose = dYaw * dPitch * dRoll;

	Eigen::Vector3d dPosition = dMotion.block(0, 0, 3, 1);

	curPose = curPose * dPose;
	curPosition += curPose * dPosition;

	// std::cout<<"curPosition: "<<curPosition.transpose()<<std::endl;
	// std::cout<<"curPose: "<<curPose.coeffs().transpose()<<std::endl;

	PublishOdometryToROS();
	PublishTF();

	lastTime = curTime;
}

void DogDriverNode::PublishOdometryToROS()
{
	odomMsg.header.stamp = ros::Time().fromSec(curTime);
	odomMsg.header.frame_id = "odom";
	odomMsg.child_frame_id = "dog_base";
	odomMsg.pose.pose.position.x = curPosition.x();
	odomMsg.pose.pose.position.y = curPosition.y();
	odomMsg.pose.pose.position.z = curPosition.z();
	odomMsg.pose.pose.orientation.w = curPose.w();
	odomMsg.pose.pose.orientation.x = curPose.x();
	odomMsg.pose.pose.orientation.y = curPose.y();
	odomMsg.pose.pose.orientation.z = curPose.z();
	pub_odom.publish(odomMsg);
}

void DogDriverNode::PublishTF()
{
	static tf::TransformBroadcaster br;
  	tf::Transform transform;
	transform.setOrigin( tf::Vector3(curPosition.x(), curPosition.y(), curPosition.z()) );
	tf::Quaternion q(curPose.x(), curPose.y(), curPose.z(), curPose.w());
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time().fromSec(curTime), 
						odomMsg.header.frame_id, odomMsg.child_frame_id));
}

void DogDriverNode::setVelocity(double vX, double vY, double vZ, double vYaw, double vPitch, double vRoll)
{
	std::string outputCmd = CONTROL_VEL;
	std::vector<double> outputSpeed(6, 0.0); // x, y, z, yaw, pitch, roll
	outputSpeed[0] = vX;
	outputSpeed[1] = vY;
	outputSpeed[2] = vZ;
	outputSpeed[3] = vYaw;
	outputSpeed[4] = vPitch;
	outputSpeed[5] = vRoll;
	for(size_t i=0; i<outputSpeed.size(); i++)
	{
		outputCmd += " ";
		// outputCmd += std::to_string(outputSpeed[i]);
		outputCmd += to_string_with_high_precision(outputSpeed[i], 3);
	}
	outputCmd += std::string("\r\n");
	serialPort.Write((outputCmd).c_str());
	ROS_DEBUG("Set velocity : (%s)", outputCmd.c_str());
}

void DogDriverNode::cmdVelHandler(const geometry_msgs::Twist::ConstPtr cmdVel)
{
	// ROS_INFO("RECEIVE MSG");
	// mtx.lock();
	setVelocity(cmdVel->linear.x, cmdVel->linear.y, cmdVel->linear.z,
					cmdVel->angular.z, cmdVel->angular.y, cmdVel->angular.x);

	lastMovingTime = ros::Time::now().toSec();
	isMoving = true;
	// mtx.unlock();
}

void DogDriverNode::ResetOdomIntegratorCallback(const std_msgs::Bool::ConstPtr& msg)
{
	// mtx.lock();
	if(msg->data)
	{
		curPosition.setZero();
		curPose.setIdentity();
	}
	// mtx.unlock();
}

void DogDriverNode::spin()
{
    ros::Rate loop_rate(framerate_);
    while(ros::ok())
    {
		checkPort();
		ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dog_driver");
    DogDriverNode DogDriverNode_;
    DogDriverNode_.spin();
    return 0;
}
