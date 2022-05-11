#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Int8.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <iostream>

void artecCallbackIn(const std_msgs::String::ConstPtr& msg) {
	std::cout << "Received save msg: " << msg << std::endl;
}

int main(int argc, char* argv[])
{
	std::wcout << L"TEST" << std::endl;
	// This must be called before anything else ROS-related
	ros::init(argc, argv, "artec_TestAblauf");
	// Create a ROS node handle
	ros::NodeHandle nh;

	ROS_INFO("Hello, World!");

	ros::Publisher startPub = nh.advertise<std_msgs::String>("artec_capture/start", 100);
	ros::Publisher stopPub = nh.advertise<std_msgs::String>("artec_capture/stop", 100);
	ros::Subscriber startSub = nh.subscribe("artec_capture/objsaved", 10, artecCallbackIn);
	std_msgs::String msg;
	msg.data = "start";
	ros::Duration(5).sleep();
	startPub.publish(msg);
	std::cout << "start msg sendet" << std::endl;

	ros::Duration(10).sleep();

	msg.data = "stop";

	stopPub.publish(msg);
	std::cout << "stop msg sendet" << std::endl;


	// Don't exit the program.
	ros::spin();
}