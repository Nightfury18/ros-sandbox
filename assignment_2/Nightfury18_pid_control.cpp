#include "ros/ros.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include "Kraken_msgs/thrusterData6Thruster.h"
#include "kraken_msgs/imuData.h"
#include "kraken_msgs/absoluteRPY"
#include "resources/topicHeader.h"

#define _USE_MATH_DEFINES 
#include <cmath>

float finalYaw; // finalYaw to be taken as input 
float Perror = 0.0, Ierror = 0.0, Derror = 0.0;
float prevError = 0.0;
float Kp_L = 0.9, Kp_R = -0.9;
float Ki_L = 0.5, Ki_R = -0.5;
float Kd_L = 0, Kd_R = 0;


void yawUpdate(const kraken_msgs::absoluteRPY::ConstPtr &msg)
{
	prevError = Perror;
	Perror = (finalYaw - msg->yaw) * M_PI / 180;
	Perror = atan2(sin(Perror), cos(Perror)) * 180 / M_PI; // so that angle always lies in the range[-180, 180]
	Ierror += Perror;
	Derror = Perror - prevError;

	ROS_INFO("Final yaw to be achieved : %lf, Current yaw : %lf, error in yaw : %lf", finalYaw, msg->yaw, Perror);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pid");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe<kraken_msgs::absoluteRPY>(topicHeader.ABSOLUTE_RPY, 1000, yawUpdate);
	ros::Publisher pub = n.publish<kraken_msgs::thrusterData6Thruster>(topicHeader.CONTROL_PID_THRUSTER6, 1000);

	kraken_msgs::thrusterData6Thruster thrusterValues;
	
	while(ros::ok())
	{
		thrusterValues.data[0] = 0.0;
		thrusterValues.data[1] = 0.0;
		thrusterValues.data[2] = Kp_L * Perror + Ki_L * Ierror + Kd_L * Derror;
		thrusterValues.data[3] = Kp_R * Perror + Ki_R * Ierror + Kd_R * Derror;
		thrusterValues.data[4] = 0.0;
		thrusterValues.data[5] = 0.0;

		pub.publish(thrusterValues);
		ros::spinOnce();
	}

	return 0;
}
