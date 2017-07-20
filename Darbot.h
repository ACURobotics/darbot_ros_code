//Includes the necessary packages and message types
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "motor_control.h"
#include <pigpio.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <string>
#include <cmath>
#include <signal.h>

//Specifies which GPIO pins are used
#define LTRIG 		12
#define CTRIG 		21
#define RTRIG 		13


#define LEFTECHO	19
#define CENTERECHO 	20
#define RIGHTECHO	16

#define ECHO_TIMEOUT	800 //200 microseconds; we'll try this value and see
#define SENSOR_VAR_THRESH	.1 // only <10% variability between subsequent measurements allowed
#define SENSOR_TIMEOUT	23250 //this is the maximum number of microseconds to wait for sensor echo

namespace darbot
{
	class Darbot
	{
		//Simply lists all of the public and private functions and variables of the Darbot class.
		public:
			Darbot(const ros::NodeHandle& nh);

			~Darbot();
			void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel);
			void velCalibrateCallback(const geometry_msgs::Twist::ConstPtr& vel);
			double distanceCheck(int triggerPin, int echoPin);
			void publishSensorData();
			ros::NodeHandle nh_;
			ros::Subscriber velocity_sub_;
			ros::Publisher left_distance_pub_;
			ros::Publisher center_distance_pub_;
			ros::Publisher right_distance_pub_;
			ros::Publisher distance_pub_;

		private:
			int handle;
			char fbSpeed, lrSpeed;
			geometry_msgs::Twist cache;
	};
}
