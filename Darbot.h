//Includes the necessary packages and message types
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "motor_control.h"
#include <pigpio.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

//Specifies which GPIO pins are used
#define TRIG 		21
#define LEFTECHO	19
#define CENTERECHO 	20
#define RIGHTECHO	16

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
			double distanceCheck(int echoPin);
			void publishSensorData();
			ros::NodeHandle nh_;
			ros::Subscriber velocity_sub_;
			ros::Publisher left_distance_pub_;
			ros::Publisher center_distance_pub_;
			ros::Publisher right_distance_pub_;
			ros::Publisher navigate_pub_;

		private:
			int handle;
			char fbSpeed, lrSpeed;
	};
}
