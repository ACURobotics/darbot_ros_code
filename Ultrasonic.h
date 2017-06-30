#include <ros/ros.h>
#include <pigpio.h>
#include <time.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#define TRIG 21
#define ECHO 20


namespace darbot
{
	class Ultrasonic
	{
		public:
			Ultrasonic(const ros::NodeHandle& nh);

			void distanceCallback(const std_msgs::Empty& check);
			ros::NodeHandle nh_;
			ros::Publisher ultrasonic_pub_;
			ros::Subscriber ultrasonic_sub_;
	};
}
