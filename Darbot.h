#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "motor_control.h"
#include <pigpio.h>

namespace darbot
{
	class Darbot
	{
		public:
			Darbot(const ros::NodeHandle& nh);

			~Darbot();
			void velocityCallback(const geometry_msgs::Twist::ConstPtr& vel);
			void velCalibrateCallback(const geometry_msgs::Twist::ConstPtr& vel);
			ros::NodeHandle nh_;
			ros::Subscriber velocity_sub_;

		private:
			int handle;
			char fbSpeed, lrSpeed;
	};
}
