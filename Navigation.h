#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>


namespace darbot
{
	class Navigation
	{
		public:
			Navigation(const ros::NodeHandle& nh);
			~Navigation();
			void navigationCallback(const std_msgs::Float32 dist);
			ros::NodeHandle nh_;
			ros::Publisher to_Ultrasonic;
			ros::Publisher to_Darbot;
			ros::Subscriber distance_reader;
	};
}
