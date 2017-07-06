//Includes the necessary packages and message tpes.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>

#define LIMIT 50

namespace darbot
{
	class Navigation
	{
		//Simply lists all of the public functions and variables of the Navigation class.
		public:
			Navigation(const ros::NodeHandle& nh);
			~Navigation();
			void recordLeft(const std_msgs::Float32 dist);
			void recordCenter(const std_msgs::Float32 dist);
			void recordRight(const std_msgs::Float32 dist);
			void navigationCallback(const std_msgs::Empty go);

			ros::NodeHandle nh_;
			ros::Publisher to_Darbot;
			ros::Subscriber left_distance_reader;
			ros::Subscriber center_distance_reader;
			ros::Subscriber right_distance_reader;
			ros::Subscriber navigate_reader;

			geometry_msgs::Twist stop;

			double left_distance;
			double center_distance;
			double right_distance;
	};
}
