//Includes the necessary packages and message tpes.
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <string>

#define WINDOWED_AVG_WIDTH 5 //keep smallish?
#define FRONTLIMIT 70*WINDOWED_AVG_WIDTH //on average, keep stuff at least 70 cm from the front
#define SIDELIMIT 70*WINDOWED_AVG_WIDTH  //on average, keep stuff at least 40 cm from the sides


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
			void navigationCallback(void);

			ros::NodeHandle nh_;
			ros::Publisher to_Darbot;
			ros::Subscriber left_distance_reader;
			ros::Subscriber center_distance_reader;
			ros::Subscriber right_distance_reader;
			ros::Subscriber navigate_reader;

			geometry_msgs::Twist stop;
			geometry_msgs::Twist cache;

			double leftBuffer[WINDOWED_AVG_WIDTH];
			double centerBuffer[WINDOWED_AVG_WIDTH];
			double rightBuffer[WINDOWED_AVG_WIDTH];

			int lIndex;
			int cIndex;
			int rIndex;

			double left_distance;
			double center_distance;
			double right_distance;

			bool backAndForth;
			int lastTurn;
	};
}
