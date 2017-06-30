#include "Navigation.h"

namespace darbot
{
	Navigation::Navigation(const ros::NodeHandle& nh)
	: nh_(nh)
	{
		distance_reader = nh_.subscribe("distance", 1, &Navigation::navigationCallback, this);
		to_Ultrasonic = nh_.advertise<std_msgs::Empty>("ultrasonic_check", 1);
		to_Darbot = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	}

	Navigation::~Navigation()
	{
		geometry_msgs::Twist stop;
		stop.linear.x = 0;
		stop.angular.z = 0;
		to_Darbot.publish(stop);
	}

	void Navigation::navigationCallback(const std_msgs::Float32 dist)
	{
		double distance = dist.data;
		geometry_msgs::Twist msg;		
		
		if(distance < 100)
		{
			msg.linear.x = -1;
			to_Darbot.publish(msg);

			ros::Duration(.5).sleep();

			msg.linear.x = 0;
			msg.angular.z = 1;
			to_Darbot.publish(msg);

			ros::Duration(1).sleep();
		}
		else
		{
			msg.linear.x = 1;
			to_Darbot.publish(msg);
			
			ros::Duration(.1).sleep();
		}
		
		std_msgs::Empty emptyMessage;
		to_Ultrasonic.publish(emptyMessage);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Navigation", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	darbot::Navigation navi = darbot::Navigation(nh);
	std_msgs::Empty start;	
	navi.to_Ultrasonic.publish(start);
	ros::spin();
}

