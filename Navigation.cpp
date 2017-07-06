//Includes the Navigation.h file, which has all of the other important package inclusions
#include "Navigation.h"

namespace darbot
{
	Navigation::Navigation(const ros::NodeHandle& nh)
	: nh_(nh) //Simply stores the value of nh into nh_
	{
		//Initializes all of the necessary subsrcibers. Whenever a message is sent on that topic, these subscribers will call the specified callback function and pass the messge as a paramter. The distance readers are published to in the Darbot publishSensorData() function and tell Navigation to record the given values. The navigate_reader waits until Darbot gives Navigation the go-ahead to process the sensor data.
		left_distance_reader = nh_.subscribe("left_distance", 1, &Navigation::recordLeft, this);
		center_distance_reader = nh_.subscribe("center_distance", 1, &Navigation::recordCenter, this);
		right_distance_reader = nh_.subscribe("right_distance", 1, &Navigation::recordRight, this);
		navigate_reader = nh_.subscribe("navigate", 1, &Navigation::navigationCallback, this);

		
		//Initializes the Twist publisher. This sends messages to the Darbot class that specify what its current velocity should be.
		to_Darbot = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		//Defines a Twist message to be rest velocity.		
		stop.linear.x = 0;
		stop.angular.z = 0;
	}

	//Destructor that is called whenever Navigation is shut down.
	Navigation::~Navigation()
	{
		//Sends a message to Darbot to stop moving. Used as a precaution to prevent the Darbot from moving while Navigation is down.
		to_Darbot.publish(stop);
	}

	//Each of the three functions simply serve to record data passed over the distance reader topics. Whenever a message is sent on those topics, ROS calls one of these three callback functions, where the distances will then be stored into class variables. This allows the entire Navigation class to have access to the distance measurements at all times.
	void Navigation::recordLeft(const std_msgs::Float32 dist)
	{
		left_distance = dist.data;
	}

	void Navigation::recordCenter(const std_msgs::Float32 dist)
	{
		center_distance = dist.data;
	}

	void Navigation::recordRight(const std_msgs::Float32 dist)
	{
		right_distance = dist.data;
	}

	//Once Darbot's publishSensorData() gives Navigation the go-ahead to process the sensor data by sending a message over the "navigate" topic, ROS calls this callback function.
	void Navigation::navigationCallback(const std_msgs::Empty go)
	{
		//Declares a Twist message that will then be initialized according to the sensor data.
		geometry_msgs::Twist msg;		
		
		//If any of the distance measurements have encountered an error, Navigation tells the Darbot to stop.
		if(left_distance == -1 || center_distance == -1 || right_distance == -1)
		{
			msg = stop;
		}
		else
		{
			//These booleans are declared to simplify the sensor processesing. If one of the sensors detects that an object is closer than the defined limit (LIMIT is defined in Navigation.h), then its respective boolean is set to true. Otherwise, it is false.
			bool leftClose = left_distance < LIMIT;
			bool centerClose = center_distance < LIMIT;
			bool rightClose = right_distance < LIMIT;

			//These if statements are set up so that some scenarios have precedence over others. The highest priority is if the center is too close. The next highest is if both left and right are too close. Then if right is too close, and finally if left is too close. The priority of left versus right was chosen arbitrarily. However, it should never be an issue, since if both left and right are too close, then the third if statement takes priority.
			if(leftClose)
			{
				//Moves backward for 0.5 seconds.
				msg.linear.x = -1;
				msg.angular.z = 0;
				to_Darbot.publish(msg);
				ros::Duration(.5).sleep();
				
				//Turns right for 2 seconds.
				msg.linear.x = 0;
				msg.angular.z = -1;
				to_Darbot.publish(msg);
				ros::Duration(2).sleep();
			}
			
			if(rightClose)
			{
				//Moves backward for 0.5 seconds.
				msg.linear.x = -1;
				msg.angular.z = 0;
				to_Darbot.publish(msg);
				ros::Duration(.5).sleep();
				
				//Turns left for 2 seconds.
				msg.linear.x = 0;
				msg.angular.z = 1;
				to_Darbot.publish(msg);
				ros::Duration(2).sleep();
			}
			
			//Ideally, this if statement would never be satisfied since both leftClose and rightClose are updated nearly continuously and the chances that they would both become too close simultaneously are slim. However, the sensors are not ideal. This is used to hopefully reset the Darbot into a position that it can more easily interpret.
			if(leftClose && rightClose)
			{
				//Moves backward for 1 second.
				msg.linear.x = -1;
				msg.angular.z = 0;
				to_Darbot.publish(msg);
				ros::Duration(1).sleep();
			}

			if(centerClose)
			{
				//Moves backwards for 0.5 seconds.
				msg.linear.x = -1;
				msg.angular.z = 0;
				to_Darbot.publish(msg);
				ros::Duration(.5).sleep();

				//First specifies that it will not move forward or backward, then uses the left and right sensor data to determine if it should move left or right, which it then does for 4 seconds.
				msg.linear.x = 0;
				
				if(right_distance < left_distance)
					msg.angular.z = 1;
				else
					msg.angular.z = -1;

				to_Darbot.publish(msg);
				ros::Duration(4).sleep();
			}
		}
	}
}

int main(int argc, char** argv)
{
	//Sets up the ROS node and instantiates the Navigation class. ros::spin() ensures that the Publishers and Subscribers will update continuously.
	ros::init(argc, argv, "Navigation", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	darbot::Navigation navi = darbot::Navigation(nh);
	ros::spin();
}

