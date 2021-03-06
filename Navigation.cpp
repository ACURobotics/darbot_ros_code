//Includes the Navigation.h file, which has all of the other important package inclusions
#include "Navigation.h"

namespace darbot
{
	Navigation::Navigation(const ros::NodeHandle& nh)
	: nh_(nh) //Simply stores the value of nh into nh_
	{
		//Initializes all of the necessary subsrcibers. Whenever a message is sent on that topic, these subscribers will call the specified callback function and pass the messge as a paramter. The distance readers are published to in the Darbot publishSensorData() function and tell Navigation to record the given values.
		left_distance_reader = nh_.subscribe("left_distance", 5, &Navigation::recordLeft, this);
		center_distance_reader = nh_.subscribe("center_distance", 5, &Navigation::recordCenter, this);
		right_distance_reader = nh_.subscribe("right_distance", 5, &Navigation::recordRight, this);

		//Initializes the Twist publisher. This sends messages to the Darbot class that specify what its current velocity should be.
		to_Darbot = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

		//Defines a Twist message to be rest velocity.
		stop.linear.x = 0;
		stop.angular.z = 0;

		//Initializes a cache Twist to store the most recent velocity command.
		cache.linear.x = 0;
		cache.angular.z = 0;

		//Used to determine if the Darbot is continually backing up and going forward.
		backAndForth = false;
		
		//Used to store the most recent turn direction. Used in the navigation algorithm.
		lastTurn = 1;

		left_distance = WINDOWED_AVG_WIDTH*100; //initialize these to 1m*WINDOWED_AVG_WIDTH
		center_distance = WINDOWED_AVG_WIDTH*100;
		right_distance = WINDOWED_AVG_WIDTH*100;

		lIndex = 0; //initialize these all to 0
		cIndex = 0;
		rIndex = 0;

		for(int i=0; i<WINDOWED_AVG_WIDTH; i++){
			leftBuffer[i] = 100; //initialize these all to 100cm
			centerBuffer[i] = 100; //initialize these all to 100cm
			rightBuffer[i] = 100; //initialize these all to 100cm
		}

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
		//OLD VERSION:
		//left_distance = dist.data;

		//NEW VERSION:
		//1) subtract next-overwritten value from distance		
		//1) Set next overwriteable buffer position to the current measurement
		//3) Add this value to running distance sum (acting as average)
		//2) Increment index using mod division

		left_distance -= leftBuffer[lIndex];
		leftBuffer[lIndex] = dist.data;
		left_distance += leftBuffer[lIndex];
		lIndex = (lIndex+1)%WINDOWED_AVG_WIDTH; //increment lIndex

		
	}

	void Navigation::recordCenter(const std_msgs::Float32 dist)
	{
		//old version
		//center_distance = dist.data;


		//NEW VERSION:
		//1) subtract next-overwritten value from distance		
		//1) Set next overwriteable buffer position to the current measurement
		//3) Add this value to running distance sum (acting as average)
		//2) Increment index using mod division

		center_distance -= centerBuffer[cIndex];
		centerBuffer[cIndex] = dist.data;
		center_distance += centerBuffer[cIndex];
		cIndex = (cIndex+1)%WINDOWED_AVG_WIDTH; //increment rIndex
	}

	void Navigation::recordRight(const std_msgs::Float32 dist)
	{
		//right_distance = dist.data;

		//NEW VERSION:
		//1) subtract next-overwritten value from distance		
		//1) Set next overwriteable buffer position to the current measurement
		//3) Add this value to running distance sum (acting as average)
		//2) Increment index using mod division

		right_distance -= rightBuffer[rIndex];
		rightBuffer[rIndex] = dist.data;
		right_distance += rightBuffer[rIndex];
		rIndex = (rIndex+1)%WINDOWED_AVG_WIDTH; //increment rIndex
	}

	//After obtaining the sensor data, the main function calls this.
	void Navigation::navigationCallback(void)
	{
		//ROS_INFO_STREAM("left: " << (left_distance/WINDOWED_AVG_WIDTH) << "   center: " << (center_distance/WINDOWED_AVG_WIDTH) << "   right: " << (right_distance/WINDOWED_AVG_WIDTH));

		//ROS_INFO_STREAM("left: " << (lIndex) << "   center: " << (cIndex) << "   right: " << (rIndex));

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
			bool leftClose = left_distance < SIDELIMIT; //these take into account the width of the windowed average (see Navigation.h for definition of SIDELIMIT and FRONTLIMIT)
			bool centerClose = center_distance < FRONTLIMIT;
			bool rightClose = right_distance < SIDELIMIT;


			//If any of the sensors are tripped, it sends a backup signal to begin the slowdown process.
			if(leftClose || centerClose || rightClose)
			{
				msg.linear.x = -1;
				msg.angular.z = 0;
				backAndForth = false;

				//These if statements are set up so that some scenarios have precedence over others. The highest priority is if the center is too close. The next highest is if both left and right are too close. Then if right is too close, and finally if left is too close. The priority of left versus right was chosen arbitrarily. However, it should never be an issue, since if both left and right are too close, then the third if statement takes priority.
				if(leftClose)
				{
					//Turns right.
					msg.linear.x = 0;
					msg.angular.z = -1;
					lastTurn = -1;
				}

				if(rightClose)
				{
					//Turns left.
					msg.linear.x = 0;
					msg.angular.z = 1;
					lastTurn = 1;
				}

				//Ideally, this if statement would never be satisfied since both leftClose and rightClose are updated nearly continuously and the chances that they would both become too close simultaneously are slim. However, the sensors are not ideal. This is used to hopefully reset the Darbot into a position that it can more easily interpret.
				if(leftClose && rightClose)
				{
					//Moves backward.
					msg.linear.x = -1;
					msg.angular.z = 0;
				}
				if(centerClose)
				{
					//Moves backwards.
					msg.linear.x = -1;
					msg.angular.z = 0;
				}
			}
			else
			{
				//Moves forward.
				msg.linear.x = 1;
				msg.angular.z = 0;

				//Checks to see if the previous command was to move backward. If so, sets backAndForth to true so that Navigation can break the loop.
				if(cache.linear.x == -1 && cache.angular.z == 0)
				{
					backAndForth = true;
				}
			}
		}

		//If it is moving back and forth, it tacks on a turn command to the Twist. The direction is the same as the last turn that the Darbot made.
		if(backAndForth)
		{
			msg.angular.z = lastTurn;
		}
		
		//Cache is updated
		cache.linear.x = msg.linear.x;
		cache.angular.z = msg.angular.z;

		to_Darbot.publish(msg); //publish one message per call
	}
}

int main(int argc, char** argv)
{
	//Sets up the ROS node and instantiates the Navigation class. ros::spin() ensures that the Publishers and Subscribers will update continuously.
	ros::init(argc, argv, "Navigation");//, ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	darbot::Navigation navi = darbot::Navigation(nh);

	ros::Rate loopRate(16);

	while(ros::ok()){
		navi.navigationCallback();
		ros::spinOnce();
		loopRate.sleep();
	}
}

