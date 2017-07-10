#include "Darbot.h"

namespace darbot
{
	Darbot::Darbot(const ros::NodeHandle& nh)
	: nh_(nh)	//This assigns the paramter nh to the class variable nh_
	{
		gpioInitialise();	//Always necessary to use gpio
		handle = spiOpen(0, 32000, 0);	//Opens SPI channel so that Darbot can write to the digital pots. spiOpen() returns the handle for the channel, which is then saved for later use.

		//Initializes a Subscriber to listen to the cmd_vel topic. This is the topic that Darbot will receive its movement instructions from outside nodes (typically from Navigation, but can be any node that broadcasts on this topic, such as teleop_twist_keyboard). When it receives a message, it calls the velocityCallback function and passes the message as its parameter.
		velocity_sub_ = nh_.subscribe("cmd_vel", 1, &Darbot::velocityCallback, this);	

		//These three Publishers are used to send the distances recorded by the three Ultrasonic sensors to the Navigation class, where the information is then copied onto three of Navigation's class variables
		left_distance_pub_ = nh_.advertise<std_msgs::Float32>("left_distance", 1);	
		center_distance_pub_ = nh_.advertise<std_msgs::Float32>("center_distance", 1);
		right_distance_pub_ = nh_.advertise<std_msgs::Float32>("right_distance", 1);

		//This Publisher takes all of the data from the ultrasonics and outputs it as a single message. Useful for running rostopic echo
		distance_pub_ = nh_.advertise<std_msgs::String>("distance", 1);
		
		//Used to give Navigation the go-ahead to process the sensor information.
		navigate_pub_ = nh_.advertise<std_msgs::Empty>("navigate", 1);

		//Used to ensure that Darbot will be stationary when turned on.
		fbSpeed = fbStop;
		lrSpeed = lrStop;
		int fbMessage = (fbStop << 8 | fbPostfix);
		int lrMessage = (lrStop << 8 | lrPostfix);
		spiWrite(handle, (char*) &fbMessage, 2);
		spiWrite(handle, (char*) &lrMessage, 2);

		//Sets the modes for the GPIO pins.
		gpioSetMode(TRIG, PI_OUTPUT);
		gpioSetMode(LEFTECHO, PI_INPUT);
		gpioSetMode(CENTERECHO, PI_INPUT);
		gpioSetMode(RIGHTECHO, PI_INPUT);

		//Makes sure the trigger is settles before it is used by the sensor.
		gpioWrite(TRIG, 0);
		ros::Duration(1).sleep();

		ROS_INFO_STREAM("Ready for commands.");
	}

	Darbot::~Darbot() //Currently not being called for some reason.
	{
		//Resets the Darbot to the stop position, then closes the SPI channel and turns off the GPIO pins.

		int fbMessage = (fbStop << 8 | fbPostfix);
		int lrMessage = (lrStop << 8 | lrPostfix);

		spiWrite(handle, (char*) &fbMessage, 2);
		spiWrite(handle, (char*) &lrMessage, 2);
  
		spiClose(handle);
		gpioTerminate();
	}

	void Darbot::velocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
	{
		//Extracts data from the Twist message and assigns it to local variables
		float lin_vel_, ang_vel_;
		lin_vel_ = vel->linear.x;
		ang_vel_ = vel->angular.z;
  
		//Interprets the data to decide how the Darbot should move. First interprets forward/backward, then left/right. Has a deadzone of (-0.3, 0.3). This is necessary if using a virtual joystick. Not necessary if only receiving commands from Navigation. The definitions for forwardFull, backwardFull, etc. are found in motor_control.h and the values were determined experimentally.
		if(lin_vel_  > 0.3)
			fbSpeed = forwardFull;
		else if(lin_vel_  < -0.3)
			fbSpeed = backwardFull;
		else
			fbSpeed = fbStop;
  
		if(ang_vel_  > 0.3)
			lrSpeed = leftFull;
		else if(ang_vel_  < -0.3)
			lrSpeed = rightFull;
		else
			lrSpeed = lrStop;

		//Each axis has a postfix associated with it so the digital pot knows which channel to change. This tacks on the postfix to the end of the velocity message.
		int fbMessage = (fbSpeed << 8 | fbPostfix);
		int lrMessage = (lrSpeed << 8 | lrPostfix);

		//This uses the handle variable that was saved earlier in order to specify the correct SPI channel. The velocity message must be casted as a char* in order to be correctly understood by the digital pot. The 2 specifies that the message is 2 bytes long (one byte for the speed, one byte for axis).
		spiWrite(handle, (char*) &fbMessage, 2);
		spiWrite(handle, (char*) &lrMessage, 2);
	}

	//This function is used for calibration only. Instead of interpreting the Twist message as its new velocity, it interprets the Twist as an incremental change in velocity. This was used to find the deadzones in the DC motors themselves.

/*	void Darbot::velCalibrateCallback(const geometry_msgs::Twist::ConstPtr& vel)
	{
		float lin_vel_, ang_vel_;
  
		// ROS_INFO_STREAM("velocityCallback called");
		lin_vel_ = vel->linear.x;
		ang_vel_ = vel->angular.z;

		//ROS_INFO_STREAM("[" << vel->linear.x << "," << vel->linear.y << "," << vel->linear.z << "], [" << vel->angular.x << "," << vel->angular.y << "," << vel->angular.z << "]");

		if(lin_vel_  > 0.3)
			fbSpeed = fbSpeed + 0x02;
		else if(lin_vel_  < -0.3)
			fbSpeed = fbSpeed - 0x02;
  
		if(ang_vel_  > 0.3)
			lrSpeed = lrSpeed + 0x02;
		else if(ang_vel_  < -0.3)
			lrSpeed = lrSpeed - 0x02;

		ROS_INFO_STREAM("[fbSpeed: " << (int) fbSpeed << ", lrSpeed: " << (int) lrSpeed << "]");

		int fbMessage = (fbSpeed << 8 | fbPostfix);
		int lrMessage = (lrSpeed << 8 | lrPostfix);

		spiWrite(handle, (char*) &fbMessage, 2);
		spiWrite(handle, (char*) &lrMessage, 2);
	}
*/

	//Senses the current distance from an ultrasonic sensor. The specific sensor is denoted by the echoPin parameter.
	double Darbot::distanceCheck(int echoPin)
	{
		//The HC-SR04 Ultrasonic Sensor is activated by a pulse sent to the TRIG pin (whose GPIO pin number is defined in the Darbot.h file). Once the sensor receives the pulse, it sets the ECHO pin to 5V (the GPIO pins only accepts 0V-3.3V as inputs, so the voltage is scaled down via a voltage divider) and sends out an ultrasonic pulse. The pulse will then propagate until it hits a surface and bounces back. When the sensor records the ultrasonic pulse, it sets the ECHO pin back to 0V. The length of time that ECHO is set to 5V determines how far away the object is.

		//Declares variables that will be used to record the start and end times of the ultrasonic pulse.
		ros::Time pulse_start;
		ros::Time pulse_end;
		
		//If an object is very close to the sensor, the ultrasonic pulse will return to the sensor before ROS even detects that EHCO went high. If that happens, it will entirely miss the beginning of the ECHO pulse and gets stuck in an infinite loop. In order to prevent that, the limit variable is incremented in the following loops and will cause ROS to exit the loop.
		int limit = 0;
		
		//This function sends a pulse along the TRIG pin. If it is sucessful, it will return 0. If it is unsucessful, it bypasses the rest of the sensing entirely.		
		if(gpioTrigger(TRIG, 20, 1) == 0)
		{		
			//ROS will stay in this while loop until the designated echoPin goes to 5V. If something goes wrong and it gets stuck in the loop, the limit variable will cause it to exit.
			while(gpioRead(echoPin) == 0 && limit < 100000)
			{
				limit++;
			}
	
			//If the limit variable had to be used to exit the loop, the reading must be ignored, so -1 is returned.
			if(limit >= 100000)
			{
				ROS_INFO_STREAM("ECHO Failed to go high");
				return -1;
			}

			//Now that ECHO has been set to 5V, the time is recorded.
			pulse_start = ros::Time::now();

			//The limit variable is reset, and then repeats the previous step, with the exception that it is now waiting for ECHO to go back to 0V, indicating that the ultrasonic pulse has been detected.	
			limit = 0;
			while(gpioRead(echoPin) == 1 && limit < 100000)
			{
				limit++;
			}

			if(limit >= 100000)
			{
				ROS_INFO_STREAM("ECHO failed to go low.");
				return -1;
			}

			pulse_end = ros::Time::now();

			//The duration of the ECHO pulse is then calculated and then used to calculate the distance of the object. 17150 was calculated using the speed of sound and the path of the pulse. The distance is measured in cm.
			ros::Duration duration = pulse_end - pulse_start;

			if(duration.toSec() < 0.0003)
			{
				return -1;
			}

			double distance = duration.toSec()*17150;	

			return distance;
		}
		else
		{
			//If the initial trigger pulse failed, -1 is returned.
			ROS_INFO_STREAM("Trigger pulse failed.");
			return -1;
		}
	}

	//This function is used to gather all of the data from the sensors and publish them to the Navigation node.
	void Darbot::publishSensorData()
	{
		//Checks the distance from each ultrasonic sensor and publishes it to Navigation. LEFTECHO, CENTERECHO, and RIGHTECHO are defined in Darbot.h
		std_msgs::Float32 left_dist;
		left_dist.data = distanceCheck(LEFTECHO);
		left_distance_pub_.publish(left_dist);

		std_msgs::Float32 center_dist;
		center_dist.data = distanceCheck(CENTERECHO);
		center_distance_pub_.publish(center_dist);

		std_msgs::Float32 right_dist;
		right_dist.data = distanceCheck(RIGHTECHO);
		right_distance_pub_.publish(right_dist);

		//Outputs all of the distances as a single String. Useful for running rostopic echo.
	//	std_msgs::String dist;
		
	//	dist.data = "[" + left_dist.data + ", " + center_dist.data + ", " + right_dist.data + "]";
		//distance_pub_.publish(dist);

		//Once Navigation has recorded the updated distances, it is given the go-ahead to process the data.
		std_msgs::Empty navigate;
		navigate_pub_.publish(navigate);
	}	
}


int main(int argc, char** argv)
{
	//Sets up the ROS node and creates an instance of the Darbot class.
	ros::init(argc, argv, "Darbot", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	darbot::Darbot darb = darbot::Darbot(nh);
	
	//This loop perpetually updates the sensor data. The spinOnce() command ensures that the Publishers and Subscribers are active.	
	while(ros::ok())
	{
		darb.publishSensorData();
		ros::spinOnce();
	}
}

