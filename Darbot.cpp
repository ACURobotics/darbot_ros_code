#include "Darbot.h"

namespace darbot
{
	Darbot::Darbot(const ros::NodeHandle& nh)
	: nh_(nh)	//This assigns the paramter nh to the class variable nh_
	{
		gpioInitialise();	//Always necessary to use gpio
		handle = spiOpen(0, 32000, 0);	//Opens SPI channel so that Darbot can write to the digital pots. spiOpen() returns the handle for the channel, which is then saved for later use.

		//Initializes a cache Twist that is used to store the most recent velocity.
		cache.linear.x = 0;
		cache.angular.z = 0;

		//Initializes a Subscriber to listen to the cmd_vel topic. This is the topic that Darbot will receive its movement instructions from outside nodes (typically from Navigation, but can be any node that broadcasts on this topic, such as teleop_twist_keyboard). When it receives a message, it calls the velocityCallback function and passes the message as its parameter.
		velocity_sub_ = nh_.subscribe("cmd_vel", 1, &Darbot::velocityCallback, this);	

		//Used for calibration only
//		velocity_sub_ = nh_.subscribe("cmd_vel", 1, &Darbot::velCalibrateCallback, this);	

		//These three Publishers are used to send the distances recorded by the three Ultrasonic sensors to the Navigation class, where the information is then copied onto three of Navigation's class variables
		left_distance_pub_ = nh_.advertise<std_msgs::Float32>("left_distance", 1);	
		center_distance_pub_ = nh_.advertise<std_msgs::Float32>("center_distance", 1);
		right_distance_pub_ = nh_.advertise<std_msgs::Float32>("right_distance", 1);

		//This Publisher takes all of the data from the ultrasonics and outputs it as a single message. Useful for running rostopic echo
		distance_pub_ = nh_.advertise<std_msgs::String>("distance", 1);

		//Used to ensure that Darbot will be stationary when turned on.
		fbSpeed = fbStop;
		lrSpeed = lrStop;
		int fbMessage = (fbStop << 8 | fbPostfix);
		int lrMessage = (lrStop << 8 | lrPostfix);
		spiWrite(handle, (char*) &fbMessage, 2);
		spiWrite(handle, (char*) &lrMessage, 2);

		//Sets the modes for the GPIO pins.
		gpioSetMode(LTRIG, PI_OUTPUT);
		gpioSetMode(CTRIG, PI_OUTPUT);
		gpioSetMode(RTRIG, PI_OUTPUT);
		gpioSetMode(LEFTECHO, PI_INPUT);
		gpioSetMode(CENTERECHO, PI_INPUT);
		gpioSetMode(RIGHTECHO, PI_INPUT);

		//Makes sure the trigger is settled before it is used by the sensor.
		gpioWrite(LTRIG, 0);
		gpioWrite(CTRIG, 0);
		gpioWrite(RTRIG, 0);
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

		//Checks to see if this velocity command is different from the most recently called one. Saves computation time.
		if(lin_vel_ != cache.linear.x || ang_vel_ != cache.angular.z)
		{
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

			//Stores the command in the cache.
			cache.linear.x = lin_vel_;
			cache.angular.z = ang_vel_;
		}
	}

	//This function is used for calibration only. Instead of interpreting the Twist message as its new velocity, it interprets the Twist as an incremental change in velocity. This was used to find the deadzones in the DC motors themselves.

	void Darbot::velCalibrateCallback(const geometry_msgs::Twist::ConstPtr& vel)
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


	//Senses the current distance from an ultrasonic sensor. The specific sensor is denoted by the echoPin parameter.
	double Darbot::distanceCheck(int triggerPin, int echoPin)
	{
		//The HC-SR04 Ultrasonic Sensor is activated by a pulse sent to the TRIG pin (whose GPIO pin number is defined in the Darbot.h file). Once the sensor receives the pulse, it sets the ECHO pin to 5V (the GPIO pins only accepts 0V-3.3V as inputs, so the voltage is scaled down via a voltage divider) and sends out an ultrasonic pulse. The pulse will then propagate until it hits a surface and bounces back. When the sensor records the ultrasonic pulse, it sets the ECHO pin back to 0V. The length of time that ECHO is set to 5V determines how far away the object is.

		//Declares variables that will be used to record the start and end times of the ultrasonic pulse.
		uint32_t echoTimeoutStart, echoTime, start; //variables to hold the time we start the echo loop, the time we stay in the echo loop, and the beginning of the ultrasonic delay
		double diff = 0.0; //double, because we'll convert the integer number of microseconds contained in diff to seconds

		//Sometimes, the ECHO pin does not reset. This manually forces the ECHO pin to reset to 0.
		gpioSetMode(echoPin, PI_OUTPUT);
		gpioWrite(echoPin, 0);
		gpioSetMode(echoPin, PI_INPUT);

		//This function sends a pulse along the trigger pin. If it is sucessful, it will return 0. If it is unsucessful, it bypasses the rest of the sensing entirely.		
		if(gpioTrigger(triggerPin, 15, 1) == 0)
		{
			//Starts the timeout count
			echoTimeoutStart = gpioTick();
			echoTime = 0;
			//ROS will stay in this while loop until the designated echoPin goes to 5V. If something goes wrong and it gets stuck in the loop, the limit variable will cause it to exit.
			while(gpioRead(echoPin) == 0 && echoTime < ECHO_TIMEOUT){
				start = gpioTick(); //records the clock tick when echoPin goes high
				echoTime = start-echoTimeoutStart; //use this timeout to escape from the while loop if necessary; start already contains the return value from gpioTick();
			}

			if(echoTime >= ECHO_TIMEOUT){
				return -1; //failure to measure distance
			}

			while(gpioRead(echoPin) == 1 && diff < SENSOR_TIMEOUT){
				diff = gpioTick()-start; //number of microseconds we've waited
			}


			//The duration of the ECHO pulse is then calculated and then used to determine the distance of the object. 17150 was calculated using the speed of sound and the path of the pulse. The distance is measured in cm.
			diff = diff*1e-6; //converts from us to s
			double distance = diff*17200; //converts from s to cm

			if(distance > 10 && distance < 1000) //checks to make sure data is not an outlier
				return distance;
			else
				return  400; //if it is an outlier, it returns 400, the maximum range of the sensor
		}
		else
		{
			//If the initial trigger pulse failed, -1 is returned.
			return -1;
		}
	}

	//This function is used to gather all of the data from the sensors and publish them to the Navigation node.
	void Darbot::publishSensorData()
	{
		//Checks the distance from each ultrasonic sensor and publishes it to Navigation. LEFTECHO, CENTERECHO, and RIGHTECHO are defined in Darbot.h
		std_msgs::Float32 left_dist;
		left_dist.data = distanceCheck(LTRIG, LEFTECHO);
		left_distance_pub_.publish(left_dist);

		std_msgs::Float32 center_dist;
		center_dist.data = distanceCheck(CTRIG, CENTERECHO);
		center_distance_pub_.publish(center_dist);


		std_msgs::Float32 right_dist;
		right_dist.data = distanceCheck(RTRIG, RIGHTECHO);
		right_distance_pub_.publish(right_dist);

/*
		//Outputs all of the distances as a single String. Useful for running rostopic echo.
		std_msgs::String dist;
		dist.data = "[" + std::to_string(left_dist.data) + ", " + std::to_string(center_dist.data) + ", " + std::to_string(right_dist.data) + "]";
		distance_pub_.publish(dist);
*/
	}
}

int main(int argc, char** argv)
{
	//Sets up the ROS node and creates an instance of the Darbot class.
	ros::init(argc, argv, "Darbot");//, ros::init_options::NoSigintHandler);

	ros::NodeHandle nh;
	darbot::Darbot darb = darbot::Darbot(nh);

	ros::Rate loopRate(15);


	//This loop perpetually updates the sensor data. The spinOnce() command ensures that the Publishers and Subscribers are active.	
	while(ros::ok())
	{
		darb.publishSensorData();
		ros::spinOnce();
		loopRate.sleep();
	}
	ros::shutdown();
}

