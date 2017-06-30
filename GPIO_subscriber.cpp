#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <pigpio.h>


void turnLEDOn(const std_msgs::Empty& toggle_msg)
{
	gpioWrite(2, 1);
}

void turnLEDOff(const std_msgs::Empty& toggle_msg)
{
	gpioWrite(2, 0);
}

int main(int argc, char **argv)
{
	gpioInitialise();
	gpioSetMode(2, PI_OUTPUT);
	
	ros::init(argc, argv, "subscribe_to_LED");
	ros::NodeHandle nh;
	
	ros::Subscriber subOn = nh.subscribe("turnOn", 1000, &turnLEDOn);
	ros::Subscriber subOff = nh.subscribe("turnOff", 1000, &turnLEDOff); 
	
	ros::spin();

	gpioTerminate();
}
