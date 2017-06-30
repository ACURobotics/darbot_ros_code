
#include "Ultrasonic.h"

namespace darbot
{

Ultrasonic::Ultrasonic(const ros::NodeHandle& nh)
: nh_(nh)
{
	gpioInitialise();
	
	gpioSetMode(TRIG, PI_OUTPUT);
	gpioSetMode(ECHO, PI_INPUT);
	gpioWrite(TRIG, 0);
	for(int i=0; i<1000000; i++)
	{}
	

	ultrasonic_sub_ = nh_.subscribe("ultrasonic_check", 1, &Ultrasonic::distanceCallback, this);
	ultrasonic_pub_ = nh_.advertise<std_msgs::Float32>("distance", 1);
}

void Ultrasonic::distanceCallback(const std_msgs::Empty& check)
{	
	ros::Time pulse_start;
	ros::Time pulse_end;

	if(gpioWrite(TRIG, 1) == 0)
	{
		for(int j=0; j<1000; j++) {}
		gpioWrite(TRIG,0);
	}
	while(gpioRead(ECHO) == 0)
	{
		pulse_start = ros::Time::now();
	}
	while(gpioRead(ECHO) == 1)
	{
		pulse_end = ros::Time::now();
	}

	//double duration = difftime(t2,t1);
	
	ros::Duration duration = pulse_end - pulse_start;

	std_msgs::Float32 dist;
	dist.data = duration.toSec()*17150;
	ultrasonic_pub_.publish(dist);
}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "Ultrasonic", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	darbot::Ultrasonic ultra = darbot::Ultrasonic(nh);
	ros::spin();
}
