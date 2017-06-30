#include "Darbot.h"

namespace darbot
{

Darbot::Darbot(const ros::NodeHandle& nh)
: nh_(nh)
{
  gpioInitialise();
  handle = spiOpen(0, 32000, 0);
  velocity_sub_ = nh_.subscribe("cmd_vel", 1, &Darbot::velocityCallback, this);
 // velocity_sub_ = nh_.subscribe("cmd_vel", 1, &Darbot::velCalibrateCallback, this);
  fbSpeed = fbStop;
  lrSpeed = lrStop;
}

Darbot::~Darbot(){
  int fbMessage = (fbStop << 8 | fbPostfix);
  int lrMessage = (lrStop << 8 | lrPostfix);

  spiWrite(handle, (char*) &fbMessage, 2);
  spiWrite(handle, (char*) &lrMessage, 2);
  
  spiClose(handle);
  gpioTerminate();
}

void Darbot::velocityCallback(const geometry_msgs::Twist::ConstPtr& vel)
{
  float lin_vel_, ang_vel_;
  
  ROS_INFO_STREAM("velocityCallback called");
  lin_vel_ = vel->linear.x;
  ang_vel_ = vel->angular.z;

  ROS_INFO_STREAM("[" << vel->linear.x << "," << vel->linear.y << "," << vel->linear.z << "], [" << vel->angular.x << "," << vel->angular.y << "," << vel->angular.z << "]");

  ROS_INFO_STREAM("[" << lin_vel_ << ", " << ang_vel_ << "]");
  
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

  int fbMessage = (fbSpeed << 8 | fbPostfix);
  int lrMessage = (lrSpeed << 8 | lrPostfix);

  spiWrite(handle, (char*) &fbMessage, 2);
  spiWrite(handle, (char*) &lrMessage, 2);
}

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
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Darbot", ros::init_options::NoSigintHandler);
  //ros::NodeHandlePtr nh;
 // nh.reset(new ros::NodeHandle);
  

  ros::NodeHandle nh;
  //ros::NodeHandle* nh = new ros::NodeHandle;
  darbot::Darbot darb = darbot::Darbot(nh);
  ros::spin();
}

