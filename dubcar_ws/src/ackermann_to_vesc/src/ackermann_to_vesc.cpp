#include "ackermann_to_vesc/ackermann_to_vesc.h"
#include "ros/ros.h"
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Float32.h>

namespace ackermann_to_vesc
{

AckermannToVesc::AckermannToVesc(ros::NodeHandle& nh)
{
  // TODO use rosparam to load magic numbers (offset, gain, etc)
  nh.subscribe("ackermann", 10, &AckermannToVesc::ackermannCallback, this);
  servo_pub_ = nh.advertise<std_msgs::Float32>("commands/servo/position", 10);
  current_pub_ = nh.advertise<std_msgs::Float32>("commands/motor/current", 10);
}

void AckermannToVesc::ackermannCallback(const ackermann_msgs::AckermannDrive::ConstPtr& cmd) 
{
  //TODO convert cmd to servo position and current
  //TODO publish to this.servo_pub_ and this.current_pub_
  // see http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers#Publishing_to_a_Topic
  // for more info
}
} // end ackermann_to_vesc namespace
