#include <ros/ros.h>
#include "vesc_driver/vesc_driver.h"
#include "vesc_driver/vesc_packet.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include <string>
#include <stdio.h>

namespace vesc_driver 
{

VescDriver::VescDriver(ros::NodeHandle nh) 
{
  const std::string port = "hi"; // TODO: use rosparam to get port
  vesc_ = VescInterface(port);
  current_sub_ = nh.subscribe<std_msgs::Float32>("commands/motor/current", 10, &this::setCurrentCallback, &this);
}


void setCurrentCallback(const std_msgs::Float32ConstPtr& msg)
{
  VescPacket::VescPacket packet;
  float current = msg->data; 
  // TODO: check current boundries
  packet = createCurrentCmd(current);
  printf("Current received: %f \n", current);
  vesc_.send(packet);
}


} // end namespace vesc_driver
