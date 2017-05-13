#include "ackermann_to_vesc/ackermann_to_vesc.h"
#include "ros/ros.h"
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

namespace ackermann_to_vesc
{

AckermannToVesc::AckermannToVesc(ros::NodeHandle& nh) : 

  // TODO: how the fuck to indent, braces, etc
  rpm_pub_(nh.advertise<std_msgs::Int32>("commands/motor/rpm", 20)),
  servo_position_pub_(nh.advertise<std_msgs::Float32>("commands/servo/servo_position", 20)),
  // TODO use rosparam to load magic numbers (offset, gain, etc)
  ackermann_sub_(nh.subscribe("commands/drivetrain/ackermann", 20, &AckermannToVesc::ackermannCallback, this, ros::TransportHints().udp()))
  {}


void AckermannToVesc::ackermannCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg) 
{
  // TODO: We cannot use current or rpm (maybe rpm....) without 
  // a PID controller. ROS is wayyyy too high level for this.
  // Controller needs to be low level....so we're going to have to
  // stick with rpm control, tune the PID, and maybe get a Hall
  // sensor for low speed issues
  
  // scale and set speed
  std_msgs::Int32::Ptr rpm_msg_out(new std_msgs::Int32);

  // 1 / (rpm_erpm_raio * gear_ratio * wheel_diam * pi * mile/in * min/hr);
  float scale = 4613; // TODO: VERIFY 
  rpm_msg_out-> data = (int32_t) (msg->speed * scale);

  std_msgs::Float32::Ptr servo_pos_msg_out(new std_msgs::Float32);
  servo_pos_msg_out->data = msg->steering_angle * -1.2134 + 0.5304; // TODO: VERIFY;
   
  rpm_pub_.publish(rpm_msg_out);
  servo_position_pub_.publish(servo_pos_msg_out);

}

} // end ackermann_to_vesc namespace
