#include <ros/ros.h>
#include "vesc_driver/vesc_driver.h"
#include "vesc_driver/vesc_packet.h"
#include "vesc_driver/vesc_interface.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"
#include <string>
#include <stdio.h>
#include <stdexcept>

namespace vesc_driver 
{

VescDriver::VescDriver(ros::NodeHandle nh)
{
  std::string port;
  if (nh.getParam("vesc_driver/port", port)) {
    vesc_.connect(port);
  } else {
    throw std::invalid_argument("Error! invalid port received: " + port + "\n"); 
  }
 

  duty_cycle_sub_ = nh.subscribe<std_msgs::Float32>("commands/motor/duty_cycle", 10, &VescDriver::setDutyCycleCallback, this);
  current_sub_ = nh.subscribe<std_msgs::Float32>("commands/motor/current", 10, &VescDriver::setCurrentCallback, this);
  current_brake_sub_ = nh.subscribe<std_msgs::Float32>("commands/motor/current_brake", 
                                                        10, &VescDriver::setCurrentBrakeCallback, this);
  rpm_sub_ = nh.subscribe<std_msgs::Int32>("commands/motor/rpm", 10, &VescDriver::setRpmCallback, this);
  position_sub_ = nh.subscribe<std_msgs::Float32>("commands/servo/position", 10, &VescDriver::setPositionCallback, this);
  servo_position_sub_ = nh.subscribe<std_msgs::Float32>("commands/servo/servo_position", 10, &VescDriver::setServoPositionCallback, this);
  reboot_sub_ = nh.subscribe<std_msgs::Empty>("commands/vesc/reboot", 10, &VescDriver::rebootCallback, this);
//  send_alive_sub_ = nh.subscribe<std_msgs::Empty>("commands/vesc/alive", 10, &VescDriver::sendAliveCallback, this);
}

// TODO: move stack allocated values to commands?
void VescDriver::setDutyCycleCallback(const std_msgs::Float32ConstPtr& msg)
{
  float duty_cycle = msg->data; 
  printf("Duty cycle received: %f \n", duty_cycle);
  vesc_.send(VescPacket::createDutyCycleCmd(duty_cycle));
}

void VescDriver::setCurrentCallback(const std_msgs::Float32ConstPtr& msg)
{
  float current = msg->data; 
  printf("Current received: %f \n", current);
  vesc_.send(VescPacket::createCurrentCmd(current));
}


void VescDriver::setCurrentBrakeCallback(const std_msgs::Float32ConstPtr& msg)
{
  float brake = msg->data; 
  printf("Current brake received: %f \n", brake);
  vesc_.send(VescPacket::createCurrentBrakeCmd(brake));
}

void VescDriver::setRpmCallback(const std_msgs::Int32::ConstPtr& msg)
{
  int rpm = msg->data; 
  int max = 15000;
  int min = -max;
  printf("RPM received: %d \n", rpm);
  if (rpm >= min && rpm <= max) {
    vesc_.send(VescPacket::createRpmCmd(rpm));
  } else {
    printf("RPM %d is outside of bounds ... clipping \n", rpm);
    printf("min: %d max: %d \n", min, max);
    if (rpm > max) {
      vesc_.send(VescPacket::createRpmCmd(max));
    } else if (rpm < min) {
      vesc_.send(VescPacket::createRpmCmd(min));
    }
  }
}

void VescDriver::setPositionCallback(const std_msgs::Float32ConstPtr& msg)
{
  float position = msg->data; 
  printf("Position received: %f \n", position);
  vesc_.send(VescPacket::createPositionCmd(position));
}

void VescDriver::setServoPositionCallback(const std_msgs::Float32ConstPtr& msg)
{
  float servo = msg->data; 
  printf("ServoPosition received: %f \n", servo);
  vesc_.send(VescPacket::createServoPositionCmd(servo));
}

void VescDriver::rebootCallback(const std_msgs::EmptyConstPtr& msg)
{
  printf("Reboot received!\n");
  vesc_.send(VescPacket::createRebootCmd());
}

void VescDriver::sendAliveCallback(const std_msgs::EmptyConstPtr& msg)
{
  printf("SendAlive received\n");
  vesc_.send(VescPacket::createSendAliveCmd());
}
} // end namespace vesc_driver
