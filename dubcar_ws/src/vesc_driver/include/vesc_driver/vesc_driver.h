#ifndef VESC_DRIVER_VESC_DRIVER_H
#define VESC_DRIVER_VESC_DRIVER_H
#include <ros/ros.h>
#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Empty.h"
#include <string>


namespace vesc_driver
{
class VescDriver 
{
  public:
    VescDriver(ros::NodeHandle nh);
    VescDriver(const std::string& port);
    ~VescDriver() {};
  private: 
    VescDriver();
    VescInterface vesc_;
    ros::Subscriber duty_cycle_sub_;
    ros::Subscriber current_sub_;
    ros::Subscriber current_brake_sub_;
    ros::Subscriber rpm_sub_;
    ros::Subscriber position_sub_;
    ros::Subscriber servo_position_sub_;
    ros::Subscriber reboot_sub_;
    ros::Subscriber send_alive_sub_;
    void setDutyCycleCallback(const std_msgs::Float32ConstPtr& msg);
    void setCurrentCallback(const std_msgs::Float32ConstPtr& msg);
    void setCurrentBrakeCallback(const std_msgs::Float32ConstPtr& msg);
    void setRpmCallback(const std_msgs::Int32::ConstPtr& msg);
    void setPositionCallback(const std_msgs::Float32ConstPtr& msg);
    void setServoPositionCallback(const std_msgs::Float32ConstPtr& msg);
    void rebootCallback(const std_msgs::EmptyConstPtr& msg);
    void sendAliveCallback(const std_msgs::EmptyConstPtr& msg);
};
} // end vesc_driver namespace
#endif
