#ifndef VESC_DRIVER_VESC_DRIVER_H
#define VESC_DRIVER_VESC_DRIVER_H
#include <ros/ros.h>
#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
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
    ros::Subscriber current_sub_;
    void setDutyCycleCallback(const std_msgs::Float32ConstPtr& duty_cycle);
    void setCurrentCallback(const std_msgs::Float32ConstPtr& msg);
    void setCurrentBrakeCallback(const std_msgs::Float32ConstPtr& brake);
    void setRpmCallback(const std_msgs::Int32::ConstPtr& rpm);
    void setPositionCallback(const std_msgs::Float32ConstPtr& position);
    void setServoPositionCallback(const std_msgs::Float32ConstPtr servo);
    void rebootCallback();
    void sendAliveCallback();
};
} // end vesc_driver namespace
#endif
