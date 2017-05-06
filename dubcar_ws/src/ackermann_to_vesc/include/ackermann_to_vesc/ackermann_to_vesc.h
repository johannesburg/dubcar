#ifndef ACKERMANN_TO_VESC_ACKERMANN_TO_VESC_H
#define ACKERMANN_TO_VESC_ACKERMANN_TO_VESC_H
#include "ros/ros.h"
#include <ackermann_msgs/AckermannDrive.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

namespace ackermann_to_vesc
{

class AckermannToVesc {
  public:
    AckermannToVesc(ros::NodeHandle& nh);
    //~AckermannToVesc();

  private:
    ros::Publisher duty_cycle_pub_;
    ros::Publisher current_pub_;
    ros::Publisher current_brake_pub_;
    ros::Publisher rpm_pub_;
    ros::Publisher position_pub_;
    ros::Publisher servo_position_pub_;
    
    ros::Subscriber ackermann_sub_;
    
    void translateDutyCycleCallback(const std_msgs::Float32ConstPtr& msg);
    void translateCurrentCallback(const std_msgs::Float32ConstPtr& msg);
    void translateCurrentBrakeCallback(const std_msgs::Float32ConstPtr& msg);
    void translateRpmCallback(const std_msgs::Int32::ConstPtr& msg);
    void translatePositionCallback(const std_msgs::Float32ConstPtr& msg);
    void translateServoPositionCallback(const std_msgs::Float32ConstPtr& msg);
    void ackermannCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg);
};
} // end ackermann_to_vesc namespace
#endif
