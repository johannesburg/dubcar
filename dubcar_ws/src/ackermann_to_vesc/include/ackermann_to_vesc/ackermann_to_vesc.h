#ifndef ACKERMANN_TO_VESC_ACKERMANN_TO_VESC_H
#define ACKERMANN_TO_VESC_ACKERMANN_TO_VESC_H
#include "ros/ros.h"
#include <ackermann_msgs/AckermannDrive.h>

namespace ackermann_to_vesc
{

class AckermannToVesc {
  public:
    AckermannToVesc();
    ~AckermannToVesc();

  private:
    ros::Publisher servo_pub_;
    ros::Publisher current_pub_;
    ros::Subscriber ackermann_sub_;
    void ackermannCallback(const ackermann_msgs::AckermannDrive::ConstPtr& cmd);
};
} // end ackermann_to_vesc namespace
#endif
