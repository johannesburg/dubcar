#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

class TeleopDubcar
{
public:
  TeleopDubcar();
private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;
  uint32_t linear_, angular_; 
  double l_scale_, a_scale_;
  ros::Publisher motor_pub_;
  ros::Subscriber joy_sub_;
};

TeleopDubcar::TeleopDubcar() 
{

}
