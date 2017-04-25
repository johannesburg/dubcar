#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

namespace teleop {

class Teleop 
{
  public:
    Teleop(ros::NodeHandle nh);
  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::NodeHandle nh_;
    ros::Publisher ackermann_pub_;
    ros::Subscriber joy_sub_;
};

}
