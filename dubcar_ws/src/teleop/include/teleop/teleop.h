#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/Joy.h>
#include <cmath>


namespace teleop {

class Teleop 
{
  public:
    
    Teleop(ros::NodeHandle nh);
    
  
  private:
   
    // TODO: better names? 
    static constexpr float max_steering_angle_ = M_PI / 8; // radians
    static constexpr float max_speed_= 2.5; // m/s
    static constexpr float steering_angle_velocity_ = 0; // change steering as quickly as possible
    static constexpr float acceleration_ = 0; // change speed as quickly as possible
    static constexpr float jerk_ = 0; // change acceleration as quickly as possible
    

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    
    ros::NodeHandle nh_;
    ros::Publisher ackermann_pub_;
    ros::Subscriber joy_sub_;
};

}
