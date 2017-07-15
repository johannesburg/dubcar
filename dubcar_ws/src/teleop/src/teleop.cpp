#include <teleop/teleop.h>
#include <teleop/ps3joy.h>
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>

namespace teleop
{

  Teleop::Teleop(ros::NodeHandle nh) : 
    nh_(nh),
    timer_(nh.createTimer(ros::Duration(0.1), &Teleop::publishCommands, this)),
    ackermann_pub_(nh.advertise<ackermann_msgs::AckermannDrive>(
          "commands/drivetrain/ackermann", 2)),
    joy_sub_(nh.subscribe<sensor_msgs::Joy>("/joy", 20, &Teleop::joyCallback, this))
    {}


  void Teleop::publishCommands(const ros::TimerEvent&) {
    // create message 
    ackermann_msgs::AckermannDrive::Ptr msg_out(new ackermann_msgs::AckermannDrive);
    msg_out->steering_angle = steering_angle_;
    msg_out->steering_angle_velocity = steering_angle_velocity_; // change steering angle velocity as quickly as possible
    msg_out->acceleration = acceleration_; // change velocity as quickly as possible
    msg_out->jerk = jerk_; // change acceleration as quickly as possible 
    msg_out->speed = speed_;

    ackermann_pub_.publish(msg_out);
  }
  void Teleop::joyCallback(const sensor_msgs::Joy::ConstPtr &msg) {
    
    steering_angle_ = msg->axes[ps3joy::PS3_AXIS_STICK_RIGHT_LEFTWARDS];
  
    // TODO: is -1 a magic number?
    if (msg->buttons[ps3joy::PS3_BUTTON_REAR_RIGHT_1] == 1) {
      // deadman switch depressed  
      speed_ = msg->axes[ps3joy::PS3_AXIS_STICK_LEFT_UPWARDS]
                       * max_speed_;
    } else {
      // default to stopped
      speed_ = 0;
    }

    ackermann_msgs::AckermannDrive::Ptr msg_out(new ackermann_msgs::AckermannDrive);
    msg_out->steering_angle = steering_angle_;
    msg_out->steering_angle_velocity = steering_angle_velocity_; // change steering angle velocity as quickly as possible
    msg_out->acceleration = acceleration_; // change velocity as quickly as possible
    msg_out->jerk = jerk_; // change acceleration as quickly as possible 
    msg_out->speed = speed_;

    ackermann_pub_.publish(msg_out);
  }

} // end namespace teleop
