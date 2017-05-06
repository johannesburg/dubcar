#include <ros/ros.h>
#include <teleop/teleop.h>
#include <iostream>
#include <teleop/ps3joy.h>

int main(int argc, char *argv[]) {
  std::cout << "hello world" << std::endl;  
  ros::init(argc, argv, "teleop_node");

  ros::NodeHandle nh;
  teleop::Teleop node(nh);
  ros::spin();
  return 0;
}
