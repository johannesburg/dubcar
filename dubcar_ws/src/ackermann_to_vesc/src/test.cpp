
#include <ros/ros.h>
#include <ackermann_to_vesc/ackermann_to_vesc.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ackermann_to_vesc_node");

  ros::NodeHandle nh;
  ackermann_to_vesc::AckermannToVesc node(nh);
  ros::spin();
  return 0;
}
