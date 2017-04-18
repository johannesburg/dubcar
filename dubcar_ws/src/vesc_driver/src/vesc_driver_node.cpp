#include <ros/ros.h>
#include "vesc_driver/vesc_driver.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "vesc_driver_node");

  ros::NodeHandle nh;
  //TODO: vesc_driver::VescDriver node(nh);

  ros::spin();
  return 0;
}
