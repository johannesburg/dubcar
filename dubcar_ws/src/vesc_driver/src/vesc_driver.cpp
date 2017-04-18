#include "vesc_driver/vesc_driver.h"
#include "vesc_driver/vesc_packet.h"
#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/datatypes.h"
#include <cstdio>
#include <string>
#include <unistd.h>

namespace vesc_driver 
{

VescDriver::VescDriver(const std::string& port) : vesc_(port) {}


} // end namespace vesc_driver

int main(int argc, char** argv) 
{
  vesc_driver::VescDriver vesc("/dev/ttyACM0");
  while(true) {
    vesc.setCurrent(1);
    usleep(1000);
  }
   
  return 0;
}

