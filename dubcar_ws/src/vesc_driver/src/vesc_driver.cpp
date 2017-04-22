#include "vesc_driver/vesc_driver.h"
#include <string>

namespace vesc_driver 
{

VescDriver::VescDriver(const std::string& port) : vesc_(port) {}


} // end namespace vesc_driver
