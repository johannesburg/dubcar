#ifndef VESC_DRIVER_VESC_DRIVER_H
#define VESC_DRIVER_VESC_DRIVER_H
#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"
#include <string>


namespace vesc_driver
{
class VescDriver 
{
  public:
    VescDriver(const std::string& port);
  private: 
    VescInterface vesc_;
};
} // end vesc_driver namespace
#endif
