#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"

namespace vesc_driver
{
class VescDriver 
{
  public:
    VescDriver();
    void setDutyCycle(float duty_cycle) 
  private: 
    VescInterface _vesc;
    class Impl;
    boost::scoped_ptr<Impl> impl_;      
}
}
