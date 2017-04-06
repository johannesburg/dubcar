#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"
#include <boost/scoped_ptr.hpp>
#include <string>


namespace vesc_driver
{
class VescDriver 
{
  public:
    VescDriver(const std::string& port);
    void setDutyCycle(float duty_cycle);
    void setCurrent(float current);
    void setCurrentBrake(float brake);
    void setRpm(int32_t rpm);
    void setPosition(float position);
    void setServoPosition(float servo);
    void reboot();
    void sendAlive();
  private: 
    VescInterface vesc_;
    class Impl;
    boost::scoped_ptr<Impl> impl_;      
};
}
