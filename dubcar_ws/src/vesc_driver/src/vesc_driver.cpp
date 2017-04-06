#include "vesc_driver/vesc_driver.h"
#include "vesc_driver/vesc_packet.h"
#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/datatypes.h"
#include <boost/scoped_ptr.hpp>
#include <cstdio>
#include <string>
#include <unistd.h>

namespace vesc_driver 
{
  // Private inner class for util functions.
  class VescDriver::Impl
  {
    const uint8_t LOWER_BYTE_MASK = 0xFF;
    public:
      Impl() {} 
      // TODO: implement append methods for other types
      void append(Buffer& buff, float number, float scale) 
      {
        append(buff, static_cast<int32_t>(scale * number));
      }

      void append(Buffer& buff, int32_t number)
      {
        buff.push_back(static_cast<uint8_t>((number >> 24) & LOWER_BYTE_MASK));
        buff.push_back(static_cast<uint8_t>((number >> 16) & LOWER_BYTE_MASK));
        buff.push_back(static_cast<uint8_t>((number >> 8) & LOWER_BYTE_MASK));
        buff.push_back(static_cast<uint8_t>(number & LOWER_BYTE_MASK));
      }

      void append(Buffer& buff, int16_t number)
      {
        buff.push_back(static_cast<uint8_t>((number >> 8) & LOWER_BYTE_MASK));
        buff.push_back(static_cast<uint8_t>(number & LOWER_BYTE_MASK));
      }
  };

VescDriver::VescDriver(const std::string& port) : impl_(new Impl()), vesc_(port) {}

void VescDriver::setDutyCycle(float duty_cycle) 
{
  Buffer payload;

  // Set signal
  payload.push_back(COMM_SET_DUTY); 
  impl_->append(payload, duty_cycle, 100000.0);
  VescPacket packet = VescPacket("set_duty_cycle", payload);

  vesc_.send(packet);
}

void VescDriver::setCurrent(float current) 
{
  Buffer payload;
  payload.push_back(COMM_SET_CURRENT); 
  impl_->append(payload, current, 1000.0);
  VescPacket packet = VescPacket("set_current", payload);
  vesc_.send(packet);
}

void VescDriver::setCurrentBrake(float brake) 
{
  Buffer payload;
  payload.push_back(COMM_SET_CURRENT_BRAKE); 
  impl_->append(payload, brake, 1000.0);
  VescPacket packet = VescPacket("set_current_brake", payload);
  vesc_.send(packet);
}

void VescDriver::setRpm(int32_t rpm) 
{
  Buffer payload;
  payload.push_back(COMM_SET_RPM); 
  impl_->append(payload, rpm);
  VescPacket packet = VescPacket("set_rpm", payload);
  vesc_.send(packet);
}

void VescDriver::setPosition(float position) 
{
  Buffer payload;
  payload.push_back(COMM_SET_POS); 
  impl_->append(payload, position, 1000000.0);
  VescPacket packet = VescPacket("set_position", payload);
  vesc_.send(packet);
}

void VescDriver::setServoPosition(float servo) 
{
  Buffer payload;
  payload.push_back(COMM_SET_SERVO_POS); 
  impl_->append(payload, static_cast<int16_t>(servo * 1000.0));
  VescPacket packet = VescPacket("set_position", payload);
  vesc_.send(packet);
}

void VescDriver::reboot() 
{
  Buffer payload;
  payload.push_back(COMM_REBOOT); 
  VescPacket packet = VescPacket("reboot", payload);
  vesc_.send(packet);
}

void VescDriver::sendAlive() 
{
  Buffer payload;
  payload.push_back(COMM_ALIVE); 
  VescPacket packet = VescPacket("alive", payload);
  vesc_.send(packet);
}

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

