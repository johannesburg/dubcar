#include "vesc_driver/vesc_driver.h"
#include "vesc_driver/vesc_packet.h"
#include "vesc_driver/datatypes.h"

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
  }

VescDriver::VescDriver() : _vesc() {}

void VescDriver::setDutyCycle(float duty_cycle) 
{
  Buffer payload;

  // Set signal
  payload.push_back(COMM_SET_DUTY); 

  // Convert and append duty_cycle to  payload
  impl_->append(payload, duty_cycle, 100000.0);
  
  // Generate and send packet
  VescPacket packet = VescPacket("set_duty", payload);
  vesc_.send(packet);
}

}
