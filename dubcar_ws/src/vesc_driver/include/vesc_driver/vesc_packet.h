#include "vesc_driver/vesc_interface.h"

namespace vesc_driver
{

VescPacket::base_packet_size = 6; 

VescPacket::VescPacket(string name, VescPayload payload) :
  name_(name)
{
  Buffer = Buffer
  byte_t start_byte = static_cast<byte_t>(((length_byt > 255) ? 3 : 2));

