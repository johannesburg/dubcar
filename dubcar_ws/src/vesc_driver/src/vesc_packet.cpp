#include "vesc_driver/vesc_interface.h"

namespace vesc_driver
{

VescPacket::BASE_PACKET_SIZE = static_cast<byte_t>(6); 
VescPacket::LARGE_PAYLOAD = static_cast<byte_t>(3);
VescPacket::SMALL_PAYLOAD = static_cast<byte_t>(2);
VescPacket::STOP_BYTE = static_cast<byte_t>(3);
VescPacket::LOWER_BYTE_MASK = 0xFF;

VescPacket::VescPacket(std::string name, Buffer payload) :
  name_(name), buf_(payload.size() + VescPacket::BASE_PACKET_SIZE)
{

  bool large_packet = payload.size() > 255;
  
  // append start byte (3 indicates a 2-byte length frame, 2 indicates a 1-byte length frame)
  this.buf_.push_back(large_packet ? LARGE_PAYLOAD : SMALL_PAYLOAD);
  
  // append length frame of either 1 or 2 bytes
  if (large_packet)
    this.buf_.push_back(payload.size() >> sizeof(byte_t));
  this.buf_.push_back(payload.size() & LOWER_BYTE_MASK);
  
  // append payload's Buffer onto this VescPacket's Buffer
  this.buf_.insert(std::end(this.buf_), 
      std::begin(payload.getBuffer()), std::end(payload.getBuffer()));

  uint16_t CRC_checksum = ???;
  
  // append CRC checksum
  this.buf_.push_back(static_cast<byte_t>(CRC_checksum >> sizeof(byte_t)));
  this.buf_.push_back(static_cast<byte_t>(CRC_checksum & LOWER_BYTE_MASK);

  // append stop bit
  this.buf_.push_back(STOP_BYTE);
}

// return unmodifiable view of this packet's Buffer
const Buffer VescPacket::getBuffer() const {
  return this.buf_;
}

// return unmodifiable view of this packet's name
const std::string VescPacket::getName() const {
  return this.name_;
}

}
