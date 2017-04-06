#include "vesc_driver/vesc_packet.h"
#include <iostream>
#include <iomanip>
#include <boost/crc.hpp>

namespace vesc_driver
{

const byte_t VescPacket::BASE_PACKET_SIZE = static_cast<byte_t>(5); 
const byte_t VescPacket::LARGE_PAYLOAD = static_cast<byte_t>(3);
const byte_t VescPacket::SMALL_PAYLOAD = static_cast<byte_t>(2);
const byte_t VescPacket::STOP_BYTE = static_cast<byte_t>(3);
const uint8_t VescPacket::LOWER_BYTE_MASK = 0xFF;

VescPacket::VescPacket(const std::string &name, const Buffer &payload) :
  name_(name) 
{
  bool large_packet = payload.size() > 255;
  this->buf_.reserve(BASE_PACKET_SIZE + (large_packet ? 1 : 0) + payload.size());
  // append start byte (3 indicates a 2-byte length frame, 2 indicates a 1-byte length frame)
  this->buf_.push_back(large_packet ? LARGE_PAYLOAD : SMALL_PAYLOAD);
  
  // append length frame of either 1 or 2 bytes
  if (large_packet)
    this->buf_.push_back(payload.size() >> 8);
  this->buf_.push_back(payload.size() & LOWER_BYTE_MASK);
  
  // append payload's Buffer onto this->VescPacket's Buffer
  this->buf_.insert(std::end(this->buf_), 
      std::begin(payload), std::end(payload));

  // calculate crc checksum
  crc_32_t crc;
  crc.process_bytes(&(*payload.begin()), payload.size()); 
  uint16_t CRC_checksum = crc.checksum(); 

  // append CRC checksum
  this->buf_.push_back(static_cast<byte_t>((CRC_checksum >> 8) & LOWER_BYTE_MASK)); 
  this->buf_.push_back(static_cast<byte_t>(CRC_checksum & LOWER_BYTE_MASK));

  // append stop bit
  this->buf_.push_back(STOP_BYTE);
  std::cout << "Buffer size: " << this->buf_.size() << std::endl;
  for (int i = 0; i < this->buf_.size(); i++) {
    std::cout << std::hex << std::setfill('0') << std::setw(2) << ((int) *(this->buf_.data() + i));
  }
  std::cout << std::endl;
}

// return unmodifiable view of this->packet's Buffer
const Buffer VescPacket::getBuffer() const {
  return this->buf_;
}

// return unmodifiable view of this->packet's name
const std::string VescPacket::getName() const {
  return this->name_;
}

}
