#include "vesc_driver/vesc_packet.h"
#include "vesc_driver/datatypes.h"
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

VescPacket VescPacket::createDutyCycleCmd(float duty_cycle)
{
  Buffer payload;

  // Set signal
  payload.push_back(COMM_SET_DUTY); 
  append(payload, duty_cycle, 100000.0);
  return VescPacket("set_duty_cycle", payload);
}

VescPacket VescPacket::createCurrentCmd(float current) 
{
  Buffer payload;
  payload.push_back(COMM_SET_CURRENT); 
  append(payload, current, 1000.0);
  return VescPacket("set_current", payload);
}

VescPacket VescPacket::createCurrentBrakeCmd(float brake) 
{
  Buffer payload;
  payload.push_back(COMM_SET_CURRENT_BRAKE); 
  append(payload, brake, 1000.0);
  return VescPacket("set_current_brake", payload);
}

VescPacket VescPacket::createRpmCmd(int32_t rpm) 
{
  Buffer payload;
  payload.push_back(COMM_SET_RPM); 
  append(payload, rpm);
  return VescPacket("set_rpm", payload);
}

VescPacket VescPacket::createPositionCmd(float position) 
{
  Buffer payload;
  payload.push_back(COMM_SET_POS); 
  append(payload, position, 1000000.0);
  return VescPacket("set_position", payload);
}

VescPacket VescPacket::createServoPositionCmd(float servo) 
{
  Buffer payload;
  payload.push_back(COMM_SET_SERVO_POS); 
  append(payload, static_cast<int16_t>(servo * 1000.0));
  return VescPacket("set_position", payload);
}

VescPacket VescPacket::createRebootCmd() 
{
  Buffer payload;
  payload.push_back(COMM_REBOOT); 
  return VescPacket("reboot", payload);
}

VescPacket VescPacket::createSendAliveCmd() 
{
  Buffer payload;
  payload.push_back(COMM_ALIVE); 
  return VescPacket("alive", payload);
}

/*
 * Private helper methods.
 *
 */

void VescPacket::append(Buffer& buff, float number, float scale) 
{
  append(buff, static_cast<int32_t>(scale * number));
}

void VescPacket::append(Buffer& buff, int32_t number)
{
  buff.push_back(static_cast<uint8_t>((number >> 24) & LOWER_BYTE_MASK));
  buff.push_back(static_cast<uint8_t>((number >> 16) & LOWER_BYTE_MASK));
  buff.push_back(static_cast<uint8_t>((number >> 8) & LOWER_BYTE_MASK));
  buff.push_back(static_cast<uint8_t>(number & LOWER_BYTE_MASK));
}

void VescPacket::append(Buffer& buff, int16_t number)
{
  buff.push_back(static_cast<uint8_t>((number >> 8) & LOWER_BYTE_MASK));
  buff.push_back(static_cast<uint8_t>(number & LOWER_BYTE_MASK));
}

}
