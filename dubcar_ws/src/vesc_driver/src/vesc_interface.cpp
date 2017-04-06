#include "vesc_driver/vesc_interface.h"
#include "vesc_driver/vesc_packet.h"

#include <vector>
#include <exception>
#include <string>
#include <serial/serial.h>
#include <iostream>
namespace vesc_driver
{
 

class VescInterface::Impl {
  static const uint32_t BAUD_RATE = 115200;
  public: 
    Impl() :
      serial_(std::string(), BAUD_RATE, serial::Timeout::simpleTimeout(1000), 
          serial::eightbits, serial::parity_none, serial::stopbits_one, 
          serial::flowcontrol_none)
  {}

  serial::Serial serial_;
};

// TODO: instert arguments, such as port, packet_handler, error_handler
VescInterface::VescInterface(const std::string& port) : impl_(new Impl()) 
{
  if (!port.empty()) {
    connect(port);
  }
}

// TODO:: disconnect
VescInterface::~VescInterface() {}


// TODO: specify arguments, input will be a vesc_packet type
void VescInterface::send(Buffer packet) 
{
  std::cout << "Outputting packet: ";
  for (auto i: packet)
    std::cout << i << ' ' ;
  std::cout << std::endl;
  // TODO: insert packet to send
  
  size_t written = this->impl_->serial_.write(packet);
  if (written != packet.size()) {
    std::stringstream ss;
    ss << "Wrote " << written << " bytes, expected " << packet.size() << ".";
    throw serial::SerialException(ss.str().c_str());
  }
}

void VescInterface::send(VescPacket packet)
{
  this->send(packet.getBuffer());
}

bool VescInterface::isConnected() const 
{
  return !this->impl_->serial_.isOpen();
}

void VescInterface::connect(const std::string& port) 
{
  if (!this->isConnected()) 
  {
    std::stringstream ss;
    ss << "Already connected to port: " << this->impl_->serial_.getPort() << ".";
    throw serial::SerialException(ss.str().c_str());
  }

  try 
  {
    this->impl_->serial_.setPort(port);
    this->impl_->serial_.open();
  } 
  catch (const std::exception& e)
  {
    // TODO: what exception type to throw?
    throw;
  }
}

} // end namespace vesc_driver
