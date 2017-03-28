#ifndef VESC_DRIVER_VESC_PACKET_H
#define VESC_DRIVER_VESC_PACKET_H

#include <vector>
#include <string>

typedef uint8_t byte_t;
typedef std::vector<byte_t> Buffer;

namespace vesc_driver
{

class VescPacket 
{
  static const int base_packet_size;
  public:
    VescPacket(string name, VescPayload payload);
    Buffer getBuffer();

  private:
    string name_;
    VescPayload payload_;
    Buffer buf_;
};

}
