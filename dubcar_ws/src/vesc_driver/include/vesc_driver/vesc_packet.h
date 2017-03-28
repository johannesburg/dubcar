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
  static const byte_t BASE_PACKET_SIZE;
  static const byte_t LARGE_PAYLOAD;
  static const byte_t SMALL_PAYLOAD;
  static const byte_t STOP_BYTE;
  static const uint16_t LOWER_BYTE_MASK;

  public:
    VescPacket(std::string name, Buffer payload);
    const Buffer getBuffer() const;
    const std::string getName() const;

  private:
    std::string name_;
    Buffer buf_;
};

}
#endif

