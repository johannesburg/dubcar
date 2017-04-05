#ifndef VESC_DRIVER_VESC_PACKET_H
#define VESC_DRIVER_VESC_PACKET_H

#include <vector>
#include <string>
#include <boost/crc.hpp>

typedef uint8_t byte_t;
typedef std::vector<byte_t> Buffer;

namespace vesc_driver
{

class VescPacket 
{
  public:
    static const byte_t BASE_PACKET_SIZE;
    static const byte_t LARGE_PAYLOAD;
    static const byte_t SMALL_PAYLOAD;
    static const byte_t STOP_BYTE;
    static const uint16_t LOWER_BYTE_MASK;

    VescPacket(const std::string &name, const Buffer &payload);
    const Buffer getBuffer() const;
    const std::string getName() const;
    typedef boost::crc_optimal<16, 0x1021, 0, 0, false, false> crc_32_t;

  private:
    std::string name_;
    Buffer buf_;

};

}
#endif

