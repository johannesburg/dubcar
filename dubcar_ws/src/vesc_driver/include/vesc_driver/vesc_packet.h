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
    static const uint8_t LOWER_BYTE_MASK;
    static VescPacket createDutyCycleCmd(float duty_cycle);
    static VescPacket createCurrentCmd(float current); 
    static VescPacket createCurrentBrakeCmd(float brake);
    static VescPacket createRpmCmd(int32_t rpm); 
    static VescPacket createPositionCmd(float position);
    static VescPacket createServoPositionCmd(float servo);
    static VescPacket createRebootCmd(); 
    static VescPacket createSendAliveCmd();

    VescPacket(const std::string &name, const Buffer &payload);
    const Buffer getBuffer() const;
    const std::string getName() const;
    typedef boost::crc_optimal<16, 0x1021, 0, 0, false, false> crc_32_t;

  private:
    std::string name_;
    Buffer buf_;
    static void append(Buffer& buff, float number, float scale);
    static void append(Buffer& buff, int32_t number);
    static void append(Buffer& buff, int16_t number);

};


}
#endif

