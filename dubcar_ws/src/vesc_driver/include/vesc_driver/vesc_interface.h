#ifndef VESC_DRIVER_VESC_INTERFACE_H
#define VESC_DRIVER_VESC_INTERFACE_H

#include <string>
#include <boost/scoped_ptr.hpp>
#include "vesc_driver/vesc_packet.h"
namespace vesc_driver
{

// TODO: copyable?
class VescInterface
{
  public:
    VescInterface();
    VescInterface(const std::string& port);
    ~VescInterface();
    void send(const Buffer& payload);
    void send(const VescPacket& packet);
    bool isConnected() const;
    void disconnect();
    void connect(const std::string& port);

  private:
    class Impl;
    boost::scoped_ptr<Impl> impl_;      

}; // namespace vesc driver

}
#endif
