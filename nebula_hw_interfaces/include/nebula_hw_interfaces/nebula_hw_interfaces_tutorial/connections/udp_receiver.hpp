#pragma once

#include "nebula_hw_interfaces/nebula_hw_interfaces_tutorial/loggers/logger.hpp"

#include <boost_udp_driver/udp_driver.hpp>

#include <cstdint>
#include <functional>

namespace nebula
{
namespace drivers
{
namespace connections
{

class UdpReceiver
{
public:
  using callback_t = typename std::function<void(std::vector<uint8_t> & buffer)>;

  /// @brief Create a receiving UDP connection and forward received packets to `packet_callback`
  /// @param packet_callback The function getting called on each received packet
  UdpReceiver(
    std::shared_ptr<nebula::drivers::loggers::Logger> logger, const std::string & host_ip, uint16_t host_port,
    callback_t packet_callback)
  : logger_(logger), ctx_(1), udp_driver_(ctx_)
  {
    try {
      udp_driver_.init_receiver(host_ip, host_port);
      udp_driver_.receiver()->open();
      udp_driver_.receiver()->bind();

      udp_driver_.receiver()->asyncReceive(packet_callback);
    } catch (const std::exception & ex) {
      throw std::runtime_error(std::string("Could not open UDP socket: ") + ex.what());
    }
  }

private:
  std::shared_ptr<nebula::drivers::loggers::Logger> logger_;

  ::drivers::common::IoContext ctx_;
  ::drivers::udp_driver::UdpDriver udp_driver_;
};

}  // namespace connections
}  // namespace drivers
}  // namespace nebula
