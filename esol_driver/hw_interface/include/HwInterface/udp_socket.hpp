#ifndef HWINTERFACE_UDP_SOCKET_HPP_
#define HWINTERFACE_UDP_SOCKET_HPP_

#include <netinet/in.h>

#include <array>
#include <string>
#include <vector>

namespace HwInterface
{
class UdpSocket
{
public:
  explicit UdpSocket(const std::string & sensor_ip = "", uint16_t sensor_port = 0);
  ~UdpSocket();

  UdpSocket(const UdpSocket &) = delete;
  UdpSocket & operator=(const UdpSocket &) = delete;

  int Open(uint16_t my_port, bool nonblock = true, bool reuse_port = false);
  void Close();

  bool IsOpen() const;

  int Send(const std::vector<uint8_t> & buff);
  int Recv(std::vector<uint8_t> & buff, int wait_msec = -1);

  std::string EndPointIp() const;
  uint16_t EndPointPort() const;

  void SetSensorIpPort(const std::string & sensor_ip = "", uint16_t sensor_port = 0);
  int Errcd() const;

private:
  void ErrorClose(int errcd = 0);

private:
  int sockfd_;
  int errcd_;
  sockaddr_in sensor_addrin_{};
  sockaddr_in rcv_addrin_{};
};
}  // namespace HwInterface
#endif  // HWINTERFACE_UDP_SOCKET_HPP_
