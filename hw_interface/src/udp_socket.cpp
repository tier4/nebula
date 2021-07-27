#include "HwInterface/udp_socket.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace HwInterface
{
/// @brief constructor
/// @param sensor_ip Device IP address
/// @param sensor_port Device UDP port number
UdpSocket::UdpSocket(const std::string & sensor_ip, uint16_t sensor_port) : sockfd_(-1), errcd_(0)
{
  SetSensorIpPort(sensor_ip, sensor_port);
}
/// @brief destructor
UdpSocket::~UdpSocket() { Close(); }

/// @brief Open is socket and bind.
/// @param sensor_ip   : Device IP address
/// @param sensor_port : Device UDP port number
/// @retval  0: success
/// @retval -1: socket error
/// @retval -2: reuser port error
/// @retval -3: nonblock settig error
/// @retval -4: bind error
int UdpSocket::Open(uint16_t my_port, bool nonblock, bool reuse_port)
{
  int retval;

  sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
  if (sockfd_ < 0) {
    errcd_ = errno;
    return -1;
  }

  rcv_addrin_.sin_port = 0;
  rcv_addrin_.sin_addr.s_addr = 0;

  if (reuse_port) {
    int on = -1;
    retval =
      setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, static_cast<const void *>(&on), sizeof(on));
    if (retval != 0) {
      ErrorClose(errno);
      return -2;
    }
  }

  if (nonblock) {
    retval = ::fcntl(sockfd_, F_SETFL, O_NONBLOCK | FASYNC);
    if (retval != 0) {
      ErrorClose(errno);
      return -3;
    }
  }

  sockaddr_in my_addrin{};
  my_addrin.sin_family = AF_INET;
  my_addrin.sin_port = htons(my_port);
  my_addrin.sin_addr.s_addr = INADDR_ANY;

  retval = ::bind(sockfd_, reinterpret_cast<sockaddr *>(&my_addrin), sizeof(my_addrin));
  if (retval != 0) {
    ErrorClose(errno);
    return -4;
  }

  return 0;
}

/// @brief socket close
void UdpSocket::Close()
{
  if (sockfd_ >= 0) {
    close(sockfd_);
    sockfd_ = -1;
  }
}

/// @brief error code set and socket close
void UdpSocket::ErrorClose(int errcd)
{
  if (errcd != 0) {
    errcd_ = errcd;
  }
  Close();
}

/// @brief set Sensor IP address and UDP port number.
void UdpSocket::SetSensorIpPort(const std::string & sensor_ip, uint16_t sensor_port)
{
  sensor_addrin_.sin_family = AF_INET;
  sensor_addrin_.sin_port = htons(sensor_port);
  if (sensor_ip.empty()) {
    sensor_addrin_.sin_addr.s_addr = 0;
  } else {
    sensor_addrin_.sin_addr.s_addr = inet_addr(sensor_ip.c_str());
  }
}

/// @brief error code
int UdpSocket::Errcd() const { return errcd_; }

/// @brief Whether the underlying connection is open.
bool UdpSocket::IsOpen() const { return (sockfd_ >= 0) ? true : false; }

/// @brief recieve Source IP address
std::string UdpSocket::EndPointIp() const
{
  std::vector<char> buf(INET_ADDRSTRLEN);
  if (rcv_addrin_.sin_addr.s_addr != 0) {
    inet_ntop(AF_INET, &(rcv_addrin_.sin_addr), &buf[0], buf.size());
  }
  return std::string(buf.data());
}

/// @brief recieve Source UDP port number.
uint16_t UdpSocket::EndPointPort() const { return ntohs(rcv_addrin_.sin_port); }

/// @brief send data.
/// @param buff : send data
/// @return On success, these calls return the number of bytes sent.
/// @retval -1: sendto error
/// @retval -2: port number not set.
int UdpSocket::Send(std::vector<uint8_t> & buff)
{
  int retval;

  if (sensor_addrin_.sin_port == 0) {
    return -2;
  }

  retval = sendto(
    sockfd_, static_cast<const void *>(buff.data()), buff.size(), 0,
    reinterpret_cast<sockaddr *>(&sensor_addrin_), sizeof(sensor_addrin_));

  return retval;
}

/// @brief receive data.
/// @param buff : receive buffer.
/// @param wait_msec : wait mili second. < 0: Permanent wait.
/// @return These calls return the number of bytes received.
/// @retval -1: poll error
/// @retval -2: device error  (POLLERR | POLLHUP | POLLNVAL)
/// @retval -3: recvfrom error
int UdpSocket::Recv(std::vector<uint8_t> & buff, int wait_msec)
{
  int poll_timeout = wait_msec >= 0 ? wait_msec : 1000;  // Permanent wait is a 1 second loop
  int nbytes = 0;
  struct pollfd fds[1];
  fds[0].fd = sockfd_;
  fds[0].events = POLLIN;

  do {
	// wait for some event on a file descriptor
    int retval = poll(fds, sizeof(fds) / sizeof(fds[0]), poll_timeout);
    if (retval < 0) {
      if (errno != EINTR) {
        errcd_ = errno;
      }
      return -1;
    } else if (retval == 0) {
      // If it is not a permanent wait, the timeout ends.
      if (wait_msec >= 0) {
        break;
      }
    } else if ((fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      return -2;  // device error?
    } else if ((fds[0].revents & POLLIN) == 0) {
      //continue;
    } else {
      socklen_t addr_size = sizeof(rcv_addrin_);
      retval = recvfrom(
        fds[0].fd, static_cast<uint8_t *>(buff.data()), buff.size(), 0,
        reinterpret_cast<sockaddr *>(&rcv_addrin_), &addr_size);
      if (retval < 0) {
        if (errno != EWOULDBLOCK) {
          errcd_ = errno;
          return -3;
        }
      } else if (
        (sensor_addrin_.sin_addr.s_addr != 0) &&
        (sensor_addrin_.sin_addr.s_addr != rcv_addrin_.sin_addr.s_addr)) {
        // If the target device is specified,
        // reception from other than the target device will be discarded.
        //continue;
      } else {
        nbytes = retval;
      }
    }
  } while (nbytes == 0);

  return nbytes;
}
}  // namespace HwInterface
