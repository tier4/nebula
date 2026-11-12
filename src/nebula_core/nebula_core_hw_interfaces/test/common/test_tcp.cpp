// Copyright 2026 TIER IV, Inc.

#include "nebula_core_hw_interfaces/connections/tcp.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace nebula::drivers::connections
{

using std::chrono_literals::operator""ms;

static const char * const g_localhost_ip = "127.0.0.1";
static const uint16_t g_server_port = 8080;

class TcpEchoThenCloseServer
{
public:
  explicit TcpEchoThenCloseServer(uint16_t port) : fd_(socket(AF_INET, SOCK_STREAM, 0))
  {
    if (fd_ == -1) throw std::runtime_error("Failed to create server socket");

    int opt = 1;
    if (setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) == -1) {
      throw std::runtime_error("Failed to set socket options");
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(fd_, (struct sockaddr *)&address, sizeof(address)) < 0) {
      throw std::runtime_error("Bind failed");
    }

    if (listen(fd_, 3) == -1) {
      throw std::runtime_error("Listen failed");
    }

    thread_ = std::thread([this]() {
      while (running_) {
        sockaddr_in client_addr{};
        int addrlen = sizeof(client_addr);
        int new_socket = accept(
          fd_, reinterpret_cast<struct sockaddr *>(&client_addr),
          reinterpret_cast<socklen_t *>(&addrlen));
        if (new_socket == -1) {
          if (running_)
            continue;  // Accept failed, maybe timeout or closed
          else
            break;
        }

        // Simple echo server
        char buffer[1024] = {0};
        ssize_t valread = read(new_socket, buffer, 1024);
        if (valread > 0) {
          send(new_socket, buffer, valread, MSG_NOSIGNAL);
        }
        ::close(new_socket);
      }
    });
  }

  TcpEchoThenCloseServer(const TcpEchoThenCloseServer &) = delete;
  TcpEchoThenCloseServer(TcpEchoThenCloseServer &&) = delete;
  TcpEchoThenCloseServer & operator=(const TcpEchoThenCloseServer &) = delete;
  TcpEchoThenCloseServer & operator=(TcpEchoThenCloseServer &&) = delete;

  ~TcpEchoThenCloseServer()
  {
    running_ = false;
    shutdown(fd_, SHUT_RDWR);
    ::close(fd_);
    if (thread_.joinable()) thread_.join();
  }

private:
  int fd_;
  std::atomic_bool running_{true};
  std::thread thread_;
};

TEST(TestTcp, TestBasicLifecycle)
{
  TcpEchoThenCloseServer server(g_server_port);
  ASSERT_NO_THROW(TcpSocket::Builder(g_localhost_ip, g_server_port).connect());
}

TEST(TestTcp, TestSendReceive)
{
  TcpEchoThenCloseServer server(g_server_port);
  std::vector<uint8_t> payload{1, 2, 3, 4};

  auto sock = TcpSocket::Builder(g_localhost_ip, g_server_port).connect();
  sock.send(payload);
  auto received = sock.receive(4);
  ASSERT_EQ(received.size(), 4);
  EXPECT_EQ(received, payload);
}

TEST(TestTcp, TestReceiveTimeout)
{
  TcpEchoThenCloseServer server(g_server_port);
  auto sock = TcpSocket::Builder(g_localhost_ip, g_server_port).connect();

  // Expect empty return on timeout (100ms) with no data sent
  auto received = sock.receive(4, 100ms);
  ASSERT_TRUE(received.empty());
}

TEST(TestTcp, TestReceiveOnClosedSocket)
{
  TcpEchoThenCloseServer server(g_server_port);
  auto sock = TcpSocket::Builder(g_localhost_ip, g_server_port).connect();

  std::vector<uint8_t> payload{1, 2, 3, 4};
  sock.send(payload);

  // Receive the echo
  sock.receive(4);

  // Server implementation closes connection after echo.
  // Next receive should throw "Connection closed"
  // Wait a bit to ensure close propagates (though it's on localhost)
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  EXPECT_THROW(
    {
      try {
        sock.receive(1);
      } catch (const SocketError & e) {
        EXPECT_STREQ(e.what(), "Connection closed");
        throw;
      }
    },
    SocketError);
}

TEST(TestTcp, TestReopen)
{
  TcpEchoThenCloseServer server(g_server_port);
  {
    auto sock = TcpSocket::Builder(g_localhost_ip, g_server_port).connect();
    // Socket is guaranteed open if exception not thrown
  }  // sock destroyed here
  {
    auto sock = TcpSocket::Builder(g_localhost_ip, g_server_port).connect();
  }
}

TEST(TestTcp, TestBuilderSetSocketBufferSize)
{
  TcpEchoThenCloseServer server(g_server_port);
  ASSERT_NO_THROW(
    TcpSocket::Builder(g_localhost_ip, g_server_port).set_socket_buffer_size(8192).connect());
}

TEST(TestTcp, TestBuilderSetPollingInterval)
{
  TcpEchoThenCloseServer server(g_server_port);
  ASSERT_NO_THROW(
    TcpSocket::Builder(g_localhost_ip, g_server_port).set_polling_interval(50).connect());
}

TEST(TestTcp, TestConnectionFailure)
{
  // Port 9999 should not have a listening server
  ASSERT_THROW(TcpSocket::Builder(g_localhost_ip, 9999).connect(), SocketError);
}

TEST(TestTcp, TestBuilderInvalidIp)
{
  ASSERT_THROW(TcpSocket::Builder("invalid_ip", g_server_port), UsageError);
  ASSERT_THROW(TcpSocket::Builder("256.0.0.1", g_server_port), UsageError);
}

TEST(TestTcp, TestConnectTimeout)
{
  // 192.0.2.1 is part of TEST-NET-1 (RFC 5737), which is non-routable.
  // Connection should timeout rapidly according to our setting, not system default.
  auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(
    {
      try {
        TcpSocket::Builder("192.0.2.1", 80)
          .set_connect_timeout(200)  // 200 ms timeout
          .connect();
      } catch (const SocketError & e) {
        EXPECT_STREQ(e.what(), "Connection timeout");
        throw;
      }
    },
    SocketError);
  auto duration = std::chrono::steady_clock::now() - start;
  // It shouldn't take more than ~500ms if the 200ms timeout works natively.
  EXPECT_LT(duration, std::chrono::milliseconds(500));
}

TEST(TestTcp, TestSetConnectTimeout)
{
  TcpEchoThenCloseServer server(g_server_port);
  ASSERT_NO_THROW(
    TcpSocket::Builder(g_localhost_ip, g_server_port).set_connect_timeout(500).connect());
}

TEST(TestTcp, TestSetBufferSize)
{
  TcpEchoThenCloseServer server(g_server_port);
  ASSERT_NO_THROW(
    TcpSocket::Builder(g_localhost_ip, g_server_port).set_socket_buffer_size(8192).connect());
}

class TcpEchoLoopServer
{
public:
  explicit TcpEchoLoopServer(uint16_t port) : fd_(socket(AF_INET, SOCK_STREAM, 0))
  {
    if (fd_ == -1) throw std::runtime_error("Failed to create server socket");

    int opt = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    int result = bind(fd_, (struct sockaddr *)&address, sizeof(address));
    if (result == -1) throw std::runtime_error("Failed to bind server socket");

    listen(fd_, 3);

    thread_ = std::thread([this]() {
      while (running_) {
        sockaddr_in client_addr{};
        int addrlen = sizeof(client_addr);
        int new_socket = accept(
          fd_, reinterpret_cast<struct sockaddr *>(&client_addr),
          reinterpret_cast<socklen_t *>(&addrlen));
        if (new_socket < 0) {
          if (running_)
            continue;
          else
            break;
        }

        char buffer[8192] = {0};
        while (running_) {
          ssize_t valread = read(new_socket, buffer, 8192);
          if (valread > 0) {
            send(new_socket, buffer, valread, MSG_NOSIGNAL);
          } else {
            break;
          }
        }
        ::close(new_socket);
      }
    });
  }

  TcpEchoLoopServer(const TcpEchoLoopServer &) = delete;
  TcpEchoLoopServer(TcpEchoLoopServer &&) = delete;
  TcpEchoLoopServer & operator=(const TcpEchoLoopServer &) = delete;
  TcpEchoLoopServer & operator=(TcpEchoLoopServer &&) = delete;

  ~TcpEchoLoopServer()
  {
    running_ = false;
    shutdown(fd_, SHUT_RDWR);
    ::close(fd_);
    if (thread_.joinable()) thread_.join();
  }

private:
  int fd_;
  std::atomic_bool running_{true};
  std::thread thread_;
};

TEST(TestTcp, TestLargePayloadSendReceive)
{
  TcpEchoLoopServer server(g_server_port);

  // Large payload to ensure it exceeds default MTU / buffers, triggering partial sends & multiple
  // receives.
  const size_t large_size = 65536;  // 64 KB
  std::vector<uint8_t> payload(large_size, 0xAA);
  for (size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<uint8_t>(i % 256);
  }

  std::vector<uint8_t> received_data;
  received_data.reserve(payload.size());

  auto sock =
    TcpSocket::Builder(g_localhost_ip, g_server_port).set_socket_buffer_size(32768).connect();

  sock.send(payload);

  for (int i = 0; i < 100 && received_data.size() < payload.size(); ++i) {  // Wait up to 5s
    auto chunk = sock.receive(8192, 50ms);
    if (chunk.empty()) continue;
    received_data.insert(received_data.end(), chunk.begin(), chunk.end());
  }

  ASSERT_EQ(received_data.size(), payload.size());
  EXPECT_TRUE(std::equal(received_data.begin(), received_data.end(), payload.begin()));
}

}  // namespace nebula::drivers::connections

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
