// Copyright 2025 TIER IV, Inc.

#include "nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/tcp.hpp"

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

class TcpServer
{
public:
  explicit TcpServer(uint16_t port)
  {
    fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (fd_ == -1) throw std::runtime_error("Failed to create server socket");

    int opt = 1;
    setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(fd_, (struct sockaddr *)&address, sizeof(address)) < 0) {
      throw std::runtime_error("Bind failed");
    }

    if (listen(fd_, 3) < 0) {
      throw std::runtime_error("Listen failed");
    }

    running_ = true;
    thread_ = std::thread([this]() {
      while (running_) {
        sockaddr_in client_addr{};
        int addrlen = sizeof(client_addr);
        int new_socket = accept(fd_, (struct sockaddr *)&client_addr, (socklen_t *)&addrlen);
        if (new_socket < 0) {
          if (running_)
            continue;  // Accept failed, maybe timeout or closed
          else
            break;
        }

        // Simple echo server
        char buffer[1024] = {0};
        ssize_t valread = read(new_socket, buffer, 1024);
        if (valread > 0) {
          send(new_socket, buffer, valread, 0);
        }
        ::close(new_socket);
      }
    });
  }

  ~TcpServer()
  {
    running_ = false;
    shutdown(fd_, SHUT_RDWR);
    ::close(fd_);
    if (thread_.joinable()) thread_.join();
  }

private:
  int fd_;
  std::atomic_bool running_;
  std::thread thread_;
};

TEST(TestTcp, TestBasicLifecycle)
{
  TcpServer server(g_server_port);
  ASSERT_NO_THROW(
    TcpSocket::Builder(g_localhost_ip, g_server_port)
      .connect()
      .subscribe([](const auto &) {})
      .unsubscribe());
}

TEST(TestTcp, TestSendReceive)
{
  TcpServer server(g_server_port);
  std::vector<uint8_t> payload{1, 2, 3, 4};
  std::atomic_bool received{false};

  auto sock = TcpSocket::Builder(g_localhost_ip, g_server_port).connect();
  sock.subscribe([&](const std::vector<uint8_t> & data) {
    if (data == payload) received = true;
  });

  sock.send(payload);

  // Wait for echo
  for (int i = 0; i < 20; ++i) {
    if (received) break;
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_TRUE(received);
}

TEST(TestTcp, TestBlockingReceive)
{
  TcpServer server(g_server_port);
  std::vector<uint8_t> payload{0xAA, 0xBB, 0xCC, 0xDD};

  auto sock = TcpSocket::Builder(g_localhost_ip, g_server_port).connect();

  // Send data to trigger echo
  sock.send(payload);

  // Blocking receive
  auto received = sock.receive(4);
  ASSERT_EQ(received.size(), 4);
  ASSERT_EQ(received, payload);
}

TEST(TestTcp, TestReopen)
{
  TcpServer server(g_server_port);
  auto sock = TcpSocket::Builder(g_localhost_ip, g_server_port).connect();
  ASSERT_TRUE(sock.isOpen());
  sock.close();
  ASSERT_FALSE(sock.isOpen());
  sock.open(g_localhost_ip, g_server_port);
  ASSERT_TRUE(sock.isOpen());
}

TEST(TestTcp, TestBuilderSetSocketBufferSize)
{
  TcpServer server(g_server_port);
  ASSERT_NO_THROW(
    TcpSocket::Builder(g_localhost_ip, g_server_port).set_socket_buffer_size(8192).connect());
}

TEST(TestTcp, TestBuilderSetPollingInterval)
{
  TcpServer server(g_server_port);
  ASSERT_NO_THROW(
    TcpSocket::Builder(g_localhost_ip, g_server_port).set_polling_interval(50).connect());
}

TEST(TestTcp, TestConnectionFailure)
{
  // Port 9999 should not have a listening server
  ASSERT_THROW(TcpSocket::Builder(g_localhost_ip, 9999).connect(), SocketError);
}

TEST(TestTcp, TestIsSubscribed)
{
  TcpServer server(g_server_port);
  auto sock = TcpSocket::Builder(g_localhost_ip, g_server_port).connect();
  ASSERT_FALSE(sock.is_subscribed());
  sock.subscribe([](const auto &) {});
  ASSERT_TRUE(sock.is_subscribed());
  sock.unsubscribe();
  ASSERT_FALSE(sock.is_subscribed());
}

TEST(TestTcp, TestMoveSemantics)
{
  TcpServer server(g_server_port);
  auto sock1 = TcpSocket::Builder(g_localhost_ip, g_server_port).connect();
  sock1.subscribe([](const auto &) {});
  ASSERT_TRUE(sock1.is_subscribed());

  TcpSocket sock2(std::move(sock1));
  ASSERT_TRUE(sock2.is_subscribed());
  ASSERT_TRUE(sock2.isOpen());
}

TEST(TestTcp, TestOpenWithEndpoint)
{
  TcpServer server(g_server_port);
  Endpoint endpoint{parse_ip(g_localhost_ip).value_or_throw(), g_server_port};

  TcpSocket sock;
  ASSERT_FALSE(sock.isOpen());
  sock.open(endpoint);
  ASSERT_TRUE(sock.isOpen());
}

}  // namespace nebula::drivers::connections

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
