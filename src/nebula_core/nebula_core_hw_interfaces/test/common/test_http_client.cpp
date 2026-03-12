// Copyright 2025 TIER IV, Inc.

#include "nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/http_client.hpp"

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace nebula::drivers::connections
{

using std::chrono_literals::operator""ms;

static const char * const g_localhost_ip = "127.0.0.1";
static const uint16_t g_server_port = 8081;

class HttpServer
{
public:
  explicit HttpServer(uint16_t port)
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
        int new_socket = accept(
          fd_, reinterpret_cast<struct sockaddr *>(&client_addr),
          reinterpret_cast<socklen_t *>(&addrlen));
        if (new_socket < 0) {
          if (running_)
            continue;
          else
            break;
        }

        char buffer[1024] = {0};
        ssize_t valread = read(new_socket, buffer, 1024);
        if (valread > 0) {
          std::string request(buffer, valread);
          std::string response;
          if (request.find("GET") == 0) {
            if (request.find("/chunked") != std::string::npos) {
              response = "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n";
              response += "5\r\nHello\r\n";
              response += "1\r\n \r\n";
              response += "6\r\nWorld!\r\n";
              response += "0\r\n\r\n";
            } else if (request.find("/slow") != std::string::npos) {
              std::this_thread::sleep_for(std::chrono::milliseconds(200));
              response = "HTTP/1.1 200 OK\r\nContent-Length: 4\r\n\r\nSlow";
            } else if (request.find("/large") != std::string::npos) {
              response = "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n";
              std::string large_chunk(5000, 'A');
              char len_buf[16];
              snprintf(len_buf, sizeof(len_buf), "%zx\r\n", large_chunk.length());
              response += len_buf;
              response += large_chunk;
              response += "\r\n";
              response += len_buf;
              response += large_chunk;
              response += "\r\n";
              response += "0\r\n\r\n";
            } else if (request.find("/error") != std::string::npos) {
              response = "HTTP/1.1 500 Internal Server Error\r\nContent-Length: 9\r\n\r\nErrorBody";
            } else if (request.find("/drop") != std::string::npos) {
              response = "HTTP/1.1 200 OK\r\nContent-Length: 100\r\n\r\nPartial";
              send(new_socket, response.c_str(), response.length(), 0);
              ::close(new_socket);
              continue;
            } else {
              response = "HTTP/1.1 200 OK\r\nContent-Length: 13\r\n\r\nHello, World!";
            }
          } else if (request.find("POST") == 0) {
            auto body_pos = request.find("\r\n\r\n");
            std::string body = (body_pos != std::string::npos) ? request.substr(body_pos + 4) : "";
            response = "HTTP/1.1 200 OK\r\nContent-Length: " + std::to_string(body.length()) +
                       "\r\n\r\n" + body;
          }
          send(new_socket, response.c_str(), response.length(), 0);
        }
        ::close(new_socket);
      }
    });
  }

  ~HttpServer()
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

TEST(TestHttpClient, TestGet)
{
  HttpServer server(g_server_port);
  HttpClient client(g_localhost_ip, g_server_port);
  auto response = client.get("/test");
  ASSERT_EQ(response, "Hello, World!");
}

TEST(TestHttpClient, TestPost)
{
  HttpServer server(g_server_port);
  HttpClient client(g_localhost_ip, g_server_port);
  std::string body = "test_body";
  auto response = client.post("/test", body);
  ASSERT_EQ(response, body);
}

TEST(TestHttpClient, TestChunkedResponse)
{
  HttpServer server(g_server_port);
  HttpClient client(g_localhost_ip, g_server_port);
  auto response = client.get("/chunked");
  ASSERT_EQ(response, "Hello World!");
}

TEST(TestHttpClient, TestTimeout)
{
  // No server running on this port, connection should fail or timeout
  HttpClient client(g_localhost_ip, 9998);
  // Client now catches SocketError internally and returns an empty string
  auto response = client.get("/test", 100);
  EXPECT_EQ(response, "");
}

TEST(TestHttpClient, TestConnectionFailure)
{
  HttpClient client(g_localhost_ip, 9997);
  auto response = client.get("/test");
  EXPECT_EQ(response, "");
}

TEST(TestHttpClient, TestInvalidHost)
{
  ASSERT_THROW(HttpClient("invalid_ip", 80), UsageError);
  ASSERT_THROW(HttpClient("256.0.0.1", 80), UsageError);
}

TEST(TestHttpClient, TestRobustHeaderParsing)
{
  uint16_t port = 8082;
  std::thread server_thread([port]() {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) == 0) {
      listen(fd, 1);
      int client = accept(fd, nullptr, nullptr);
      if (client >= 0) {
        char buf[1024];
        if (read(client, buf, 1024) > 0) {
          // Send weird header: Content-Length:5 (no space)
          std::string resp = "HTTP/1.1 200 OK\r\nContent-Length:5\r\n\r\nHello";
          send(client, resp.c_str(), resp.length(), 0);
        }
        close(client);
      }
      close(fd);
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  HttpClient client(g_localhost_ip, port);
  auto resp = client.get("/robust");
  ASSERT_EQ(resp, "Hello");
  if (server_thread.joinable()) server_thread.join();
}

TEST(TestHttpClient, TestEmptyResponse)
{
  HttpServer server(g_server_port);
  HttpClient client(g_localhost_ip, g_server_port);
  auto response = client.get("/anything");
  ASSERT_EQ(response, "Hello, World!");
}

TEST(TestHttpClient, TestSlowServer)
{
  HttpServer server(g_server_port);
  HttpClient client(g_localhost_ip, g_server_port);
  auto response = client.get("/slow", 50);  // Timeout before server responds
  EXPECT_EQ(response, "");
}

TEST(TestHttpClient, TestLargeChunkedResponse)
{
  HttpServer server(g_server_port);
  HttpClient client(g_localhost_ip, g_server_port);
  auto response = client.get("/large");
  // 5000 'A's + 5000 'A's = 10000 characters
  EXPECT_EQ(response.length(), 10000u);
  EXPECT_TRUE(response.find_first_not_of('A') == std::string::npos);
}

TEST(TestHttpClient, TestEndpointNormalization)
{
  HttpServer server(g_server_port);
  HttpClient client(g_localhost_ip, g_server_port);
  // Missing leading slash, client should prefix it automatically.
  // Should hit fallback Hello World
  auto response = client.get("normalize");
  EXPECT_EQ(response, "Hello, World!");
}

TEST(TestHttpClient, TestServerError)
{
  HttpServer server(g_server_port);
  HttpClient client(g_localhost_ip, g_server_port);
  auto response = client.get("/error");
  // Our HttpClient presently extracts body regardless of status code if parsable
  EXPECT_EQ(response, "ErrorBody");
}

TEST(TestHttpClient, TestConnectionDropMidResponse)
{
  HttpServer server(g_server_port);
  HttpClient client(g_localhost_ip, g_server_port);
  auto response = client.get("/drop");
  // Connection closed before complete content length was received, but
  // our client retains the partial body for robustness
  EXPECT_EQ(response, "Partial");
}

}  // namespace nebula::drivers::connections

int main(int argc, char * argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
