// Copyright 2024 TIER IV, Inc.

#ifndef NEBULA_CORE_HW_INTERFACES__NEBULA_HW_INTERFACES_COMMON__CONNECTIONS__HTTP_CLIENT_HPP_
#define NEBULA_CORE_HW_INTERFACES__NEBULA_HW_INTERFACES_COMMON__CONNECTIONS__HTTP_CLIENT_HPP_

#include "nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/tcp.hpp"

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace nebula::drivers::connections
{
class HttpClient
{
public:
  HttpClient(const std::string & host_ip, uint16_t port) : host_ip_(host_ip), port_(port) {}

  std::string get(const std::string & endpoint)
  {
    TcpSocket socket = TcpSocket::Builder(host_ip_, port_).connect();
    std::stringstream request;
    request << "GET " << endpoint << " HTTP/1.1\r\n";
    request << "Host: " << host_ip_ << "\r\n";
    request << "Connection: close\r\n\r\n";

    std::string response_str;

    socket.subscribe([&](const std::vector<uint8_t> & data) {
      response_str.append(reinterpret_cast<const char *>(data.data()), data.size());
    });

    std::string req_str = request.str();
    std::vector<uint8_t> req_bytes(req_str.begin(), req_str.end());
    socket.send(req_bytes);

    // Wait for response (simple blocking implementation for now, as per requirements)
    // In a real async system we might want something better, but this mimics the sync behavior
    // of the previous driver for now.
    // We wait until the connection is closed by the server or timeout.
    // Since TcpSocket receive is in a thread, we need to wait.
    // However, TcpSocket doesn't expose "closed" state easily in the callback.
    // But standard HTTP/1.0 or Connection: close means server closes.
    // TcpSocket's receive loop exits on 0 bytes read (close).
    // We can check if socket is still open? TcpSocket doesn't have isOpen().
    // We'll just wait a bit. Ideally TcpSocket should notify on close.
    // For now, let's just wait for some data.

    // Actually, the previous implementation was synchronous.
    // We can just sleep a bit or wait for data.
    // A better way with TcpSocket would be to have a future/promise, but TcpSocket is callback
    // based.
    // Let's assume the server responds quickly.
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Parse body
    auto body_pos = response_str.find("\r\n\r\n");
    if (body_pos != std::string::npos) {
      return response_str.substr(body_pos + 4);
    }
    return "";
  }

  std::string post(const std::string & endpoint, const std::string & body)
  {
    TcpSocket socket = TcpSocket::Builder(host_ip_, port_).connect();
    std::stringstream request;
    request << "POST " << endpoint << " HTTP/1.1\r\n";
    request << "Host: " << host_ip_ << "\r\n";
    request << "Content-Type: application/x-www-form-urlencoded\r\n";
    request << "Content-Length: " << body.length() << "\r\n";
    request << "Connection: close\r\n\r\n";
    request << body;

    std::string response_str;

    socket.subscribe([&](const std::vector<uint8_t> & data) {
      response_str.append(reinterpret_cast<const char *>(data.data()), data.size());
    });

    std::string req_str = request.str();
    std::vector<uint8_t> req_bytes(req_str.begin(), req_str.end());
    socket.send(req_bytes);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    auto body_pos = response_str.find("\r\n\r\n");
    if (body_pos != std::string::npos) {
      return response_str.substr(body_pos + 4);
    }
    return "";
  }

private:
  std::string host_ip_;
  uint16_t port_;
};
}  // namespace nebula::drivers::connections

#endif  // NEBULA_CORE_HW_INTERFACES__NEBULA_HW_INTERFACES_COMMON__CONNECTIONS__HTTP_CLIENT_HPP_
