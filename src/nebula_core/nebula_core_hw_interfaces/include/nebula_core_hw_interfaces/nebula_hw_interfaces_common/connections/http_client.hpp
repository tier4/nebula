// Copyright 2024 TIER IV, Inc.

#ifndef NEBULA_CORE_HW_INTERFACES__NEBULA_HW_INTERFACES_COMMON__CONNECTIONS__HTTP_CLIENT_HPP_
#define NEBULA_CORE_HW_INTERFACES__NEBULA_HW_INTERFACES_COMMON__CONNECTIONS__HTTP_CLIENT_HPP_

#include "nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/tcp.hpp"

#include <condition_variable>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace nebula::drivers::connections
{
class HttpClient
{
public:
  HttpClient(const std::string & host_ip, uint16_t port) : host_ip_(host_ip), port_(port) {}

  std::string get(const std::string & endpoint, int timeout_ms = 1000)
  {
    TcpSocket socket = TcpSocket::Builder(host_ip_, port_).connect();
    std::stringstream request;
    request << "GET " << endpoint << " HTTP/1.1\r\n";
    request << "Host: " << host_ip_ << "\r\n";
    request << "Connection: close\r\n\r\n";

    std::string response_str;
    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;

    socket.subscribe([&](const std::vector<uint8_t> & data) {
      std::lock_guard<std::mutex> lock(mtx);
      response_str.append(reinterpret_cast<const char *>(data.data()), data.size());
      if (response_str.find("\r\n\r\n") != std::string::npos) {
        done = true;
        cv.notify_one();
      }
    });

    std::string req_str = request.str();
    std::vector<uint8_t> req_bytes(req_str.begin(), req_str.end());
    socket.send(req_bytes);

    std::unique_lock<std::mutex> lock(mtx);
    cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] { return done; });

    // Parse body
    auto body_pos = response_str.find("\r\n\r\n");
    if (body_pos != std::string::npos) {
      return response_str.substr(body_pos + 4);
    }
    return "";
  }

  std::string post(const std::string & endpoint, const std::string & body, int timeout_ms = 1000)
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
    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;

    socket.subscribe([&](const std::vector<uint8_t> & data) {
      std::lock_guard<std::mutex> lock(mtx);
      response_str.append(reinterpret_cast<const char *>(data.data()), data.size());
      if (response_str.find("\r\n\r\n") != std::string::npos) {
        done = true;
        cv.notify_one();
      }
    });

    std::string req_str = request.str();
    std::vector<uint8_t> req_bytes(req_str.begin(), req_str.end());
    socket.send(req_bytes);

    std::unique_lock<std::mutex> lock(mtx);
    cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] { return done; });

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
