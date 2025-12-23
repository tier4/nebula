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
/**
 * @brief A simple HTTP/1.1 client for GET and POST requests.
 *
 * @note This client only supports responses with a Content-Length header.
 *       Responses using Transfer-Encoding: chunked are not supported and
 *       will result in an empty or partial response body.
 */
class HttpClient
{
public:
  HttpClient(const std::string & host_ip, uint16_t port)
  : host_ip_(host_ip), host_endpoint_{parse_ip(host_ip).value_or_throw(), port}
  {
  }

  std::string get(const std::string & endpoint, int timeout_ms = 1000)
  {
    return request("GET", endpoint, "", timeout_ms);
  }

  std::string post(const std::string & endpoint, const std::string & body, int timeout_ms = 1000)
  {
    return request("POST", endpoint, body, timeout_ms);
  }

  std::string request(
    const std::string & method, const std::string & endpoint, const std::string & body,
    int timeout_ms)
  {
    TcpSocket socket = TcpSocket::Builder(host_endpoint_).connect();
    std::stringstream request_ss;
    request_ss << method << " " << endpoint << " HTTP/1.1\r\n";
    request_ss << "Host: " << host_ip_ << "\r\n";
    if (method == "POST") {
      request_ss << "Content-Type: application/x-www-form-urlencoded\r\n";
      request_ss << "Content-Length: " << body.length() << "\r\n";
    }
    request_ss << "Connection: close\r\n\r\n";
    if (method == "POST") {
      request_ss << body;
    }

    std::string response_str;
    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    size_t content_length = 0;
    bool header_parsed = false;

    socket.subscribe([&](const std::vector<uint8_t> & data) {
      std::lock_guard<std::mutex> lock(mtx);
      response_str.append(reinterpret_cast<const char *>(data.data()), data.size());

      if (!header_parsed) {
        auto header_end = response_str.find("\r\n\r\n");
        if (header_end != std::string::npos) {
          header_parsed = true;
          auto cl_pos = response_str.find("Content-Length: ");
          if (cl_pos != std::string::npos) {
            auto cl_end = response_str.find("\r\n", cl_pos);
            try {
              content_length = std::stoul(response_str.substr(cl_pos + 16, cl_end - (cl_pos + 16)));
            } catch (...) {
              content_length = 0;
            }
          }
        }
      }

      if (header_parsed) {
        auto header_end = response_str.find("\r\n\r\n");
        if (response_str.length() >= header_end + 4 + content_length) {
          done = true;
          cv.notify_one();
        }
      }
    });

    std::string req_str = request_ss.str();
    std::vector<uint8_t> req_bytes(req_str.begin(), req_str.end());
    socket.send(req_bytes);

    std::unique_lock<std::mutex> lock(mtx);
    if (cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] { return done; })) {
      auto body_pos = response_str.find("\r\n\r\n");
      if (body_pos != std::string::npos) {
        return response_str.substr(body_pos + 4);
      }
    }

    return "";
  }

private:
  std::string host_ip_;
  Endpoint host_endpoint_;
};
}  // namespace nebula::drivers::connections

#endif  // NEBULA_CORE_HW_INTERFACES__NEBULA_HW_INTERFACES_COMMON__CONNECTIONS__HTTP_CLIENT_HPP_
