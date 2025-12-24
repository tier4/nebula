// Copyright 2025 TIER IV, Inc.

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
 * @note This client supports responses with Content-Length header and
 *       Transfer-Encoding: chunked.
 */
class HttpClient
{
public:
  HttpClient(const std::string & host_ip, uint16_t port)
  : host_ip_(host_ip), host_endpoint_{parse_ip(host_ip).value_or_throw(), port}
  {
  }

  /**
   * @brief Perform an HTTP GET request.
   *
   * @param endpoint The resource path (e.g. "/api/v1/status").
   * @param timeout_ms Timeout associated with request. Default is 1000ms.
   * @return The response body as a string.
   */
  std::string get(const std::string & endpoint, int timeout_ms = 1000)
  {
    return request("GET", endpoint, "", timeout_ms);
  }

  /**
   * @brief Perform an HTTP POST request.
   *
   * @param endpoint The resource path.
   * @param body The body content to send.
   * @param timeout_ms Timeout associated with request. Default is 1000ms.
   * @return The response body as a string.
   */
  std::string post(const std::string & endpoint, const std::string & body, int timeout_ms = 1000)
  {
    return request("POST", endpoint, body, timeout_ms);
  }

  /**
   * @brief Perform a raw HTTP request.
   *        Supports Content-Length and Transfer-Encoding: chunked.
   *
   * @param method HTTP method (GET, POST, etc.).
   * @param endpoint The resource path.
   * @param body The body content to send.
   * @param timeout_ms Timeout associated with request.
   * @return The response body as a string, decoded if chunked.
   */
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
    bool is_chunked = false;

    socket.subscribe([&](const std::vector<uint8_t> & data) {
      std::lock_guard<std::mutex> lock(mtx);
      response_str.append(reinterpret_cast<const char *>(data.data()), data.size());

      if (!header_parsed) {
        auto header_end = response_str.find("\r\n\r\n");
        if (header_end != std::string::npos) {
          header_parsed = true;
          if (response_str.find("Transfer-Encoding: chunked") != std::string::npos) {
            is_chunked = true;
          } else {
            auto cl_pos = response_str.find("Content-Length: ");
            if (cl_pos != std::string::npos) {
              auto cl_end = response_str.find("\r\n", cl_pos);
              try {
                content_length =
                  std::stoul(response_str.substr(cl_pos + 16, cl_end - (cl_pos + 16)));
              } catch (...) {
                content_length = 0;
              }
            }
          }
        }
      }

      if (header_parsed) {
        auto header_end = response_str.find("\r\n\r\n");
        if (is_chunked) {
          size_t pos = header_end + 4;
          while (true) {
            auto crlf = response_str.find("\r\n", pos);
            if (crlf == std::string::npos) break;

            size_t chunk_size = 0;
            try {
              chunk_size = std::stoul(response_str.substr(pos, crlf - pos), nullptr, 16);
            } catch (...) {
              break;
            }

            size_t next_chunk_start = crlf + 2 + chunk_size + 2;
            if (response_str.length() >= next_chunk_start) {
              if (chunk_size == 0) {
                done = true;
                cv.notify_one();
                break;
              }
              pos = next_chunk_start;
            } else {
              break;
            }
          }
        } else {
          if (response_str.length() >= header_end + 4 + content_length) {
            done = true;
            cv.notify_one();
          }
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
        if (is_chunked) {
          std::string decoded_body;
          size_t pos = body_pos + 4;
          while (true) {
            auto crlf = response_str.find("\r\n", pos);
            if (crlf == std::string::npos) break;
            size_t chunk_size = 0;
            try {
              chunk_size = std::stoul(response_str.substr(pos, crlf - pos), nullptr, 16);
            } catch (...) {
              break;
            }
            if (chunk_size == 0) break;
            decoded_body += response_str.substr(crlf + 2, chunk_size);
            pos = crlf + 2 + chunk_size + 2;
          }
          return decoded_body;
        } else {
          return response_str.substr(body_pos + 4);
        }
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
