// Copyright 2026 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NEBULA_CORE_HW_INTERFACES__NEBULA_HW_INTERFACES_COMMON__CONNECTIONS__HTTP_CLIENT_HPP_
#define NEBULA_CORE_HW_INTERFACES__NEBULA_HW_INTERFACES_COMMON__CONNECTIONS__HTTP_CLIENT_HPP_

#include "nebula_core_hw_interfaces/connections/tcp.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

namespace nebula::drivers::connections
{
/**
 * @brief A simple HTTP/1.1 client for GET and POST requests.
 *
 * @note This client currently supports responses with Content-Length only.
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
   * @param timeout_ms Timeout associated with request. Default is 500ms.
   * @return The response body as a string.
   */
  std::string get(const std::string & endpoint, int timeout_ms = 500)
  {
    std::string req = build_http_request_boilerplate("GET", endpoint, host_ip_);
    req += "Connection: close\r\n\r\n";
    return request(req, timeout_ms);
  }

  /**
   * @brief Perform an HTTP POST request.
   *
   * @param endpoint The resource path.
   * @param body The body content to send.
   * @param timeout_ms Timeout associated with request. Default is 500ms.
   * @return The response body as a string.
   */
  std::string post(const std::string & endpoint, const std::string & body, int timeout_ms = 500)
  {
    std::string req = build_http_request_boilerplate("POST", endpoint, host_ip_);
    req += "Content-Type: application/x-www-form-urlencoded\r\n";
    req += "Content-Length: " + std::to_string(body.length()) + "\r\n";
    req += "Connection: close\r\n\r\n";
    req += body;
    return request(req, timeout_ms);
  }

  /**
   * @brief Perform a raw HTTP request.
   *        Supports Content-Length responses.
   *
   * @param req A complete, well-formed HTTP request string (including headers and body).
   * @param timeout_ms Timeout associated with request.
   * @return The response body as a string, decoded if chunked.
   */
  std::string request(const std::string & req, int timeout_ms)
  {
    if (timeout_ms < 0) {
      throw UsageError("timeout_ms must be non-negative");
    }

    TcpSocket socket =
      timeout_ms > 0 ? TcpSocket::Builder(host_endpoint_).set_connect_timeout(timeout_ms).connect()
                     : TcpSocket::Builder(host_endpoint_).connect();

    std::vector<uint8_t> req_bytes(req.begin(), req.end());
    socket.send(req_bytes);

    std::string response_str;
    response_str.reserve(4096);

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    bool header_parsed = false;
    size_t content_length = 0;
    size_t body_pos = std::string::npos;

    while (true) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        throw SocketError("HTTP receive timeout");
      }

      const auto remaining_duration = deadline - now;
      auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(remaining_duration);
      if (remaining.count() <= 0) {
        remaining = std::chrono::milliseconds(1);
      }
      auto chunk = socket.receive(remaining);
      if (chunk.empty()) {
        throw SocketError("HTTP receive timeout");
      }

      response_str.append(reinterpret_cast<const char *>(chunk.data()), chunk.size());

      if (!header_parsed) {
        auto header_end = response_str.find("\r\n\r\n");
        if (header_end != std::string::npos) {
          header_parsed = true;
          body_pos = header_end;
          parse_headers(response_str, body_pos, content_length);
        }
      }

      if (!header_parsed) {
        continue;
      }

      const size_t body_start = body_pos + 4;
      if (response_str.size() >= body_start + content_length) {
        return response_str.substr(body_start, content_length);
      }
    }
  }

private:
  static void parse_headers(
    const std::string & response, size_t header_end_pos, size_t & content_length)
  {
    const std::string headers = response.substr(0, header_end_pos);
    const auto line_end = headers.find("\r\n");
    if (line_end == std::string::npos) {
      throw SocketError("Invalid HTTP response: malformed status line");
    }

    const auto status_line = headers.substr(0, line_end);
    if (status_line.rfind("HTTP/", 0) != 0) {
      throw SocketError("Invalid HTTP response: status line does not start with HTTP/");
    }

    std::string lower_headers = headers;
    std::transform(
      lower_headers.begin(), lower_headers.end(), lower_headers.begin(),
      [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if (lower_headers.find("transfer-encoding: chunked") != std::string::npos) {
      throw SocketError("Invalid HTTP response: chunked transfer encoding is not supported");
    }

    const std::string cl_key = "content-length:";
    const auto cl_pos = lower_headers.find(cl_key);
    if (cl_pos == std::string::npos) {
      throw SocketError("Invalid HTTP response: missing Content-Length");
    }

    size_t value_start = cl_pos + cl_key.size();
    while (value_start < lower_headers.size() &&
           (lower_headers[value_start] == ' ' || lower_headers[value_start] == '\t')) {
      ++value_start;
    }
    auto value_end = lower_headers.find("\r\n", value_start);
    if (value_end == std::string::npos) {
      value_end = lower_headers.size();
    }
    while (value_end > value_start &&
           (lower_headers[value_end - 1] == ' ' || lower_headers[value_end - 1] == '\t')) {
      --value_end;
    }
    if (value_start == value_end) {
      throw SocketError("Invalid HTTP response: malformed Content-Length");
    }
    for (size_t i = value_start; i < value_end; ++i) {
      if (!std::isdigit(static_cast<unsigned char>(lower_headers[i]))) {
        throw SocketError("Invalid HTTP response: non-numeric Content-Length");
      }
    }

    try {
      content_length = std::stoull(lower_headers.substr(value_start, value_end - value_start));
    } catch (const std::invalid_argument & e) {
      throw SocketError("Invalid HTTP response: Content-Length invalid: " + std::string(e.what()));
    } catch (const std::out_of_range & e) {
      throw SocketError(
        "Invalid HTTP response: Content-Length out of range: " + std::string(e.what()));
    }
  }

  static std::string build_http_request_boilerplate(
    const std::string & method, const std::string & endpoint, const std::string & host_ip)
  {
    std::string req = method + " ";
    if (endpoint.empty() || endpoint[0] != '/') {
      req += "/";
    }
    req += endpoint + " HTTP/1.1\r\n";
    req += "Host: " + host_ip + "\r\n";
    return req;
  }

  std::string host_ip_;
  Endpoint host_endpoint_;
};
}  // namespace nebula::drivers::connections

#endif  // NEBULA_CORE_HW_INTERFACES__NEBULA_HW_INTERFACES_COMMON__CONNECTIONS__HTTP_CLIENT_HPP_
