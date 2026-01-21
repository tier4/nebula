// Copyright 2025 TIER IV, Inc.

#ifndef NEBULA_CORE_HW_INTERFACES__NEBULA_HW_INTERFACES_COMMON__CONNECTIONS__HTTP_CLIENT_HPP_
#define NEBULA_CORE_HW_INTERFACES__NEBULA_HW_INTERFACES_COMMON__CONNECTIONS__HTTP_CLIENT_HPP_

#include "nebula_core_hw_interfaces/nebula_hw_interfaces_common/connections/tcp.hpp"

#include <algorithm>
#include <cctype>
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
    const std::string & method, std::string endpoint, const std::string & body, int timeout_ms)
  {
    if (endpoint.empty() || endpoint[0] != '/') {
      endpoint = "/" + endpoint;
    }

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
    // Reserve some memory to avoid frequent reallocations
    response_str.reserve(4096);

    std::mutex mtx;
    std::condition_variable cv;
    bool done = false;
    size_t content_length = 0;
    bool header_parsed = false;
    bool is_chunked = false;
    size_t parse_pos = 0;

    socket.subscribe([&](const std::vector<uint8_t> & data) {
      std::lock_guard<std::mutex> lock(mtx);
      response_str.append(reinterpret_cast<const char *>(data.data()), data.size());

      if (!header_parsed) {
        // Search for double CRLF to find end of headers
        // Optimization: only search the newly added part plus overlap, but for headers,
        // searching from beginning is acceptable as headers are small.
        auto header_end = response_str.find("\r\n\r\n");
        if (header_end != std::string::npos) {
          header_parsed = true;

          // Parse headers (Case-insensitive)
          std::string headers = response_str.substr(0, header_end);
          // Simple lowercasing for easier search
          std::transform(headers.begin(), headers.end(), headers.begin(), [](unsigned char c) {
            return std::tolower(c);
          });

          if (headers.find("transfer-encoding: chunked") != std::string::npos) {
            is_chunked = true;
          } else {
            auto cl_pos = headers.find("content-length: ");
            if (cl_pos != std::string::npos) {
              auto cl_end = headers.find("\r\n", cl_pos);
              try {
                // headers is a copy, so indexes match relative to start
                // But we need to use the value from the original string?
                // Actually, digits are same in lower/upper, so it's fine.
                content_length = std::stoul(headers.substr(cl_pos + 16, cl_end - (cl_pos + 16)));
              } catch (...) {
                content_length = 0;
              }
            }
          }
          // Initialize parse_pos to start of body
          parse_pos = header_end + 4;
        }
      }

      if (header_parsed) {
        if (is_chunked) {
          // Stateful chunk parsing loop
          while (true) {
            auto crlf = response_str.find("\r\n", parse_pos);
            if (crlf == std::string::npos) break;  // Need more data for size line

            size_t chunk_size = 0;
            try {
              chunk_size =
                std::stoul(response_str.substr(parse_pos, crlf - parse_pos), nullptr, 16);
            } catch (...) {
              // Invalid chunk size format? Should probably error out or break.
              break;
            }

            size_t next_chunk_start = crlf + 2 + chunk_size + 2;  // size_line + CRLF + data + CRLF
            if (response_str.length() >= next_chunk_start) {
              if (chunk_size == 0) {
                // 0-sized chunk indicates end of stream
                done = true;
                cv.notify_one();
                break;
              }
              // Move to next chunk
              parse_pos = next_chunk_start;
            } else {
              break;  // Need more data for content
            }
          }
        } else {
          // Content-Length mode
          // If content_length is 0, we might be waiting for connection close,
          // or there really is no content.
          // If we have a content_length > 0, check if we have enough data.
          // Note: parse_pos is set to body start.
          if (content_length > 0) {
            if (response_str.length() >= parse_pos + content_length) {
              done = true;
              cv.notify_one();
            }
          } else {
            // content_length == 0. If headers didn't specify, we shouldn't assume we are done
            // if we expect connection close behavior.
            // BUT, if the response was truly 0 length (e.g. 204 No Content), we hang?
            // "Connection: close" handling without explicit length is tricky with purely this
            // logic. For this implementation, we won't set done=true here to allow receiving until
            // close/timeout if strict behavior is desired. However, common safe default for simple
            // clients: if CL=0 and not chunked, maybe assume done? Reviewer said: "The client
            // should not set done = true... It must wait until the socket detects a remote
            // closure". So we do NOTHING here. we wait for timeout or close.
          }
        }
      }
    });

    std::string req_str = request_ss.str();
    std::vector<uint8_t> req_bytes(req_str.begin(), req_str.end());
    socket.send(req_bytes);

    std::unique_lock<std::mutex> lock(mtx);
    // Wait for done OR timeout
    cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] { return done; });
    lock.unlock();

    // Ensure receiver thread is stopped before locals go out of scope
    socket.unsubscribe();

    if (!done && is_chunked) {
      // If chunked and not done, it's incomplete
      return "";
    }
    // If not chunked and not done (e.g. timeout or connection close), we might still have data
    // For Connection: close, we take what we have.

    auto body_pos = response_str.find("\r\n\r\n");
    if (body_pos == std::string::npos) {
      return "";
    }

    if (is_chunked) {
      std::string decoded_body;
      decoded_body.reserve(response_str.size());  // Approximation
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
    }

    return response_str.substr(body_pos + 4);
  }

private:
  std::string host_ip_;
  Endpoint host_endpoint_;
};
}  // namespace nebula::drivers::connections

#endif  // NEBULA_CORE_HW_INTERFACES__NEBULA_HW_INTERFACES_COMMON__CONNECTIONS__HTTP_CLIENT_HPP_
