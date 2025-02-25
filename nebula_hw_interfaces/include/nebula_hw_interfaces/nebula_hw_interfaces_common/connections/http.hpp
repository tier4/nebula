// Copyright 2025 TIER IV, Inc.
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

#pragma once

#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/ssl/context.hpp>
#include <boost/asio/ssl/stream.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/core/error.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/http/field.hpp>
#include <boost/beast/http/status.hpp>
#include <boost/beast/version.hpp>

#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <string>
#include <utility>

namespace nebula::drivers::connections
{

namespace beast = boost::beast;
namespace http = beast::http;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

class HttpClient
{
  static constexpr uint8_t http_version = 11;

public:
  static constexpr char const * const content_type_json = "application/json";
  static constexpr char const * const content_type_cotest_stream = "application/octet-stream";

  struct HttpResponse
  {
    http::status status;
    std::string body;
  };

  HttpClient(std::string host, std::string port) : host_(std::move(host)), port_(std::move(port))
  {
    tcp::resolver resolver(ioc_);
    auto endpoints = resolver.resolve(host_, port_);
    stream_.connect(endpoints);
  }

  [[nodiscard]] HttpResponse get(const std::string & target)
  {
    http::request<http::string_body> req{http::verb::get, target, http_version};
    req.set(http::field::host, host_);
    return execute_request(req);
  }

  [[nodiscard]] std::future<HttpResponse> async_get(const std::string & target)
  {
    http::request<http::string_body> req{http::verb::get, target, http_version};
    req.set(http::field::host, host_);
    return execute_request_async(req);
  }

  [[nodiscard]] HttpResponse post(
    const std::string & target, const std::string & body, const std::string & content_type)
  {
    http::request<http::string_body> req{http::verb::post, target, http_version};
    req.set(http::field::host, host_);
    req.set(http::field::content_type, content_type);
    req.body() = body;
    req.prepare_payload();
    return execute_request(req);
  }

  [[nodiscard]] std::future<HttpResponse> async_post(
    const std::string & target, const std::string & body, const std::string & content_type)
  {
    http::request<http::string_body> req{http::verb::post, target, http_version};
    req.set(http::field::host, host_);
    req.set(http::field::content_type, content_type);
    req.body() = body;
    req.prepare_payload();
    return execute_request_async(req);
  }

private:
  HttpResponse execute_request(http::request<http::string_body> & req)
  {
    http::write(stream_, req);
    http::response<http::string_body> res;
    beast::flat_buffer buf;
    http::read(stream_, buf, res);
    return HttpResponse{res.result(), res.body()};
  }

  std::future<HttpResponse> execute_request_async(http::request<http::string_body> & req)
  {
    auto promise = std::make_shared<std::promise<HttpResponse>>();
    auto future = promise->get_future();

    http::async_write(
      stream_, req,
      [this, req = std::move(req), promise = std::move(promise)](beast::error_code ec, size_t) {
        if (ec) {
          promise->set_exception(std::make_exception_ptr(beast::system_error{ec}));
          return;
        }

        http::response<http::string_body> res;
        beast::flat_buffer buf;
        http::async_read(
          stream_, buf, res,
          [promise, buf = std::move(buf), res = std::move(res)](beast::error_code ec, size_t) {
            if (ec) {
              promise->set_exception(std::make_exception_ptr(beast::system_error{ec}));
              return;
            }
            promise->set_value(HttpResponse{res.result(), res.body()});
          });
      });

    return future;
  }

  std::string host_;
  std::string port_;
  asio::io_context ioc_;
  beast::tcp_stream stream_{ioc_};
};

}  // namespace nebula::drivers::connections
