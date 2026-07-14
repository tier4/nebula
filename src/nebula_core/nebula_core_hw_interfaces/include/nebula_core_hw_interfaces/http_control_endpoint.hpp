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

#ifndef NEBULA_HTTP_CONTROL_ENDPOINT_HPP
#define NEBULA_HTTP_CONTROL_ENDPOINT_HPP

#include <nebula_core_hw_interfaces/connections/http_client.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

namespace nebula::drivers
{
class HttpControlEndpoint
{
public:
  void configure(
    const std::string & name, const std::string & host_ip, uint16_t port,
    const std::string & default_path)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    name_ = name;
    host_ip_ = host_ip;
    port_ = port;
    default_path_ = default_path;
    client_ = std::make_unique<connections::HttpClient>(host_ip_, port_);
  }

  const std::string & name() const { return name_; }

  // Returns a copy because the underlying string may be reassigned by configure().
  std::string default_path() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return default_path_;
  }

  std::string get(int timeout_ms = 500) const
  {
    std::string path;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      path = default_path_;
    }
    return get(path, timeout_ms);
  }

  std::string get(const std::string & path, int timeout_ms = 500) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return client().get(path, timeout_ms);
  }

  std::string post(const std::string & body, int timeout_ms = 500) const
  {
    std::string path;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      path = default_path_;
    }
    return post(path, body, timeout_ms);
  }

  std::string post(const std::string & path, const std::string & body, int timeout_ms = 500) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return client().post(path, body, timeout_ms);
  }

private:
  connections::HttpClient & client() const
  {
    if (!client_) {
      throw std::runtime_error("HTTP control endpoint is not configured");
    }
    return *client_;
  }

  std::string name_;
  std::string host_ip_;
  uint16_t port_{0};
  std::string default_path_{"/"};
  std::unique_ptr<connections::HttpClient> client_;
  mutable std::mutex mutex_;
};
}  // namespace nebula::drivers

#endif  // NEBULA_HTTP_CONTROL_ENDPOINT_HPP
