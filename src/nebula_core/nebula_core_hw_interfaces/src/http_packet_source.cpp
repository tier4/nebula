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

#include <nebula_core_hw_interfaces/http_packet_source.hpp>

#include <chrono>
#include <memory>
#include <string>

namespace nebula::drivers
{

HttpPacketSource::HttpPacketSource()
{
}

HttpPacketSource::~HttpPacketSource()
{
  stop();
}

void HttpPacketSource::configure(
  const std::string & host_ip, uint16_t port, const std::string & path)
{
  host_ip_ = host_ip;
  port_ = port;
  path_ = path;
}

void HttpPacketSource::set_packet_callback(SensorPacketCallback callback)
{
  callback_ = callback;
}

void HttpPacketSource::set_error_callback(SensorErrorCallback callback)
{
  error_callback_ = callback;
}

void HttpPacketSource::start()
{
  if (running_ && running_->load()) return;
  if (thread_.joinable()) {
    if (thread_.get_id() == std::this_thread::get_id()) return;
    thread_.join();
  }

  running_ = std::make_shared<std::atomic<bool>>(true);
  thread_ = std::thread(
    &HttpPacketSource::run, running_, host_ip_, port_, path_, callback_, error_callback_);
}

void HttpPacketSource::stop()
{
  if (running_) {
    running_->store(false);
  }
  if (thread_.joinable()) {
    if (thread_.get_id() == std::this_thread::get_id()) {
      thread_.detach();
      return;
    }
    thread_.join();
  }
}

bool HttpPacketSource::is_running() const
{
  return running_ && running_->load();
}

void HttpPacketSource::run(
  std::shared_ptr<std::atomic<bool>> running, std::string host_ip, uint16_t port, std::string path,
  SensorPacketCallback callback, SensorErrorCallback error_callback)
{
  std::unique_ptr<connections::HttpClient> client;
  try {
    client = std::make_unique<connections::HttpClient>(host_ip, port);
  } catch (const std::exception & e) {
    if (error_callback) {
      SensorError error;
      error.type = SensorErrorType::ConfigError;
      error.message = std::string("HttpPacketSource: failed to create HTTP client: ") + e.what();
      error_callback(error);
    }
    running->store(false);
    return;
  }

  while (running->load()) {
    try {
      auto response = client->get(path, 1000);
      if (!response.empty() && callback) {
        SensorPacket sp;
        sp.transport = SensorTransportKind::HTTP;
        sp.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();

        sp.destination = SensorEndpoint{host_ip, port};
        sp.payload.assign(response.begin(), response.end());

        callback(sp);
      }
    } catch (const std::exception & e) {
      if (error_callback) {
        SensorError error;
        error.type = SensorErrorType::TransportError;
        error.message = e.what();
        error_callback(error);
      }
    }

    // Basic polling interval for HTTP source
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  running->store(false);
}

}  // namespace nebula::drivers
