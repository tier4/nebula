// Copyright 2024 TIER IV, Inc.
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
  if (running_) return;
  running_ = true;
  thread_ = std::thread(&HttpPacketSource::run, this);
}

void HttpPacketSource::stop()
{
  running_ = false;
  if (thread_.joinable()) {
    if (thread_.get_id() == std::this_thread::get_id()) {
      thread_.detach();
    } else {
      thread_.join();
    }
  }
}

bool HttpPacketSource::is_running() const
{
  return running_;
}

void HttpPacketSource::run()
{
  client_ = std::make_unique<connections::HttpClient>(host_ip_, port_);

  while (running_) {
    try {
      auto response = client_->get(path_, 1000);
      if (!response.empty() && callback_) {
        SensorPacket sp;
        sp.transport = SensorTransportKind::HTTP;
        sp.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();

        SensorEndpoint dst;
        dst.address = host_ip_;
        dst.port = port_;
        sp.destination = dst;
        sp.payload.assign(response.begin(), response.end());

        callback_(sp);
      }
    } catch (const std::exception & e) {
      if (error_callback_) {
        SensorError error;
        error.type = SensorErrorType::TransportError;
        error.message = e.what();
        error_callback_(error);
      }
    }

    // Basic polling interval for HTTP source
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  client_.reset();
  running_ = false;
}

}  // namespace nebula::drivers
