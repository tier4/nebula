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

#include <nebula_core_hw_interfaces/tcp_packet_source.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace nebula::drivers
{

TcpPacketSource::TcpPacketSource()
{
}

TcpPacketSource::~TcpPacketSource()
{
  stop();
}

void TcpPacketSource::configure(const std::string & host_ip, uint16_t port)
{
  host_ip_ = host_ip;
  port_ = port;
}

void TcpPacketSource::set_packet_callback(SensorPacketCallback callback)
{
  callback_ = callback;
}

void TcpPacketSource::set_error_callback(SensorErrorCallback callback)
{
  error_callback_ = callback;
}

void TcpPacketSource::start()
{
  if (running_ && running_->load()) return;
  if (thread_.joinable()) {
    if (thread_.get_id() == std::this_thread::get_id()) return;
    thread_.join();
  }

  running_ = std::make_shared<std::atomic<bool>>(true);
  thread_ =
    std::thread(&TcpPacketSource::run, running_, host_ip_, port_, callback_, error_callback_);
}

void TcpPacketSource::stop()
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

bool TcpPacketSource::is_running() const
{
  return running_ && running_->load();
}

void TcpPacketSource::run(
  std::shared_ptr<std::atomic<bool>> running, std::string host_ip, uint16_t port,
  SensorPacketCallback callback, SensorErrorCallback error_callback)
{
  std::unique_ptr<connections::TcpSocket> socket;
  try {
    auto s_builder = connections::TcpSocket::Builder(host_ip, port);
    s_builder.set_connect_timeout(1000);
    socket = std::make_unique<connections::TcpSocket>(std::move(s_builder).connect());
  } catch (const std::exception & e) {
    if (error_callback) {
      SensorError error;
      error.type = SensorErrorType::TransportError;
      error.message = e.what();
      error_callback(error);
    }
    running->store(false);
    return;
  }

  while (running->load() && socket) {
    try {
      auto data = socket->receive(std::chrono::milliseconds(100));
      if (!data.empty() && callback) {
        SensorPacket sp;
        sp.transport = SensorTransportKind::TCP;
        sp.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();

        sp.destination = SensorEndpoint{host_ip, port};
        sp.payload = std::move(data);

        callback(sp);
      }
    } catch (const std::exception & e) {
      if (error_callback) {
        SensorError error;
        error.type = SensorErrorType::TransportError;
        error.message = e.what();
        error_callback(error);
      }
      break;
    }
  }

  socket.reset();
  running->store(false);
}

}  // namespace nebula::drivers
