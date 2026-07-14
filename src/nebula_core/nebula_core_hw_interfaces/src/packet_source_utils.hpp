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

#ifndef NEBULA_PACKET_SOURCE_UTILS_HPP
#define NEBULA_PACKET_SOURCE_UTILS_HPP

#include <nebula_core_hw_interfaces/packet_source.hpp>

#include <atomic>
#include <cassert>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace nebula::drivers
{
inline void assert_not_worker_thread(const std::thread & thread, const char * method_name)
{
  assert(
    (!thread.joinable() || thread.get_id() != std::this_thread::get_id()) &&
    "packet source lifecycle methods must not be called from the worker thread");
  (void)method_name;
}

template <typename ThreadFactory>
void start_worker_thread(
  std::shared_ptr<std::atomic<bool>> & running, std::thread & thread,
  ThreadFactory && thread_factory)
{
  assert_not_worker_thread(thread, "start()");
  if (running && running->load()) {
    return;
  }
  if (thread.joinable()) {
    thread.join();
  }

  running = std::make_shared<std::atomic<bool>>(true);
  thread = thread_factory(running);
}

inline void stop_worker_thread(std::shared_ptr<std::atomic<bool>> & running, std::thread & thread)
{
  assert_not_worker_thread(thread, "stop()");
  if (running) {
    running->store(false);
  }
  if (thread.joinable()) {
    thread.join();
  }
}

inline void report_transport_error(
  const SensorErrorCallback & error_callback, SensorErrorType type, const std::string & message)
{
  if (!error_callback) {
    return;
  }

  SensorError error;
  error.type = type;
  error.message = message;
  error_callback(error);
}

inline void invoke_packet_callback(
  const char * source_name, const SensorPacketCallback & callback, const SensorPacket & packet)
{
  if (!callback) {
    return;
  }

  try {
    callback(packet);
  } catch (const std::exception & e) {
    std::cerr << source_name << ": user callback threw: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << source_name << ": user callback threw a non-std::exception" << std::endl;
  }
}

}  // namespace nebula::drivers

#endif  // NEBULA_PACKET_SOURCE_UTILS_HPP
