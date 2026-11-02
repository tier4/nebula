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

#include "model_detect.hpp"

#include <nebula_hesai_decoders/decoders/pandar_128e4x.hpp>
#include <nebula_hesai_decoders/decoders/pandar_40.hpp>
#include <nebula_hesai_decoders/decoders/pandar_64.hpp>
#include <nebula_hesai_decoders/decoders/pandar_at128.hpp>
#include <nebula_hesai_decoders/decoders/pandar_qt128.hpp>
#include <nebula_hesai_decoders/decoders/pandar_qt64.hpp>
#include <nebula_hesai_decoders/decoders/pandar_xt16.hpp>
#include <nebula_hesai_decoders/decoders/pandar_xt32.hpp>
#include <nebula_hesai_decoders/decoders/pandar_xt32m.hpp>

#include <nebula_hesai_common/hesai_common.hpp>

#include <cstring>
#include <stdexcept>
#include <utility>

namespace nebula::cli
{
namespace
{
using namespace nebula::drivers::hesai_packet;  // NOLINT(build/namespaces)

template <typename PacketT>
uint8_t hesai_return_byte(const std::vector<uint8_t> & data)
{
  PacketT packet;
  std::memcpy(&packet, data.data(), sizeof(PacketT));
  return packet.tail.return_mode;
}

struct HesaiSize
{
  const char * model;
  size_t size;
  uint8_t (*return_byte)(const std::vector<uint8_t> &);
};

#define NEBULA_CLI_HESAI(name, packet_t) \
  {                                      \
    name, sizeof(packet_t), hesai_return_byte<packet_t> \
  }

// Pandar128E3X is left out: it has the same size as Pandar128E4X and no `--model` name.
const HesaiSize kHesai[] = {
  NEBULA_CLI_HESAI("Pandar64", Packet64),         NEBULA_CLI_HESAI("Pandar40P", Packet40P),
  NEBULA_CLI_HESAI("PandarQT64", PacketQT64),     NEBULA_CLI_HESAI("PandarQT128", PacketQT128C2X),
  NEBULA_CLI_HESAI("PandarXT16", PacketXT16),     NEBULA_CLI_HESAI("PandarXT32", PacketXT32),
  NEBULA_CLI_HESAI("PandarXT32M", PacketXT32M2X), NEBULA_CLI_HESAI("PandarAT128", PacketAT128E2X),
  NEBULA_CLI_HESAI("Pandar128E4X", Packet128E4X)};

// shortcut: real packets are the struct size plus optional trailers (UDP sequence, cyber security
// signature, ...). The allowed trailer sizes were measured on the test bags; a new trailer size
// makes detection fail with a clear error, in which case pass --model.
constexpr size_t kHesaiTrailers[] = {0, 4, 32, 36, 40};

std::string detect_hesai(size_t size)
{
  std::string found;
  for (const auto & m : kHesai) {
    for (const size_t trailer : kHesaiTrailers) {
      if (size == m.size + trailer) {
        if (!found.empty() && found != m.model) {
          throw std::runtime_error(
            "Packet size " + std::to_string(size) + " matches " + found + " and " + m.model +
            "; pass --model");
        }
        found = m.model;
      }
    }
  }
  if (found.empty()) {
    throw std::runtime_error(
      "Packet size " + std::to_string(size) + " matches no supported Hesai model; pass --model");
  }
  return found;
}

std::string detect_hesai_return_mode(const std::string & model_name, const std::vector<uint8_t> & p)
{
  const std::string name = model_name == "Pandar40M" ? "Pandar40P" : model_name;  // same packet
  for (const auto & m : kHesai) {
    if (name != m.model) continue;
    if (p.size() < m.size) {
      throw std::runtime_error("Hesai packet too short to read the return mode; pass --return-mode");
    }
    // Names accepted by return_mode_from_string_hesai for the given packet byte
    std::string mode;
    switch (m.return_byte(p)) {
      case return_mode::SINGLE_FIRST:
        mode = "First";
        break;
      case return_mode::SINGLE_STRONGEST:
        mode = "Strongest";
        break;
      case return_mode::SINGLE_LAST:
        mode = "Last";
        break;
      case return_mode::DUAL_LAST_STRONGEST:
        mode = "LastStrongest";
        break;
      case return_mode::DUAL_FIRST_LAST:
        mode = "LastFirst";
        break;
      case return_mode::DUAL_FIRST_STRONGEST:
        mode = "FirstStrongest";
        break;
      default:
        throw std::runtime_error("Unsupported Hesai return mode byte; pass --return-mode");
    }
    const auto sensor_model = nebula::drivers::sensor_model_from_string(model_name);
    if (nebula::drivers::return_mode_from_string_hesai(mode, sensor_model) == nebula::drivers::ReturnMode::UNKNOWN) {
      throw std::runtime_error(
        model_name + " does not support return mode " + mode + "; pass --return-mode");
    }
    return mode;
  }
  throw std::runtime_error("Unknown Hesai model " + model_name + "; pass --return-mode");
}

std::string detect_velodyne(const std::vector<uint8_t> & packet)
{
  constexpr size_t packet_size = 1206;
  constexpr size_t product_id_offset = 1205;
  if (packet.size() != packet_size) {
    throw std::runtime_error(
      "Velodyne packet size is " + std::to_string(packet.size()) + ", expected 1206");
  }
  switch (packet[product_id_offset]) {
    case 0x22:
      return "VLP16";
    case 0x28:
      return "VLP32";
    case 0xA1:
      return "VLS128";
    default:
      throw std::runtime_error(
        "Unknown Velodyne product ID " + std::to_string(packet[product_id_offset]) +
        "; pass --model");
  }
}

std::string detect_velodyne_return_mode(const std::vector<uint8_t> & packet)
{
  constexpr size_t return_mode_offset = 1204;
  if (packet.size() <= return_mode_offset) {
    throw std::runtime_error("Velodyne packet too short to read the return mode; pass --return-mode");
  }
  switch (packet[return_mode_offset]) {
    case 55:
      return "Strongest";
    case 56:
      return "Last";
    case 57:
      return "Dual";
    default:
      throw std::runtime_error("Unknown Velodyne return mode byte; pass --return-mode");
  }
}
}  // namespace

std::string detect_model(const std::string & vendor, const std::vector<uint8_t> & packet)
{
  if (vendor == "hesai") return detect_hesai(packet.size());
  if (vendor == "velodyne") return detect_velodyne(packet);
  throw std::runtime_error("Cannot detect the model for vendor '" + vendor + "'; pass --model");
}

std::string detect_return_mode(
  const std::string & vendor, const std::string & model, const std::vector<uint8_t> & packet)
{
  if (vendor == "hesai") return detect_hesai_return_mode(model, packet);
  if (vendor == "velodyne") return detect_velodyne_return_mode(packet);
  throw std::runtime_error("Cannot detect the return mode for vendor '" + vendor + "'; pass --return-mode");
}

}  // namespace nebula::cli
