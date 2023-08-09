#pragma once

#include "nebula_common/nebula_common.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector_calibration_based.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector_correction_based.hpp"
#include "nebula_decoders/nebula_decoders_hesai/decoders/hesai_packet.hpp"

#include <type_traits>

namespace nebula
{
namespace drivers
{

enum class AngleCorrectionType { CALIBRATION, CORRECTION };

/// @brief Base class for all sensor definitions
/// @tparam PacketT The packet type of the sensor
template <typename PacketT, AngleCorrectionType AngleCorrection = AngleCorrectionType::CALIBRATION>
class HesaiSensor
{
public:
  typedef PacketT packet_t;
  typedef typename std::conditional<
    (AngleCorrection == AngleCorrectionType::CALIBRATION),
    AngleCorrectorCalibrationBased<PacketT::N_CHANNELS, PacketT::DEGREE_SUBDIVISIONS>,
    AngleCorrectorCorrectionBased<PacketT::N_CHANNELS, PacketT::DEGREE_SUBDIVISIONS>>::type
    angle_corrector_t;

  HesaiSensor() = default;
  virtual ~HesaiSensor() = default;

  /// @brief Computes the exact relative time between the timestamp of the given packet and the one
  /// of the point identified by the given block and channel, in nanoseconds
  /// @param block_id The point's block id
  /// @param channel_id The point's channel id
  /// @param packet The packet
  /// @return The relative time offset in nanoseconds
  virtual int getPacketRelativePointTimeOffset(
    uint32_t block_id, uint32_t channel_id, const PacketT & packet);

  /// @brief Get the return type of the point given by return_idx
  ///
  /// For duplicate points, the return type is reported as-is for the first point, and set to @ref
  /// ReturnType::IDENTICAL for the duplicate(s). For DUAL_LAST_STRONGEST and DUAL_FIRST_STRONGEST,
  /// if the last/first point is also the strongest, it will be returned as @ref
  /// ReturnType::LAST_STRONGEST / @ref ReturnType::FIRST_STRONGEST respectively, with the second
  /// point being reported as @ref ReturnType::SECONDSTRONGEST.
  ///
  /// @param return_mode The sensor's currently active return mode
  /// @param return_idx The block index of the point within the group of blocks that make up the
  /// return group (e.g. either 0 or 1 for dual return)
  /// @param return_units The units corresponding to all the returns in the group. These are usually
  /// from the same column across adjascent blocks.
  /// @return The return type of the point
  virtual ReturnType getReturnType(
    hesai_packet::return_mode::ReturnMode return_mode, unsigned int return_idx,
    std::vector<typename packet_t::body_t::block_t::unit_t *> return_units)
  {
    unsigned int n_returns = return_units.size();

    const auto is_strongest = [&]() {
      for (unsigned int i = 0; i < n_returns; ++i) {
        if (i == return_idx) {
          continue;
        }

        if (return_units[return_idx]->reflectivity < return_units[i]->reflectivity) {
          return false;
        }
      }

      return true;
    };

    const auto is_duplicate = [&]() {
      for (unsigned int i = 0; i < n_returns; ++i) {
        if (i == return_idx) {
          continue;
        }

        if (
          return_units[return_idx]->distance == return_units[i]->distance &&
          return_units[return_idx]->reflectivity == return_units[i]->reflectivity) {
          return true;
        }
      }

      return false;
    };

    if (n_returns != 1 && is_duplicate()) {
      return ReturnType::IDENTICAL;
    }

    switch (return_mode) {
      case hesai_packet::return_mode::SINGLE_FIRST:
        return ReturnType::FIRST;
      case hesai_packet::return_mode::SINGLE_SECOND:
        return ReturnType::SECOND;
      case hesai_packet::return_mode::SINGLE_STRONGEST:
        return ReturnType::STRONGEST;
      case hesai_packet::return_mode::SINGLE_LAST:
        return ReturnType::LAST;
      case hesai_packet::return_mode::DUAL_LAST_STRONGEST:
        if (is_strongest()) {
          return return_idx == 0 ? ReturnType::LAST_STRONGEST : ReturnType::STRONGEST;
        } else {
          return return_idx == 0 ? ReturnType::LAST : ReturnType::SECONDSTRONGEST;
        }
      case hesai_packet::return_mode::DUAL_FIRST_SECOND:
        return return_idx == 0 ? ReturnType::FIRST : ReturnType::SECOND;
      case hesai_packet::return_mode::DUAL_FIRST_LAST:
        return return_idx == 0 ? ReturnType::FIRST : ReturnType::LAST;
      case hesai_packet::return_mode::DUAL_FIRST_STRONGEST:
        if (is_strongest()) {
          return return_idx == 0 ? ReturnType::FIRST_STRONGEST : ReturnType::STRONGEST;
        } else {
          return return_idx == 0 ? ReturnType::FIRST : ReturnType::SECONDSTRONGEST;
        }
      case hesai_packet::return_mode::DUAL_STRONGEST_SECONDSTRONGEST:
        return return_idx == 0 ? ReturnType::STRONGEST : ReturnType::SECONDSTRONGEST;
      case hesai_packet::return_mode::TRIPLE_FIRST_LAST_STRONGEST:
        switch (return_idx) {
          case 0:
            return ReturnType::FIRST;
          case 1:
            return ReturnType::LAST;
          case 2:
            return ReturnType::STRONGEST;
          default:
            return ReturnType::UNKNOWN;
        }
      default:
        return ReturnType::UNKNOWN;
    }
  }
};

}  // namespace drivers
}  // namespace nebula