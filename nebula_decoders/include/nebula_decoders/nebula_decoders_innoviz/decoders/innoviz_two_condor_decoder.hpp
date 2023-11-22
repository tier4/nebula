#pragma once

#include <cinttypes>
#include "innoviz_scan_decoder.hpp"

namespace nebula
{
namespace drivers
{
namespace itwo_condor
{

constexpr uint32_t MAX_POINTS = 16*32*600;

/// @brief Class for Innoviz Two Condor decoder
class InnovizTwoCondor : public InnovizScanDecoder
{
public:
    InnovizTwoCondor(const std::shared_ptr<drivers::InnovizSensorConfiguration>& sensorConfiguration);
};

}
}
}