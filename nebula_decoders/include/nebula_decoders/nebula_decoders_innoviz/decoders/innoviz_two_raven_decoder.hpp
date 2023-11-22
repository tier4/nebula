#pragma once

#include <cinttypes>
#include "innoviz_scan_decoder.hpp"

namespace nebula
{
namespace drivers
{
namespace itwo_raven
{

constexpr uint32_t MAX_POINTS = 30*32*600;

/// @brief Class for Innoviz Two Raven decoder
class InnovizTwoRaven : public InnovizScanDecoder
{
public:
    InnovizTwoRaven(const std::shared_ptr<drivers::InnovizSensorConfiguration>& sensorConfiguration);
};

}
}
}