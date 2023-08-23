#pragma once

#include "nebula_common/nebula_common.hpp"

namespace nebula
{
namespace drivers
{
    struct InnovizSensorConfiguration : SensorConfigurationBase
    {
        uint8_t min_confidence = 21;
        bool filter_artifacts = true;
    };
}
}