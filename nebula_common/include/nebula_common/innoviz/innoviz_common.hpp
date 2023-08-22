#pragma once

#include "nebula_common/nebula_common.hpp"

namespace nebula
{
namespace drivers
{
    struct InnovizSensorConfiguration : SensorConfigurationBase
    {
        uint8_t minConfidence = 21;
        bool filterArtifacts = true;
    };
}
}