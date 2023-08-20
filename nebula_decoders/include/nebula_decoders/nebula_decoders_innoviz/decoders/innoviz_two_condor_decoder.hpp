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
    
    class InnovizTwoCondor : InnovizScanDecoder
    {
        InnovizTwoCondor();
    };
}
}
}