
#include "nebula_decoders/nebula_decoders_innoviz/decoders/innoviz_two_condor_decoder.hpp"

namespace nebula
{
namespace drivers
{
namespace itwo_condor
{
    InnovizTwoCondor::InnovizTwoCondor(const std::shared_ptr<drivers::InnovizSensorConfiguration>& sensorConfiguration) 
            : InnovizScanDecoder(MAX_POINTS, sensorConfiguration)
    {}
}
}
}