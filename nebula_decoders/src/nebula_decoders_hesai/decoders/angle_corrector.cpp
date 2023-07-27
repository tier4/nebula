#include "nebula_decoders/nebula_decoders_hesai/decoders/angle_corrector.hpp"

namespace nebula
{
namespace drivers
{


template <size_t LookupTableSize>
AngleCorrector<LookupTableSize>::AngleCorrector(
    const std::shared_ptr<HesaiCalibrationConfiguration> & sensor_calibration,
    const std::shared_ptr<HesaiCorrection> & sensor_correction)
  : sensor_calibration_(sensor_calibration), sensor_correction_(sensor_correction), cos_map_(calculateLUT(true)), sin_map_(calculateLUT(false))
  {
  }

// Explicit template instantiation to prevent linker errors
template class AngleCorrector<360 * 100>;
template class AngleCorrector<360 * 100 * 256>;

}  // namespace drivers
}  // namespace nebula