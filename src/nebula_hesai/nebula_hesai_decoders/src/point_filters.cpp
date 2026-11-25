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

#include "nebula_hesai_decoders/point_filters.hpp"

#include "nebula_hesai_decoders/decoders/pandar_128e4x.hpp"
#include "nebula_hesai_decoders/decoders/pandar_40.hpp"
#include "nebula_hesai_decoders/decoders/pandar_64.hpp"
#include "nebula_hesai_decoders/decoders/pandar_at128.hpp"
#include "nebula_hesai_decoders/decoders/pandar_qt128.hpp"
#include "nebula_hesai_decoders/decoders/pandar_qt64.hpp"
#include "nebula_hesai_decoders/decoders/pandar_xt16.hpp"
#include "nebula_hesai_decoders/decoders/pandar_xt32.hpp"
#include "nebula_hesai_decoders/decoders/pandar_xt32m.hpp"

#include <nebula_core_common/nebula_common.hpp>

#include <memory>
#include <stdexcept>
#include <string>

namespace nebula::drivers
{

point_filters::DownsampleMaskFilter make_downsample_mask_filter(
  SensorModel sensor_model, const std::string & path,
  const std::shared_ptr<loggers::Logger> & logger)
{
  // The dithered mask is only written by the decoder, for inspection
  constexpr bool export_dithered_mask = false;

  switch (sensor_model) {
    case SensorModel::HESAI_PANDAR64:
      return make_downsample_mask_filter<Pandar64>(path, logger, export_dithered_mask);
    case SensorModel::HESAI_PANDAR40P:
    case SensorModel::HESAI_PANDAR40M:
      return make_downsample_mask_filter<Pandar40>(path, logger, export_dithered_mask);
    case SensorModel::HESAI_PANDARQT64:
      return make_downsample_mask_filter<PandarQT64>(path, logger, export_dithered_mask);
    case SensorModel::HESAI_PANDARQT128:
      return make_downsample_mask_filter<PandarQT128>(path, logger, export_dithered_mask);
    case SensorModel::HESAI_PANDARXT16:
      return make_downsample_mask_filter<PandarXT16>(path, logger, export_dithered_mask);
    case SensorModel::HESAI_PANDARXT32:
      return make_downsample_mask_filter<PandarXT32>(path, logger, export_dithered_mask);
    case SensorModel::HESAI_PANDARXT32M:
      return make_downsample_mask_filter<PandarXT32M>(path, logger, export_dithered_mask);
    case SensorModel::HESAI_PANDARAT128:
      return make_downsample_mask_filter<PandarAT128>(path, logger, export_dithered_mask);
    case SensorModel::HESAI_AT128P:
      return make_downsample_mask_filter<PandarAT128P>(path, logger, export_dithered_mask);
    case SensorModel::HESAI_PANDAR128_E4X:
      return make_downsample_mask_filter<Pandar128E4X>(path, logger, export_dithered_mask);
    default:
      throw std::invalid_argument(
        "Unsupported sensor model: " + sensor_model_to_string(sensor_model));
  }
}

}  // namespace nebula::drivers
