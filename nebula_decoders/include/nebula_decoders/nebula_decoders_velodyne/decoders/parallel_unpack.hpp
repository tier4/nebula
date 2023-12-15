#pragma once

#include "nebula_common/point_types.hpp"

#include <vector>

namespace nebula
{

namespace drivers
{
class ParallelUnpack
{
protected:
  std::vector<drivers::NebulaPointCloud> packet_clouds_;

public:
  void reset_packet_clouds(const size_t & n_packets, const int & n_pts_per_packet)
  {
    packet_clouds_.clear();
    packet_clouds_.resize(n_packets);
    for (auto & packet_cloud : packet_clouds_) {
      packet_cloud.points.reserve(n_pts_per_packet);
    }
  }

  void append_point(const drivers::NebulaPoint & point, const size_t & packet_idx)
  {
    packet_clouds_[packet_idx].points.push_back(point);
  }
};
}  // namespace drivers

}  // namespace nebula