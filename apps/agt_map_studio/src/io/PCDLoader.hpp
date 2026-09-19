#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <pcl/PCLPointCloud2.h>

namespace agt_map_studio {

struct LoadedPointCloud {
  pcl::PCLPointCloud2::Ptr source;
  std::vector<float> xyz;
  std::vector<float> intensity;
  // Renderable point i originated at this record in source->data. Keeping
  // this mapping lets the editor export every original PCL field losslessly.
  std::vector<std::size_t> source_indices;
  Eigen::Vector3f min_bound = Eigen::Vector3f::Zero();
  Eigen::Vector3f max_bound = Eigen::Vector3f::Zero();
  bool has_intensity = false;
  std::size_t valid_point_count = 0;

  std::size_t point_count() const { return xyz.size() / 3U; }
  Eigen::Vector3f center() const { return (min_bound + max_bound) * 0.5F; }
  float diagonal() const { return (max_bound - min_bound).norm(); }
};

class PCDLoader {
public:
  static bool load(const std::string &path, LoadedPointCloud *result,
                   std::string *error);
};

}  // namespace agt_map_studio
