#pragma once

#include "agt_pcd2grid_exporter/ProjectionParameters.hpp"

#include <pcl/PCLPointCloud2.h>

#include <string>

namespace agt_pcd2grid_exporter {

struct TemporalFilterStats {
  std::size_t keyframes = 0;
  std::size_t input_points = 0;
  std::size_t retained_points = 0;
  std::size_t removed_points = 0;
};

// Rebuilds a static cloud from body-frame keyframe patches and optimized
// T_map_body poses. A voxel is retained only when independent keyframes
// observe it over a sufficient keyframe span.
class TemporalPersistenceFilter {
public:
  static bool filter_package(const std::string &package_dir,
                             const ProjectionParameters &parameters,
                             pcl::PCLPointCloud2 *cloud,
                             TemporalFilterStats *stats,
                             std::string *error);
};

}  // namespace agt_pcd2grid_exporter
