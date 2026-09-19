#pragma once

#include "agt_pcd2grid_exporter/ProjectionParameters.hpp"

#include <pcl/PCLPointCloud2.h>

#include <string>

namespace agt_pcd2grid_exporter {

class PCDProjector {
public:
  static bool project(const pcl::PCLPointCloud2 &cloud,
                      const ProjectionParameters &parameters,
                      OccupancyGrid *grid, ProjectionStats *stats,
                      std::string *error);
};

}  // namespace agt_pcd2grid_exporter
