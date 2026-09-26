#pragma once

#include "agt_pcd2grid_exporter/ProjectionParameters.hpp"

#include <string>

namespace agt_pcd2grid_exporter {

class OccupancyGridWriter {
public:
  static bool write_navigation_map(const OccupancyGrid &grid,
                                   const ProjectionParameters &parameters,
                                   const ProjectionStats &stats,
                                   const std::string &output_dir,
                                   const std::string &source_pcd,
                                   std::string *error,
                                   const TraversabilityStats *traversability = nullptr);
};

}  // namespace agt_pcd2grid_exporter
