#pragma once

#include "agt_pcd2grid_exporter/ProjectionParameters.hpp"

#include <string>

namespace agt_pcd2grid_exporter {

// Builds a Nav2 trinary map from a keyframe mapping package
// (patches/*.pcd in the body frame + poses_timed.txt with T_map_body).
//
// Unlike the point-projection modes, free space is inferred from evidence:
//   1. robot self-returns inside the footprint column are dropped;
//   2. a coarse ground surface is grown from the driven trajectory with a
//      bounded step, bridging short data gaps;
//   3. points are classified against that surface (ground / obstacle band);
//      obstacle evidence uses only temporally persistent points;
//   4. every sensor ray carves the cells it crosses while it is inside
//      [-ground_tolerance, carve_max_height] above the grown ground;
//   5. occupied = persistent obstacle hits (never cleared by rays), free =
//      grown ground + (ground return or >= free_min_passes carving rays),
//      plus the robot footprint swept along the optimized trajectory and small
//      holes fully enclosed by free cells; everything else stays unknown.
class TraversabilityGridBuilder {
public:
  static bool build(const std::string &package_dir,
                    const ProjectionParameters &parameters, OccupancyGrid *grid,
                    ProjectionStats *stats, TraversabilityStats *traversability,
                    std::string *error, const std::string &debug_dir = "");
};

}  // namespace agt_pcd2grid_exporter
