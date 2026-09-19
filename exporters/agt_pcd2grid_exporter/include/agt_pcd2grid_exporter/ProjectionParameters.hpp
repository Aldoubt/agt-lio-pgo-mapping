#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace agt_pcd2grid_exporter {

enum class EmptyCellPolicy { Unknown, Free };

struct ProjectionParameters {
  float resolution = 0.05F;
  bool origin_auto = true;
  float origin_x = 0.0F;
  float origin_y = 0.0F;
  float z_min = -0.3F;
  float z_max = 1.5F;
  std::uint32_t occupied_threshold = 3U;
  EmptyCellPolicy empty_cell = EmptyCellPolicy::Unknown;
};

struct ProjectionStats {
  std::size_t input_points = 0;
  std::size_t accepted_points = 0;
  std::size_t z_filtered_points = 0;
  std::size_t occupied_cells = 0;
  std::size_t empty_cells = 0;
  float accepted_x_min = 0.0F;
  float accepted_x_max = 0.0F;
  float accepted_y_min = 0.0F;
  float accepted_y_max = 0.0F;
};

struct OccupancyGrid {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  float resolution = 0.05F;
  float origin_x = 0.0F;
  float origin_y = 0.0F;
  std::vector<std::uint32_t> hit_count;

  std::size_t size() const { return hit_count.size(); }
  std::uint32_t hits(std::uint32_t gx, std::uint32_t gy) const {
    return hit_count[static_cast<std::size_t>(gy) * width + gx];
  }
  std::int8_t value(std::size_t index, const ProjectionParameters &parameters) const {
    if (hit_count[index] >= parameters.occupied_threshold) return 100;
    return parameters.empty_cell == EmptyCellPolicy::Free ? 0 : -1;
  }
};

}  // namespace agt_pcd2grid_exporter
