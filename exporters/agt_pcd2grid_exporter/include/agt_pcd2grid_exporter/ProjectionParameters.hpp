#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace agt_pcd2grid_exporter {

enum class EmptyCellPolicy { Unknown, Free };
enum class ProjectionMode { FixedHeight, LocalGround };

struct ProjectionParameters {
  float resolution = 0.05F;
  bool origin_auto = true;
  float origin_x = 0.0F;
  float origin_y = 0.0F;
  float z_min = -0.3F;
  float z_max = 1.5F;
  ProjectionMode projection_mode = ProjectionMode::LocalGround;
  float ground_cell_size = 0.5F;
  float ground_percentile = 0.10F;
  std::uint32_t ground_neighbor_radius = 1U;
  float ground_free_tolerance = 0.10F;
  float obstacle_min_height = 0.12F;
  float obstacle_max_height = 1.80F;
  std::uint32_t occupied_threshold = 2U;
  std::uint32_t free_threshold = 1U;
  std::uint32_t free_space_radius_cells = 2U;
  std::uint32_t closing_radius_cells = 1U;
  std::uint32_t min_component_cells = 3U;
  bool temporal_filter_enabled = true;
  float temporal_voxel_size = 0.20F;
  std::uint32_t temporal_min_observations = 2U;
  std::uint32_t temporal_min_keyframe_span = 2U;
  EmptyCellPolicy empty_cell = EmptyCellPolicy::Unknown;
};

struct ProjectionStats {
  std::size_t input_points = 0;
  std::size_t accepted_points = 0;
  std::size_t z_filtered_points = 0;
  std::size_t ground_points = 0;
  std::size_t obstacle_points = 0;
  std::size_t occupied_cells = 0;
  std::size_t free_cells = 0;
  std::size_t unknown_cells = 0;
  std::size_t empty_cells = 0;
  std::size_t removed_small_component_cells = 0;
  std::size_t closing_added_cells = 0;
  std::size_t free_expanded_cells = 0;
  std::size_t temporal_input_points = 0;
  std::size_t temporal_retained_points = 0;
  std::size_t temporal_removed_points = 0;
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
  std::vector<std::uint32_t> free_count;
  std::vector<std::int8_t> occupancy;

  std::size_t size() const { return hit_count.size(); }
  std::uint32_t hits(std::uint32_t gx, std::uint32_t gy) const {
    return hit_count[static_cast<std::size_t>(gy) * width + gx];
  }
  std::int8_t value(std::size_t index, const ProjectionParameters &parameters) const {
    if (occupancy.size() == hit_count.size()) return occupancy[index];
    if (hit_count[index] >= parameters.occupied_threshold) return 100;
    return parameters.empty_cell == EmptyCellPolicy::Free ? 0 : -1;
  }
};

}  // namespace agt_pcd2grid_exporter
