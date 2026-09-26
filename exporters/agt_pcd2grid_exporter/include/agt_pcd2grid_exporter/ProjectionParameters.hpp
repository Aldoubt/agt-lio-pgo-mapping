#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace agt_pcd2grid_exporter {

enum class EmptyCellPolicy { Unknown, Free };
enum class ProjectionMode { FixedHeight, LocalGround, Traversability };

// Rigid robot geometry needed by the traversability exporter. The mapping
// package stores T_map_body (FAST-LIO body = MID360 IMU frame); the robot
// footprint and self-filter are defined in base_footprint.
struct RobotModel {
  // Pose of the mapping body frame expressed in base_footprint (T_base_body).
  // Default: Bunker v1 = T_base_lidar(agt_robot_description, 13 deg pitch)
  // * T_lidar_body(FAST-LIO t_il = [-0.011, -0.02329, 0.04412], r_il = I).
  std::array<float, 3> base_from_body_xyz{{0.40686F, 0.02329F, 0.88201F}};
  std::array<float, 3> base_from_body_rpy{{0.0F, 0.2268928F, 0.0F}};
  // Footprint polygon in base_footprint (x forward, y left), metres.
  std::vector<std::pair<float, float>> footprint{
      {0.52F, 0.40F}, {0.52F, -0.40F}, {-0.52F, -0.40F}, {-0.52F, 0.40F}};
};

// Parameters of ProjectionMode::Traversability (ray-cast free-space evidence).
struct TraversabilityParameters {
  // Points inside the robot's own footprint column at their keyframe are
  // robot self-returns (camera mast, rear pole, chassis) and are dropped.
  bool self_filter_enabled = true;
  float self_filter_margin = 0.10F;
  float self_filter_min_height = -0.20F;
  float self_filter_max_height = 2.20F;
  // Coarse ground surface grown from the driven trajectory.
  float ground_cell_size = 0.25F;
  float ground_percentile = 0.10F;
  float ground_max_step = 0.10F;          // per coarse cell transition
  float ground_fill_max_distance = 1.0F;  // flat extrapolation across data gaps
  float ground_tolerance = 0.12F;         // |z - ground| <= tol -> ground return
  // Structures just outside the grown surface are classified against the
  // nearest grown floor height (never marks them free).
  float ground_reference_extension = 2.0F;
  // Free-space carving along sensor rays.
  bool raycast_enabled = true;
  float raycast_max_range = 30.0F;
  float carve_max_height = 1.0F;          // ray height above ground that may carve
  std::uint32_t carve_end_margin_cells = 2U;
  std::uint32_t free_min_passes = 2U;
  // Porous structure (vegetation, fences) leaves <= 1 hit per 5 cm cell while
  // rays slip through: a cell with a persistent hit is occupied when its
  // (2r+1)^2 neighbourhood holds >= occupied_threshold hits, and a cell with
  // any persistent hit is never declared free by carving.
  std::uint32_t obstacle_support_radius_cells = 1U;
  bool free_requires_no_hits = true;
  // Obstacle evidence must have been observed at least once from this range
  // [m]; voxels only ever seen in the near field are people following or
  // walking next to the robot, or robot attachments (0 disables).
  float obstacle_min_observation_range = 2.0F;
  // The footprint swept along the optimized trajectory is traversable: the
  // robot physically drove there.  padding > 0 grows the footprint (use the
  // costmap footprint_padding so the driven path stays plannable), < 0 shrinks.
  bool sweep_enabled = true;
  float sweep_padding = 0.03F;
  // Small unknown holes completely enclosed by free cells become free.
  float hole_fill_max_area = 0.50F;       // m^2
  // A ground robot never tilts this far; larger base tilts mean the mapping
  // run diverged (the package is rejected instead of exporting a bad map).
  float max_keyframe_tilt_deg = 30.0F;
  bool debug_layers = false;
};

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
  RobotModel robot;
  TraversabilityParameters traversability;
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

// Extra evidence statistics of ProjectionMode::Traversability.
struct TraversabilityStats {
  bool valid = false;
  std::size_t keyframes = 0;
  std::size_t self_filtered_points = 0;
  std::size_t ground_seed_cells = 0;
  std::size_t ground_observed_cells = 0;
  std::size_t ground_inferred_cells = 0;
  std::size_t ground_rejected_cells = 0;
  std::size_t rays_cast = 0;
  std::size_t ray_cells_visited = 0;
  std::size_t ground_hit_cells = 0;
  std::size_t carved_cells = 0;
  std::size_t support_occupied_cells = 0;
  std::size_t hit_vetoed_cells = 0;
  std::size_t near_field_only_points = 0;
  std::size_t sweep_cells = 0;
  std::size_t sweep_cleared_occupied_cells = 0;
  std::size_t hole_filled_cells = 0;
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
