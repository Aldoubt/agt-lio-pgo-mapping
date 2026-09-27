#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

namespace agt_spatial_map_core {

// Coordinates are floored in map frame at the configured voxel size. Do not
// interpret a key as a point position or an index into a 2D navigation grid.
struct VoxelKey {
  std::int64_t x = 0;
  std::int64_t y = 0;
  std::int64_t z = 0;

  bool operator==(const VoxelKey &other) const noexcept {
    return x == other.x && y == other.y && z == other.z;
  }
  bool operator<(const VoxelKey &other) const noexcept {
    if (x != other.x) return x < other.x;
    if (y != other.y) return y < other.y;
    return z < other.z;
  }
};

struct VoxelKeyHash {
  std::size_t operator()(const VoxelKey &key) const noexcept;
};

enum class ManualOverrideMode : std::uint32_t {
  AUTO = 0,
  FORCE_HIGH = 1,
  FORCE_LOW = 2,
  IGNORE = 3,
};

struct SpatialVoxelEvidence {
  VoxelKey key;
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  std::uint32_t point_count = 0;
  std::uint32_t observed_keyframes = 0;  // distinct keyframes, never point count
  std::uint32_t first_keyframe = 0;     // zero-based nonempty poses_timed record
  std::uint32_t last_keyframe = 0;
  std::uint32_t keyframe_span = 0;      // last - first, not an elapsed duration
  float observation_score = 0.0F;
  float persistence_score = 0.0F;
  float geometry_score = 1.0F;         // V1 has no geometry estimator (deferred)
  float auto_confidence = 0.0F;        // single-session evidence, NOT P(stable)
  ManualOverrideMode override_mode = ManualOverrideMode::AUTO;
  float manual_value = 0.0F;
  bool has_manual_value = false;
  float final_confidence = 0.0F;
};

using SpatialEvidenceMap =
    std::unordered_map<VoxelKey, SpatialVoxelEvidence, VoxelKeyHash>;

struct ConfidenceParameters {
  float voxel_size = 0.20F;
  float observation_reference = 4.0F;
  float keyframe_span_reference = 3.0F;
  float persistence_alpha = 0.7F;
  float stable_threshold = 0.60F;
  float force_low_value = 0.05F;
};

// Match TemporalPersistenceFilter's float32 floor(point / float32 voxel_size)
// indexing, including negative and exact-boundary coordinates.
VoxelKey voxel_for(const Eigen::Vector3f &point, float voxel_size);

void validate_parameters(const ConfidenceParameters &parameters);

// Recalculation preserves override_mode, manual_value and has_manual_value.
// Throws on invalid parameters or evidence rather than silently clamping bad
// observations into a plausible-looking map.
void calculate_confidence(SpatialVoxelEvidence *evidence,
                          const ConfidenceParameters &parameters);

const char *override_mode_name(ManualOverrideMode mode);
ManualOverrideMode parse_override_mode(const std::string &name);

}  // namespace agt_spatial_map_core
