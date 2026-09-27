#pragma once

#include <agt_spatial_map_core/spatial_evidence.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace agt_map_studio {

// One *voxel representative*, not a record in the source map.pcd. Field
// semantics and override enum are taken directly from agt_spatial_map_core.
struct ConfidenceVoxel {
  agt_spatial_map_core::VoxelKey key;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  std::uint32_t point_count = 0;
  std::uint32_t observed_keyframes = 0;
  std::uint32_t first_keyframe = 0;
  std::uint32_t last_keyframe = 0;
  std::uint32_t keyframe_span = 0;
  float observation_score = 0.0F;
  float persistence_score = 0.0F;
  float geometry_score = 1.0F;
  float auto_confidence = 0.0F;
  agt_spatial_map_core::ManualOverrideMode override_mode =
      agt_spatial_map_core::ManualOverrideMode::AUTO;
  bool has_manual_value = false;
  float manual_value = 0.0F;
  float final_confidence = 0.0F;
};

struct ConfidenceArtifactInfo {
  std::filesystem::path derivative_dir;
  std::filesystem::path source_package;
  std::string parent_manifest_sha256;
  std::string parent_checksums_sha256;
  float voxel_size = 0.0F;
  float stable_threshold = 0.0F;  // metadata.parameters.stable_threshold
  float force_low_value = 0.0F;
  std::uint64_t source_keyframes = 0;
  std::uint64_t stable_voxels = 0;
};

// UI-only immutable evidence snapshot. Neither a QWidget nor a PCL byte
// buffer owns/edits these typed values. A separate raw LoadedPointCloud and
// SelectionManager remain responsible for map.pcd and point deletion.
class SpatialConfidenceModel {
public:
  void assign(ConfidenceArtifactInfo info, std::vector<ConfidenceVoxel> voxels);
  void clear();
  bool empty() const { return voxels_.empty(); }
  const ConfidenceArtifactInfo &info() const { return info_; }
  const std::vector<ConfidenceVoxel> &voxels() const { return voxels_; }
  const std::vector<float> &xyz() const { return xyz_; }
  const ConfidenceVoxel *find(const agt_spatial_map_core::VoxelKey &key) const;
  bool is_stable_preview(std::size_t index) const;
  std::size_t stable_preview_count() const;

private:
  ConfidenceArtifactInfo info_;
  std::vector<ConfidenceVoxel> voxels_;
  std::vector<float> xyz_;  // map-frame voxel centers; index == voxels_ index only
  std::unordered_map<agt_spatial_map_core::VoxelKey, std::size_t,
                     agt_spatial_map_core::VoxelKeyHash> by_key_;
};

}  // namespace agt_map_studio
