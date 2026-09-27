#include "confidence/SpatialConfidenceModel.hpp"

#include <stdexcept>
#include <utility>

namespace agt_map_studio {

void SpatialConfidenceModel::assign(ConfidenceArtifactInfo info,
                                    std::vector<ConfidenceVoxel> voxels) {
  std::vector<float> xyz;
  xyz.reserve(voxels.size() * 3U);
  decltype(by_key_) keys;
  keys.reserve(voxels.size());
  for (std::size_t i = 0; i < voxels.size(); ++i) {
    if (!keys.emplace(voxels[i].key, i).second) {
      throw std::invalid_argument("Duplicate confidence voxel key");
    }
    const auto &center = voxels[i].center;
    xyz.insert(xyz.end(), {center.x(), center.y(), center.z()});
  }
  info_ = std::move(info);
  voxels_ = std::move(voxels);
  xyz_ = std::move(xyz);
  by_key_ = std::move(keys);
}

void SpatialConfidenceModel::clear() {
  info_ = ConfidenceArtifactInfo{};
  voxels_.clear();
  xyz_.clear();
  by_key_.clear();
}

const ConfidenceVoxel *SpatialConfidenceModel::find(
    const agt_spatial_map_core::VoxelKey &key) const {
  const auto it = by_key_.find(key);
  return it == by_key_.end() ? nullptr : &voxels_[it->second];
}

bool SpatialConfidenceModel::is_stable_preview(std::size_t index) const {
  const auto &v = voxels_.at(index);
  // Preview only. The published stable_map.pcd is always rebuilt by core.
  return v.override_mode != agt_spatial_map_core::ManualOverrideMode::IGNORE &&
         v.override_mode != agt_spatial_map_core::ManualOverrideMode::FORCE_LOW &&
         v.final_confidence >= info_.stable_threshold;
}

std::size_t SpatialConfidenceModel::stable_preview_count() const {
  std::size_t count = 0;
  for (std::size_t i = 0; i < voxels_.size(); ++i) {
    if (is_stable_preview(i)) ++count;
  }
  return count;
}

}  // namespace agt_map_studio
