#include "geometry/GeometryEvidenceModel.hpp"

#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace agt_map_studio {

void GeometryEvidenceModel::assign(agt_spatial_map_core::GeometryEvidence evidence,
                                   const SpatialConfidenceModel &confidence,
                                   std::filesystem::path sidecar_directory) {
  if (confidence.empty() || evidence.voxels.size() != confidence.voxels().size() ||
      evidence.voxel_size != confidence.info().voxel_size) {
    throw std::invalid_argument("geometry evidence does not match the loaded V1 confidence model");
  }
  std::unordered_map<agt_spatial_map_core::VoxelKey, std::size_t,
                     agt_spatial_map_core::VoxelKeyHash> by_key;
  by_key.reserve(evidence.voxels.size());
  for (std::size_t i = 0; i < evidence.voxels.size(); ++i) {
    if (!by_key.emplace(evidence.voxels[i].key, i).second) {
      throw std::invalid_argument("duplicate geometry voxel key");
    }
  }
  std::vector<std::size_t> mapping;
  mapping.reserve(confidence.voxels().size());
  for (const auto &v : confidence.voxels()) {
    const auto found = by_key.find(v.key);
    if (found == by_key.end() ||
        evidence.voxels[found->second].center.x() != v.center.x() ||
        evidence.voxels[found->second].center.y() != v.center.y() ||
        evidence.voxels[found->second].center.z() != v.center.z()) {
      throw std::invalid_argument("geometry key/centroid differs from loaded confidence voxel");
    }
    mapping.push_back(found->second);
  }
  evidence_ = std::move(evidence);
  confidence_to_geometry_ = std::move(mapping);
  sidecar_directory_ = std::move(sidecar_directory);
}

void GeometryEvidenceModel::clear() {
  evidence_ = agt_spatial_map_core::GeometryEvidence{};
  confidence_to_geometry_.clear();
  sidecar_directory_.clear();
}

const agt_spatial_map_core::GeometryVoxelEvidence *
GeometryEvidenceModel::at_confidence_index(std::size_t index) const {
  if (index >= confidence_to_geometry_.size()) return nullptr;
  return &evidence_.voxels[confidence_to_geometry_[index]];
}

}  // namespace agt_map_studio
