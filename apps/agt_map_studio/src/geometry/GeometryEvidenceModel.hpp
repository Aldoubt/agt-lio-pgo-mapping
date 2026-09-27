#pragma once

#include "confidence/SpatialConfidenceModel.hpp"

#include <agt_spatial_map_core/geometry_evidence.hpp>

#include <cstddef>
#include <filesystem>
#include <vector>

namespace agt_map_studio {

// Immutable typed snapshot. The mapping from Studio confidence voxel indices
// to geometry indices is BY VOXEL KEY, NEVER by raw map.pcd point index.
class GeometryEvidenceModel {
public:
  void assign(agt_spatial_map_core::GeometryEvidence evidence,
              const SpatialConfidenceModel &confidence,
              std::filesystem::path sidecar_directory);
  void clear();
  bool empty() const { return evidence_.voxels.empty(); }
  const agt_spatial_map_core::GeometryVoxelEvidence *at_confidence_index(
      std::size_t index) const;
  const agt_spatial_map_core::GeometryEvidence &evidence() const { return evidence_; }
  const std::filesystem::path &sidecar_directory() const { return sidecar_directory_; }

private:
  agt_spatial_map_core::GeometryEvidence evidence_;
  std::vector<std::size_t> confidence_to_geometry_;
  std::filesystem::path sidecar_directory_;
};

}  // namespace agt_map_studio
