#pragma once

#include "agt_spatial_map_core/spatial_evidence.hpp"

#include <cstdint>
#include <filesystem>

namespace agt_spatial_map_core {

struct EvidenceBuildStats {
  std::uint32_t source_keyframes = 0;
  std::uint64_t input_points = 0;
  std::uint64_t usable_points = 0;
  std::uint64_t nonfinite_points = 0;
  std::uint64_t voxel_count = 0;
};

// Streams existing body-frame patches and optimized poses_timed.txt through
// the same T_map_body + floor(float32 / float32) convention as the legacy
// temporal filter. Does not modify or replace that filter / traversability.
//
// This aggregation API checks input contracts but does not verify the parent
// manifest/checksum index. The CLI invokes the existing artifact validator
// before calling build(); other callers must verify their parent separately.
class SpatialEvidenceBuilder {
public:
  static SpatialEvidenceMap build(const std::filesystem::path &map_package,
                                  const ConfidenceParameters &parameters,
                                  EvidenceBuildStats *stats = nullptr);
};

}  // namespace agt_spatial_map_core
