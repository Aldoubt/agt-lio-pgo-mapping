#pragma once

#include "geometry/GeometryEvidenceModel.hpp"

#include <filesystem>
#include <string>

namespace agt_map_studio {

class GeometryEvidenceLoader {
public:
  // Core owns truth: its loader validates exactly three files and the complete
  // V1 derivative, parent/checksum digest, schema and typed VoxelKeys. Studio
  // additionally binds rows to the currently loaded confidence model by key.
  // On failure *model remains unchanged. No geometry editor/output exists.
  static bool load(const std::filesystem::path &sidecar,
                   const std::filesystem::path &expected_parent,
                   const std::filesystem::path &expected_confidence,
                   const SpatialConfidenceModel &confidence,
                   GeometryEvidenceModel *model, std::string *error);
};

}  // namespace agt_map_studio
