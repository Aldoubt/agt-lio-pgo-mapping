#pragma once

#include "confidence/SpatialConfidenceEditor.hpp"

#include <filesystem>
#include <string>

namespace agt_map_studio {

// Saves an audited, sorted Phase-1-compatible v1 override INTENT YAML only.
// No source derivative, original PGO package, confidence PCD or stable map is
// modified. Uses QSaveFile's temp+rename, and refuses target/source aliases.
class SpatialConfidenceIntentIO {
public:
  static bool save(const std::filesystem::path &path,
                   const SpatialConfidenceModel &model,
                   const SpatialConfidenceEditor &editor,
                   bool allow_replace = false, std::string *error = nullptr);
};

}  // namespace agt_map_studio
