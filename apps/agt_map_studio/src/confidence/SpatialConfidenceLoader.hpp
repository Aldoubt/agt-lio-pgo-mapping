#pragma once

#include "confidence/SpatialConfidenceModel.hpp"

#include <filesystem>
#include <string>

namespace agt_map_studio {

class SpatialConfidenceLoader {
public:
  // Validates all five derivative files, full checksum coverage, schema,
  // required typed PCD fields, manual YAML and parent manifest/checksum
  // identity. expected_parent is the *already validated* mapping package
  // opened by the studio; the UI separately runs the existing PGO validator.
  // No partial model is installed on failure.
  static bool load(const std::filesystem::path &derivative_dir,
                   const std::filesystem::path &expected_parent,
                   SpatialConfidenceModel *model, std::string *error);
};

}  // namespace agt_map_studio
