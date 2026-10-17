#include "geometry/GeometryEvidenceLoader.hpp"

#include <agt_spatial_map_core/geometry_evidence.hpp>

#include <exception>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace agt_map_studio {

bool GeometryEvidenceLoader::load(const std::filesystem::path &sidecar,
                                  const std::filesystem::path &expected_parent,
                                  const std::filesystem::path &expected_confidence,
                                  const SpatialConfidenceModel &confidence,
                                  GeometryEvidenceModel *model, std::string *error) {
  if (!model) {
    if (error) *error = "GeometryEvidenceModel must not be null";
    return false;
  }
  try {
    if (confidence.empty() ||
        std::filesystem::canonical(confidence.info().source_package) !=
            std::filesystem::canonical(expected_parent) ||
        std::filesystem::canonical(confidence.info().derivative_dir) !=
            std::filesystem::canonical(expected_confidence)) {
      throw std::invalid_argument("open the exact matching PGO parent and V1 confidence first");
    }
    auto result = agt_spatial_map_core::load_geometry_evidence(
        sidecar, expected_parent, expected_confidence);
    GeometryEvidenceModel verified;
    verified.assign(std::move(result), confidence, std::filesystem::canonical(sidecar));
    *model = std::move(verified);
    if (error) error->clear();
    return true;
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
}

}  // namespace agt_map_studio
