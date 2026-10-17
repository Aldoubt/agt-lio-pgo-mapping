#include "geometry/GeometryColor.hpp"

#include <algorithm>
#include <cmath>

namespace agt_map_studio {

ConfidenceRgb geometry_color(const agt_spatial_map_core::GeometryVoxelEvidence &v,
                             GeometryColorMode mode) {
  constexpr ConfidenceRgb invalid{0.46F, 0.46F, 0.46F};
  if (mode == GeometryColorMode::NormalShape) {
    if (!v.normal_valid) return invalid;
    return {0.12F + .82F * std::clamp(v.linearity, 0.0F, 1.0F),
            0.12F + .82F * std::clamp(v.planarity, 0.0F, 1.0F),
            0.12F + .82F * std::clamp(v.scattering, 0.0F, 1.0F)};
  }
  const bool translation = mode == GeometryColorMode::TranslationQ ||
                           mode == GeometryColorMode::TranslationWeak;
  const auto &s = translation ? v.translation : v.rotation;
  if (!s.valid) return invalid;
  if (mode == GeometryColorMode::TranslationQ || mode == GeometryColorMode::RotationQ) {
    return confidence_color(s.isotropy); // a heatmap, NOT confidence/probability
  }
  return {std::clamp(std::abs(s.weak_direction.x()), 0.0F, 1.0F),
          std::clamp(std::abs(s.weak_direction.y()), 0.0F, 1.0F),
          std::clamp(std::abs(s.weak_direction.z()), 0.0F, 1.0F)};
}

}  // namespace agt_map_studio
