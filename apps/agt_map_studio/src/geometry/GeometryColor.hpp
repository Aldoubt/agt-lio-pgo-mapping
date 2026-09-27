#pragma once

#include "confidence/ConfidenceColor.hpp"

#include <agt_spatial_map_core/geometry_evidence.hpp>

namespace agt_map_studio {

enum class GeometryColorMode {
  NormalShape, TranslationQ, RotationQ, TranslationWeak, RotationWeak,
};

// Invalid evidence is neutral gray, never painted as Q=0 or a good score.
// Shape encodes (linearity, planarity, scattering) as RGB; weak axes encode
// |map-frame x/y/z| as RGB, with eigenvector sign intentionally ignored.
ConfidenceRgb geometry_color(const agt_spatial_map_core::GeometryVoxelEvidence &v,
                             GeometryColorMode mode);

}  // namespace agt_map_studio
