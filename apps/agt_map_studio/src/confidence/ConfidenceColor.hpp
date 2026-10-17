#pragma once

namespace agt_map_studio {

struct ConfidenceRgb { float r, g, b; };

// Centralized, deterministic [0,1] heatmap used by both the voxel GPU buffer
// and the QPainter legend. Monotone perceptual lightness at quarter stops.
ConfidenceRgb confidence_color(float value);

}  // namespace agt_map_studio
