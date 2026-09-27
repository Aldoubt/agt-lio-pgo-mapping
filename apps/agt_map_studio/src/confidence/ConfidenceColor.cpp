#include "confidence/ConfidenceColor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace agt_map_studio {

ConfidenceRgb confidence_color(float value) {
  constexpr std::array<ConfidenceRgb, 5> stops{{
      {0.06F, 0.12F, 0.32F}, {0.12F, 0.38F, 0.55F},
      {0.28F, 0.62F, 0.62F}, {0.62F, 0.77F, 0.55F},
      {0.94F, 0.92F, 0.42F},
  }};
  const float t = std::isfinite(value) ? std::clamp(value, 0.0F, 1.0F) : 0.0F;
  const float position = t * 4.0F;
  const auto low = std::min(static_cast<std::size_t>(position), stops.size() - 1);
  const auto high = std::min(low + 1, stops.size() - 1);
  const float fraction = position - static_cast<float>(low);
  return {stops[low].r + fraction * (stops[high].r - stops[low].r),
          stops[low].g + fraction * (stops[high].g - stops[low].g),
          stops[low].b + fraction * (stops[high].b - stops[low].b)};
}

}  // namespace agt_map_studio
