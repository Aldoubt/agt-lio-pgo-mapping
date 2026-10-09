#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace agt_map_studio {

struct RenderPointSubset {
  std::vector<std::size_t> source_indices;
  std::vector<float> xyz;
};

// Deterministic, approximately even sampling in source order. The caller keeps
// the full source cloud for editing/export; this subset is only for rendering.
inline RenderPointSubset make_render_point_subset(const std::vector<float> &xyz,
                                                   std::size_t maximum_points,
                                                   bool z_filter_enabled,
                                                   double z_min, double z_max) {
  if (xyz.size() % 3U != 0U) {
    throw std::invalid_argument("point cloud XYZ buffer must contain triples");
  }
  const double low = std::min(z_min, z_max);
  const double high = std::max(z_min, z_max);
  const std::size_t source_count = xyz.size() / 3U;
  const auto eligible = [&](std::size_t index) {
    const float z = xyz[index * 3U + 2U];
    return !z_filter_enabled || (std::isfinite(z) && z >= low && z <= high);
  };

  std::size_t eligible_count = 0U;
  for (std::size_t i = 0; i < source_count; ++i) {
    if (eligible(i)) ++eligible_count;
  }
  const std::size_t target_count = maximum_points == 0U
      ? eligible_count : std::min(eligible_count, maximum_points);

  RenderPointSubset result;
  result.source_indices.reserve(target_count);
  result.xyz.reserve(target_count * 3U);
  if (target_count == eligible_count) {
    for (std::size_t i = 0; i < source_count; ++i) {
      if (!eligible(i)) continue;
      result.source_indices.push_back(i);
      result.xyz.insert(result.xyz.end(), xyz.begin() + static_cast<std::ptrdiff_t>(i * 3U),
                        xyz.begin() + static_cast<std::ptrdiff_t>(i * 3U + 3U));
    }
    return result;
  }

  std::size_t eligible_ordinal = 0U;
  std::size_t sample_ordinal = 0U;
  for (std::size_t i = 0; i < source_count && sample_ordinal < target_count; ++i) {
    if (!eligible(i)) continue;
    const auto desired = static_cast<std::size_t>(
        (static_cast<long double>(sample_ordinal) + 0.5L) * eligible_count / target_count);
    if (eligible_ordinal == desired) {
      result.source_indices.push_back(i);
      result.xyz.insert(result.xyz.end(), xyz.begin() + static_cast<std::ptrdiff_t>(i * 3U),
                        xyz.begin() + static_cast<std::ptrdiff_t>(i * 3U + 3U));
      ++sample_ordinal;
    }
    ++eligible_ordinal;
  }
  return result;
}

}  // namespace agt_map_studio
