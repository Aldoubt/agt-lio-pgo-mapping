#include "annotation/AisleProposalGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>

namespace agt_map_studio {
namespace {

struct LocalPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

constexpr double kPi = 3.14159265358979323846;

struct Peak {
  int bin = 0;
  double height = 0.0;
  double prominence = 0.0;
};

struct SupportedRow {
  int center_bin = 0;
  int start_bin = 0;
  int end_bin = -1;
  double support_fraction = 0.0;
  double prominence_fraction = 0.0;
};

struct OrientationCandidate {
  double angle_rad = 0.0;
  double score = 0.0;
  int supported_ridges = 0;
};

double cross(const QPointF &a, const QPointF &b, const QPointF &p) {
  return (b.x() - a.x()) * (p.y() - a.y()) - (b.y() - a.y()) * (p.x() - a.x());
}

bool inside_polygon(const QVector<QPointF> &polygon, double x, double y) {
  bool inside = false;
  const QPointF p(x, y);
  for (int i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const QPointF &a = polygon[j];
    const QPointF &b = polygon[i];
    const double edge_cross = cross(a, b, p);
    if (std::abs(edge_cross) <= 1e-9 &&
        x >= std::min(a.x(), b.x()) - 1e-9 && x <= std::max(a.x(), b.x()) + 1e-9 &&
        y >= std::min(a.y(), b.y()) - 1e-9 && y <= std::max(a.y(), b.y()) + 1e-9) return true;
    if ((a.y() > y) != (b.y() > y)) {
      const double crossing_x = (b.x() - a.x()) * (y - a.y()) / (b.y() - a.y()) + a.x();
      if (x < crossing_x) inside = !inside;
    }
  }
  return inside;
}

std::vector<double> smooth_box(const std::vector<double> &values, int radius) {
  std::vector<double> result(values.size(), 0.0);
  if (values.empty()) return result;
  for (std::size_t i = 0; i < values.size(); ++i) {
    const std::size_t first = i > static_cast<std::size_t>(radius) ? i - radius : 0;
    const std::size_t last = std::min(values.size() - 1, i + static_cast<std::size_t>(radius));
    double sum = 0.0;
    for (std::size_t j = first; j <= last; ++j) sum += values[j];
    result[i] = sum / static_cast<double>(last - first + 1);
  }
  return result;
}

void bridge_small_gaps(std::vector<bool> *active, int maximum_gap) {
  if (!active || maximum_gap <= 0) return;
  int i = 0;
  const int size = static_cast<int>(active->size());
  while (i < size) {
    if ((*active)[static_cast<std::size_t>(i)]) { ++i; continue; }
    const int begin = i;
    while (i < size && !(*active)[static_cast<std::size_t>(i)]) ++i;
    const int length = i - begin;
    if (begin > 0 && i < size && length <= maximum_gap) {
      for (int k = begin; k < i; ++k) (*active)[static_cast<std::size_t>(k)] = true;
    }
  }
}

QPointF from_uv(double u, double v, double origin_x, double origin_y,
                double direction_x, double direction_y) {
  const double lateral_x = -direction_y;
  const double lateral_y = direction_x;
  return {origin_x + u * direction_x + v * lateral_x,
          origin_y + u * direction_y + v * lateral_y};
}

double percentile(std::vector<double> values, double fraction) {
  if (values.empty()) return 0.0;
  const std::size_t index = std::min(values.size() - 1,
      static_cast<std::size_t>(fraction * static_cast<double>(values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
  return values[index];
}

double endpoint_z(const std::vector<LocalPoint> &points,
                  double x, double y) {
  constexpr double kRadiusSquared = 1.0;
  std::vector<double> local_z;
  for (const auto &point : points) {
    const double dx = point.x - x;
    const double dy = point.y - y;
    if (dx * dx + dy * dy <= kRadiusSquared) local_z.push_back(point.z);
  }
  return local_z.empty() ? 0.0 : percentile(std::move(local_z), 0.10);
}

double normalize_axis_angle(double angle) {
  while (angle < 0.0) angle += kPi;
  while (angle >= kPi) angle -= kPi;
  return angle;
}

double axial_angle_distance(double a, double b) {
  const double difference = std::abs(normalize_axis_angle(a) - normalize_axis_angle(b));
  return std::min(difference, kPi - difference);
}

OrientationCandidate score_row_orientation(const std::vector<LocalPoint> &samples,
                                           const QVector<QPointF> &boundary,
                                           double origin_x, double origin_y,
                                           double angle,
                                           const AisleExtractionParameters &p) {
  OrientationCandidate candidate;
  candidate.angle_rad = normalize_axis_angle(angle);
  if (samples.size() < 100U) return candidate;

  const double dx = std::cos(candidate.angle_rad);
  const double dy = std::sin(candidate.angle_rad);
  const double bin_size = std::max(0.10, std::min(p.profile_bin_m, 0.25));
  constexpr double kLongitudinalBinM = 0.50;
  double min_u = std::numeric_limits<double>::infinity();
  double max_u = -std::numeric_limits<double>::infinity();
  double min_v = std::numeric_limits<double>::infinity();
  double max_v = -std::numeric_limits<double>::infinity();
  for (const auto &point : boundary) {
    const double x = point.x() - origin_x;
    const double y = point.y() - origin_y;
    const double u = x * dx + y * dy;
    const double v = -x * dy + y * dx;
    min_u = std::min(min_u, u); max_u = std::max(max_u, u);
    min_v = std::min(min_v, v); max_v = std::max(max_v, v);
  }
  const int u_bins = static_cast<int>(std::ceil((max_u - min_u) / kLongitudinalBinM));
  const int v_bins = static_cast<int>(std::ceil((max_v - min_v) / bin_size));
  if (u_bins < 3 || v_bins < 3 || u_bins > 10000 || v_bins > 10000 ||
      static_cast<std::uint64_t>(u_bins) * static_cast<std::uint64_t>(v_bins) > 5000000ULL) return candidate;

  std::vector<std::uint32_t> grid(static_cast<std::size_t>(u_bins) * static_cast<std::size_t>(v_bins), 0U);
  std::vector<double> profile(static_cast<std::size_t>(v_bins), 0.0);
  for (const auto &point : samples) {
    const double x = point.x - origin_x;
    const double y = point.y - origin_y;
    const int ui = static_cast<int>((x * dx + y * dy - min_u) / kLongitudinalBinM);
    const int vi = static_cast<int>((-x * dy + y * dx - min_v) / bin_size);
    if (ui < 0 || ui >= u_bins || vi < 0 || vi >= v_bins) continue;
    ++grid[static_cast<std::size_t>(ui) * static_cast<std::size_t>(v_bins) + static_cast<std::size_t>(vi)];
    profile[static_cast<std::size_t>(vi)] += 1.0;
  }
  const auto smoothed = smooth_box(profile, std::max(1, static_cast<int>(std::lround(0.18 / bin_size))));
  const double maximum_peak = *std::max_element(smoothed.cbegin(), smoothed.cend());
  if (maximum_peak <= 0.0) return candidate;

  const int separation_bins = std::max(1, static_cast<int>(std::ceil(p.minimum_row_spacing_m / bin_size)));
  std::vector<Peak> peaks;
  for (int i = 1; i + 1 < v_bins; ++i) {
    const double height = smoothed[static_cast<std::size_t>(i)];
    if (height < maximum_peak * 0.01 || height < smoothed[static_cast<std::size_t>(i - 1)] ||
        height <= smoothed[static_cast<std::size_t>(i + 1)]) continue;
    const int radius = std::max(1, separation_bins / 2);
    double left_min = height;
    double right_min = height;
    for (int k = std::max(0, i - radius); k < i; ++k)
      left_min = std::min(left_min, smoothed[static_cast<std::size_t>(k)]);
    for (int k = i + 1; k <= std::min(v_bins - 1, i + radius); ++k)
      right_min = std::min(right_min, smoothed[static_cast<std::size_t>(k)]);
    const double prominence = height - std::max(left_min, right_min);
    if (prominence < height * 0.015) continue;
    peaks.push_back({i, height, prominence});
  }
  std::sort(peaks.begin(), peaks.end(), [](const Peak &a, const Peak &b) { return a.height > b.height; });
  std::vector<Peak> separated;
  for (const auto &peak : peaks) {
    if (std::none_of(separated.begin(), separated.end(), [&](const Peak &accepted) {
          return std::abs(accepted.bin - peak.bin) < separation_bins;
        })) separated.push_back(peak);
  }

  const int lateral_radius = std::max(1, static_cast<int>(std::ceil(0.25 / bin_size)));
  double weighted_prominence = 0.0;
  double weighted_coverage = 0.0;
  for (const auto &peak : separated) {
    std::vector<bool> active(static_cast<std::size_t>(u_bins), false);
    for (int u = 0; u < u_bins; ++u) {
      std::uint64_t count = 0;
      for (int v = std::max(0, peak.bin - lateral_radius); v <= std::min(v_bins - 1, peak.bin + lateral_radius); ++v)
        count += grid[static_cast<std::size_t>(u) * static_cast<std::size_t>(v_bins) + static_cast<std::size_t>(v)];
      active[static_cast<std::size_t>(u)] = count > 0;
    }
    bridge_small_gaps(&active, 2);
    int longest_run = 0;
    for (int u = 0; u < u_bins;) {
      while (u < u_bins && !active[static_cast<std::size_t>(u)]) ++u;
      const int start = u;
      while (u < u_bins && active[static_cast<std::size_t>(u)]) ++u;
      longest_run = std::max(longest_run, u - start);
    }
    const double supported_length = longest_run * kLongitudinalBinM;
    const double coverage = static_cast<double>(longest_run) / static_cast<double>(u_bins);
    if (supported_length < std::max(p.minimum_aisle_length_m, 0.25 * u_bins * kLongitudinalBinM) || coverage < 0.35) continue;
    ++candidate.supported_ridges;
    weighted_prominence += std::clamp(peak.prominence / std::max(peak.height, 1.0), 0.0, 1.0);
    weighted_coverage += coverage;
  }
  if (candidate.supported_ridges >= 2) {
    candidate.score = weighted_prominence * weighted_coverage /
                      static_cast<double>(candidate.supported_ridges);
  }
  return candidate;
}

bool infer_row_orientation(const std::vector<LocalPoint> &roi_points,
                           const QVector<QPointF> &boundary,
                           double origin_x, double origin_y,
                           const AisleExtractionParameters &p,
                           OrientationCandidate *winner,
                           double *margin) {
  if (!winner || !margin || roi_points.size() < 100U) return false;
  constexpr std::size_t kMaximumOrientationSamples = 60000U;
  const std::size_t stride = std::max<std::size_t>(1U, (roi_points.size() + kMaximumOrientationSamples - 1U) /
                                                       kMaximumOrientationSamples);
  std::vector<LocalPoint> samples;
  samples.reserve(std::min(kMaximumOrientationSamples, roi_points.size()));
  for (std::size_t i = 0; i < roi_points.size(); i += stride) samples.push_back(roi_points[i]);

  std::vector<OrientationCandidate> candidates;
  const auto evaluate = [&](double degrees) {
    const double angle = normalize_axis_angle(degrees * kPi / 180.0);
    const bool seen = std::any_of(candidates.begin(), candidates.end(), [&](const OrientationCandidate &item) {
      return axial_angle_distance(item.angle_rad, angle) < 0.1 * kPi / 180.0;
    });
    if (!seen) candidates.push_back(score_row_orientation(samples, boundary, origin_x, origin_y, angle, p));
  };
  for (int degrees = 0; degrees < 180; degrees += 5) evaluate(degrees);
  auto best_candidate = [&]() {
    return std::max_element(candidates.begin(), candidates.end(), [](const auto &a, const auto &b) {
      return a.score < b.score;
    });
  };
  auto best = best_candidate();
  if (best == candidates.end() || best->score <= 0.0) return false;
  const double coarse_degrees = best->angle_rad * 180.0 / kPi;
  for (int offset = -5; offset <= 5; ++offset) evaluate(coarse_degrees + offset);
  best = best_candidate();
  const double fine_degrees = best->angle_rad * 180.0 / kPi;
  for (int step = -4; step <= 4; ++step) evaluate(fine_degrees + 0.25 * step);
  best = best_candidate();
  if (best == candidates.end() || best->supported_ridges < 2 || best->score < 0.20) return false;

  double competing_score = 0.0;
  for (const auto &candidate : candidates) {
    if (axial_angle_distance(candidate.angle_rad, best->angle_rad) < 15.0 * kPi / 180.0) continue;
    competing_score = std::max(competing_score, candidate.score);
  }
  *margin = (best->score - competing_score) / std::max(best->score, 1e-9);
  if (*margin < 0.20) return false;
  *winner = *best;
  return true;
}

bool valid_parameters(const AisleExtractionParameters &p, QString *error) {
  const bool valid = std::isfinite(p.z_min_m) && std::isfinite(p.z_max_m) && p.z_min_m <= p.z_max_m &&
      std::isfinite(p.profile_bin_m) && p.profile_bin_m >= 0.03 && p.profile_bin_m <= 1.0 &&
      std::isfinite(p.minimum_row_spacing_m) && p.minimum_row_spacing_m >= 0.30 &&
      std::isfinite(p.row_half_width_m) && p.row_half_width_m > 0.0 &&
      std::isfinite(p.side_clearance_m) && p.side_clearance_m >= 0.0 &&
      std::isfinite(p.minimum_aisle_width_m) && p.minimum_aisle_width_m > 0.0 &&
      std::isfinite(p.maximum_aisle_width_m) && p.maximum_aisle_width_m >= p.minimum_aisle_width_m &&
      std::isfinite(p.minimum_aisle_length_m) && p.minimum_aisle_length_m > 0.0 &&
      std::isfinite(p.minimum_row_support_fraction) && p.minimum_row_support_fraction > 0.0 &&
      p.minimum_row_support_fraction <= 1.0 && std::isfinite(p.minimum_peak_fraction) &&
      p.minimum_peak_fraction > 0.0 && p.minimum_peak_fraction <= 1.0 &&
      std::isfinite(p.minimum_peak_prominence_fraction) && p.minimum_peak_prominence_fraction >= 0.0 &&
      p.minimum_peak_prominence_fraction <= 1.0 && p.minimum_points_per_support_bin >= 1 &&
      p.maximum_support_gap_bins >= 0;
  if (!valid && error) *error = QStringLiteral("Aisle proposal parameters are invalid or outside supported ranges.");
  return valid;
}

}  // namespace

bool generate_aisle_proposals(const std::vector<float> &xyz,
                              const QVector<QPointF> &boundary,
                              const AisleExtractionParameters &p,
                              AisleProposalGenerationResult *result,
                              QString *error) {
  if (result) *result = {};
  if (!result || xyz.size() < 9 || xyz.size() % 3 != 0 || boundary.size() < 3) {
    if (error) *error = QStringLiteral("A point cloud and a saved greenhouse boundary polygon are required.");
    return false;
  }
  if (!valid_parameters(p, error)) return false;
  double boundary_twice_area = 0.0;
  for (int i = 0; i < boundary.size(); ++i) {
    const auto &point = boundary[i];
    const auto &next = boundary[(i + 1) % boundary.size()];
    if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
      if (error) *error = QStringLiteral("Greenhouse boundary must contain only finite XY coordinates.");
      return false;
    }
    boundary_twice_area += point.x() * next.y() - next.x() * point.y();
  }
  if (std::abs(boundary_twice_area) <= 1e-6) {
    if (error) *error = QStringLiteral("Greenhouse boundary polygon must have nonzero area.");
    return false;
  }

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (const auto &point : boundary) { mean_x += point.x(); mean_y += point.y(); }
  mean_x /= static_cast<double>(boundary.size());
  mean_y /= static_cast<double>(boundary.size());
  std::vector<LocalPoint> roi_points;
  roi_points.reserve(std::min<std::size_t>(xyz.size() / 3U, 2000000U));
  result->points_examined = xyz.size() / 3;
  const double min_x = std::min_element(boundary.cbegin(), boundary.cend(), [](const QPointF &a, const QPointF &b) { return a.x() < b.x(); })->x();
  const double max_x = std::max_element(boundary.cbegin(), boundary.cend(), [](const QPointF &a, const QPointF &b) { return a.x() < b.x(); })->x();
  const double min_y = std::min_element(boundary.cbegin(), boundary.cend(), [](const QPointF &a, const QPointF &b) { return a.y() < b.y(); })->y();
  const double max_y = std::max_element(boundary.cbegin(), boundary.cend(), [](const QPointF &a, const QPointF &b) { return a.y() < b.y(); })->y();
  for (std::size_t i = 0; i + 2 < xyz.size(); i += 3) {
    const double x = xyz[i];
    const double y = xyz[i + 1];
    const double z = xyz[i + 2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z < p.z_min_m || z > p.z_max_m ||
        x < min_x || x > max_x || y < min_y || y > max_y || !inside_polygon(boundary, x, y)) continue;
    roi_points.push_back({x, y, z});
    ++result->points_inside_roi;
  }
  if (result->points_inside_roi < 100) {
    if (error) *error = QStringLiteral("Fewer than 100 finite points fall inside the saved greenhouse boundary and Z range.");
    return false;
  }

  OrientationCandidate inferred_orientation;
  double orientation_margin = 0.0;
  if (!infer_row_orientation(roi_points, boundary, mean_x, mean_y, p,
                             &inferred_orientation, &orientation_margin)) {
    if (error) *error = QStringLiteral(
        "Point-cloud row direction is weak or ambiguous in this greenhouse boundary and Z range. Adjust the Z interval or review the boundary before extracting aisle candidates.");
    return false;
  }
  double angle = inferred_orientation.angle_rad;
  if (angle >= 0.5 * kPi) angle -= kPi;
  const double direction_x = std::cos(angle);
  const double direction_y = std::sin(angle);
  result->dominant_axis_deg = angle * 180.0 / kPi;
  result->orientation_score = inferred_orientation.score;
  result->orientation_margin = orientation_margin;

  double min_u = std::numeric_limits<double>::infinity();
  double max_u = -std::numeric_limits<double>::infinity();
  double min_v = std::numeric_limits<double>::infinity();
  double max_v = -std::numeric_limits<double>::infinity();
  for (const auto &point : boundary) {
    const double dx = point.x() - mean_x;
    const double dy = point.y() - mean_y;
    const double u = dx * direction_x + dy * direction_y;
    const double v = -dx * direction_y + dy * direction_x;
    min_u = std::min(min_u, u); max_u = std::max(max_u, u);
    min_v = std::min(min_v, v); max_v = std::max(max_v, v);
  }
  const int u_bins = static_cast<int>(std::ceil((max_u - min_u) / p.profile_bin_m));
  const int v_bins = static_cast<int>(std::ceil((max_v - min_v) / p.profile_bin_m));
  if (u_bins < 3 || v_bins < 3 || u_bins > 100000 || v_bins > 100000 ||
      static_cast<std::uint64_t>(u_bins) * static_cast<std::uint64_t>(v_bins) > 20000000ULL) {
    if (error) *error = QStringLiteral("Greenhouse boundary is too small or too large for the selected profile resolution.");
    return false;
  }

  std::vector<std::uint32_t> grid(static_cast<std::size_t>(u_bins) * static_cast<std::size_t>(v_bins), 0U);
  std::vector<double> profile(static_cast<std::size_t>(v_bins), 0.0);
  for (const auto &point : roi_points) {
    const double dx = point.x - mean_x;
    const double dy = point.y - mean_y;
    const double u = dx * direction_x + dy * direction_y;
    const double v = -dx * direction_y + dy * direction_x;
    const int ui = std::clamp(static_cast<int>((u - min_u) / p.profile_bin_m), 0, u_bins - 1);
    const int vi = std::clamp(static_cast<int>((v - min_v) / p.profile_bin_m), 0, v_bins - 1);
    ++grid[static_cast<std::size_t>(ui) * static_cast<std::size_t>(v_bins) + static_cast<std::size_t>(vi)];
    profile[static_cast<std::size_t>(vi)] += 1.0;
  }

  const int smoothing_radius = std::max(1, static_cast<int>(std::lround(0.20 / p.profile_bin_m)));
  const auto smoothed_profile = smooth_box(profile, smoothing_radius);
  const double maximum_peak = *std::max_element(smoothed_profile.cbegin(), smoothed_profile.cend());
  std::vector<Peak> raw_peaks;
  const int separation_bins = std::max(1, static_cast<int>(std::ceil(p.minimum_row_spacing_m / p.profile_bin_m)));
  for (int i = 1; i + 1 < v_bins; ++i) {
    const double height = smoothed_profile[static_cast<std::size_t>(i)];
    if (height < maximum_peak * p.minimum_peak_fraction ||
        height < smoothed_profile[static_cast<std::size_t>(i - 1)] ||
        height <= smoothed_profile[static_cast<std::size_t>(i + 1)]) continue;
    const int radius = std::max(1, separation_bins / 2);
    double left_min = height;
    double right_min = height;
    for (int k = std::max(0, i - radius); k < i; ++k)
      left_min = std::min(left_min, smoothed_profile[static_cast<std::size_t>(k)]);
    for (int k = i + 1; k <= std::min(v_bins - 1, i + radius); ++k)
      right_min = std::min(right_min, smoothed_profile[static_cast<std::size_t>(k)]);
    const double prominence = height - std::max(left_min, right_min);
    if (prominence < height * p.minimum_peak_prominence_fraction) continue;
    raw_peaks.push_back({i, height, prominence});
  }
  std::sort(raw_peaks.begin(), raw_peaks.end(), [](const Peak &a, const Peak &b) { return a.height > b.height; });
  std::vector<Peak> peaks;
  for (const auto &peak : raw_peaks) {
    const bool too_close = std::any_of(peaks.begin(), peaks.end(), [&](const Peak &accepted) {
      return std::abs(accepted.bin - peak.bin) < separation_bins;
    });
    if (!too_close) peaks.push_back(peak);
  }
  std::sort(peaks.begin(), peaks.end(), [](const Peak &a, const Peak &b) { return a.bin < b.bin; });

  std::vector<SupportedRow> rows;
  const int lateral_radius = std::max(0, static_cast<int>(std::ceil(p.row_half_width_m / p.profile_bin_m)));
  const int longitudinal_smoothing_radius = std::max(1, static_cast<int>(std::lround(0.25 / p.profile_bin_m)));
  for (const auto &peak : peaks) {
    std::vector<double> longitudinal(static_cast<std::size_t>(u_bins), 0.0);
    for (int v = std::max(0, peak.bin - lateral_radius); v <= std::min(v_bins - 1, peak.bin + lateral_radius); ++v) {
      for (int u = 0; u < u_bins; ++u) {
        longitudinal[static_cast<std::size_t>(u)] += grid[static_cast<std::size_t>(u) * static_cast<std::size_t>(v_bins) + static_cast<std::size_t>(v)];
      }
    }
    const auto smoothed = smooth_box(longitudinal, longitudinal_smoothing_radius);
    std::vector<double> positive;
    for (double value : smoothed) if (value > 0.0) positive.push_back(value);
    if (positive.empty()) continue;
    const double threshold = std::max(static_cast<double>(p.minimum_points_per_support_bin), percentile(positive, 0.90) * 0.10);
    std::vector<bool> active(static_cast<std::size_t>(u_bins), false);
    for (int u = 0; u < u_bins; ++u) active[static_cast<std::size_t>(u)] = smoothed[static_cast<std::size_t>(u)] >= threshold;
    bridge_small_gaps(&active, p.maximum_support_gap_bins);
    int best_start = -1, best_end = -1;
    int cursor = 0;
    while (cursor < u_bins) {
      while (cursor < u_bins && !active[static_cast<std::size_t>(cursor)]) ++cursor;
      const int start = cursor;
      while (cursor < u_bins && active[static_cast<std::size_t>(cursor)]) ++cursor;
      const int end = cursor - 1;
      if (start <= end && (best_start < 0 || end - start > best_end - best_start)) {
        best_start = start;
        best_end = end;
      }
    }
    if (best_start < 0) continue;
    const double length = (best_end - best_start + 1) * p.profile_bin_m;
    int supported_bins = 0;
    for (int u = best_start; u <= best_end; ++u) if (active[static_cast<std::size_t>(u)]) ++supported_bins;
    const double support_fraction = static_cast<double>(supported_bins) / static_cast<double>(best_end - best_start + 1);
    if (length < p.minimum_aisle_length_m || support_fraction < p.minimum_row_support_fraction) continue;
    rows.push_back({peak.bin, best_start, best_end, support_fraction,
                    std::clamp(peak.prominence / std::max(peak.height, 1.0), 0.0, 1.0)});
  }
  result->supported_row_count = static_cast<int>(rows.size());
  if (rows.size() < 2) return true;

  int aisle_index = 0;
  for (std::size_t i = 0; i + 1 < rows.size(); ++i) {
    const auto &left = rows[i];
    const auto &right = rows[i + 1];
    const double left_v = min_v + (left.center_bin + 0.5) * p.profile_bin_m;
    const double right_v = min_v + (right.center_bin + 0.5) * p.profile_bin_m;
    const double low_v = left_v + p.row_half_width_m + p.side_clearance_m;
    const double high_v = right_v - p.row_half_width_m - p.side_clearance_m;
    const double width = high_v - low_v;
    if (width < p.minimum_aisle_width_m || width > p.maximum_aisle_width_m) continue;
    int start_u = std::max(left.start_bin, right.start_bin);
    int end_u = std::min(left.end_bin, right.end_bin);
    if (end_u <= start_u) continue;
    double u0 = min_u + start_u * p.profile_bin_m;
    double u1 = min_u + (end_u + 1) * p.profile_bin_m;
    if (u1 - u0 < p.minimum_aisle_length_m) continue;
    const double middle_v = 0.5 * (low_v + high_v);
    QVector<QPointF> polygon{
        from_uv(u0, low_v, mean_x, mean_y, direction_x, direction_y),
        from_uv(u1, low_v, mean_x, mean_y, direction_x, direction_y),
        from_uv(u1, high_v, mean_x, mean_y, direction_x, direction_y),
        from_uv(u0, high_v, mean_x, mean_y, direction_x, direction_y),
    };
    const auto all_inside = [&]() {
      return std::all_of(polygon.cbegin(), polygon.cend(), [&](const QPointF &point) {
        return inside_polygon(boundary, point.x(), point.y());
      });
    };
    int clip_attempts = 0;
    while (!all_inside() && u1 - u0 > p.minimum_aisle_length_m && clip_attempts++ < 100) {
      u0 += p.profile_bin_m;
      u1 -= p.profile_bin_m;
      polygon = {
          from_uv(u0, low_v, mean_x, mean_y, direction_x, direction_y),
          from_uv(u1, low_v, mean_x, mean_y, direction_x, direction_y),
          from_uv(u1, high_v, mean_x, mean_y, direction_x, direction_y),
          from_uv(u0, high_v, mean_x, mean_y, direction_x, direction_y),
      };
    }
    if (!all_inside()) continue;
    const QPointF start = from_uv(u0, middle_v, mean_x, mean_y, direction_x, direction_y);
    const QPointF end = from_uv(u1, middle_v, mean_x, mean_y, direction_x, direction_y);
    const double length = u1 - u0;
    const double support = 0.5 * (left.support_fraction + right.support_fraction);
    const double ridge = 0.5 * (left.prominence_fraction + right.prominence_fraction);
    AisleGeometryProposal proposal;
    proposal.proposal_id = QStringLiteral("AUTO-A%1").arg(++aisle_index, 3, 10, QLatin1Char('0'));
    proposal.polygon_xy_m = polygon;
    proposal.start_xy_m = start;
    proposal.end_xy_m = end;
    proposal.start_z_m = endpoint_z(roi_points, start.x(), start.y());
    proposal.end_z_m = endpoint_z(roi_points, end.x(), end.y());
    proposal.width_m = width;
    proposal.length_m = length;
    proposal.confidence = std::clamp(support * ridge, 0.0, 1.0);
    result->proposals.push_back(proposal);
  }
  return true;
}

}  // namespace agt_map_studio
