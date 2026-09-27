#include "agt_spatial_map_core/spatial_evidence.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace agt_spatial_map_core {
namespace {

std::int64_t floor_index(float coordinate, float size) {
  if (!std::isfinite(coordinate)) {
    throw std::invalid_argument("voxel coordinate must be finite");
  }
  // The float32 division deliberately matches the existing temporal filter.
  const float divided = coordinate / size;
  const long double floored = std::floor(static_cast<long double>(divided));
  if (!std::isfinite(divided) ||
      floored < static_cast<long double>(std::numeric_limits<std::int64_t>::min()) ||
      floored >= static_cast<long double>(std::numeric_limits<std::int64_t>::max())) {
    throw std::out_of_range("voxel index exceeds int64 range");
  }
  return static_cast<std::int64_t>(floored);
}

void unit_interval(float value, const char *name) {
  if (!std::isfinite(value) || value < 0.0F || value > 1.0F) {
    throw std::invalid_argument(std::string(name) + " must be finite and in [0,1]");
  }
}

}  // namespace

std::size_t VoxelKeyHash::operator()(const VoxelKey &key) const noexcept {
  std::size_t seed = std::hash<std::int64_t>{}(key.x);
  seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
  return seed;
}

VoxelKey voxel_for(const Eigen::Vector3f &point, float voxel_size) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0F) {
    throw std::invalid_argument("voxel_size must be finite and positive");
  }
  return {floor_index(point.x(), voxel_size),
          floor_index(point.y(), voxel_size),
          floor_index(point.z(), voxel_size)};
}

void validate_parameters(const ConfidenceParameters &p) {
  if (!std::isfinite(p.voxel_size) || p.voxel_size <= 0.0F ||
      !std::isfinite(p.observation_reference) || p.observation_reference <= 0.0F ||
      !std::isfinite(p.keyframe_span_reference) || p.keyframe_span_reference <= 0.0F) {
    throw std::invalid_argument("voxel_size and confidence references must be finite and positive");
  }
  unit_interval(p.persistence_alpha, "persistence_alpha");
  unit_interval(p.stable_threshold, "stable_threshold");
  unit_interval(p.force_low_value, "force_low_value");
}

void calculate_confidence(SpatialVoxelEvidence *evidence, const ConfidenceParameters &p) {
  if (!evidence) throw std::invalid_argument("evidence must not be null");
  validate_parameters(p);
  auto &v = *evidence;
  unit_interval(v.geometry_score, "geometry_score");
  if (v.point_count == 0U || v.observed_keyframes == 0U ||
      v.observed_keyframes > v.point_count || v.last_keyframe < v.first_keyframe ||
      v.keyframe_span != v.last_keyframe - v.first_keyframe) {
    throw std::invalid_argument("invalid voxel observation evidence");
  }
  const double n = static_cast<double>(v.observed_keyframes);
  const double s = static_cast<double>(v.keyframe_span);
  const double q_obs = -std::expm1(-n / p.observation_reference);
  const double q_span = std::min(1.0, s / p.keyframe_span_reference);
  const double q_persist = std::pow(q_obs, p.persistence_alpha) *
                           std::pow(q_span, 1.0F - p.persistence_alpha);
  v.observation_score = static_cast<float>(q_obs);
  v.persistence_score = static_cast<float>(q_persist);
  v.auto_confidence = static_cast<float>(q_persist * v.geometry_score);
  switch (v.override_mode) {
    case ManualOverrideMode::AUTO:
      v.final_confidence = v.auto_confidence;
      break;
    case ManualOverrideMode::FORCE_HIGH:
      v.final_confidence = 1.0F;
      break;
    case ManualOverrideMode::FORCE_LOW:
      if (v.has_manual_value) unit_interval(v.manual_value, "manual_value");
      v.final_confidence = v.has_manual_value ? v.manual_value : p.force_low_value;
      break;
    case ManualOverrideMode::IGNORE:
      v.final_confidence = 0.0F;
      break;
    default:
      throw std::invalid_argument("unsupported manual override mode");
  }
}

const char *override_mode_name(ManualOverrideMode mode) {
  switch (mode) {
    case ManualOverrideMode::AUTO: return "AUTO";
    case ManualOverrideMode::FORCE_HIGH: return "FORCE_HIGH";
    case ManualOverrideMode::FORCE_LOW: return "FORCE_LOW";
    case ManualOverrideMode::IGNORE: return "IGNORE";
    default: throw std::invalid_argument("unsupported manual override mode");
  }
}

ManualOverrideMode parse_override_mode(const std::string &name) {
  if (name == "AUTO") return ManualOverrideMode::AUTO;
  if (name == "FORCE_HIGH") return ManualOverrideMode::FORCE_HIGH;
  if (name == "FORCE_LOW") return ManualOverrideMode::FORCE_LOW;
  if (name == "IGNORE") return ManualOverrideMode::IGNORE;
  throw std::invalid_argument("unsupported manual override mode: " + name);
}

}  // namespace agt_spatial_map_core
