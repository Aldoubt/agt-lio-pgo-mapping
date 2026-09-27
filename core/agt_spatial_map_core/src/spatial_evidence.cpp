#include "agt_spatial_map_core/spatial_evidence.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <regex>
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

void validate_manual_override_audit(const ManualOverrideAudit &audit) {
  if (!audit.reason.empty()) {
    constexpr std::array<const char *, 8> kReasons{{
        "PARKING_AREA", "VEGETATION", "TEMPORARY_OBJECT", "CONSTRUCTION",
        "MOVING_OBJECT_PRONE", "LOW_GEOMETRY", "MANUAL_ANCHOR", "OTHER"}};
    if (std::none_of(kReasons.begin(), kReasons.end(), [&audit](const char *tag) {
          return audit.reason == tag;
        })) {
      throw std::invalid_argument("invalid manual override reason tag");
    }
  }
  if (!audit.edited_at.empty()) {
    static const std::regex utc(R"(^[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z$)");
    if (!std::regex_match(audit.edited_at, utc)) {
      throw std::invalid_argument("manual override edited_at must be UTC ISO-8601 seconds");
    }
    const int year = std::stoi(audit.edited_at.substr(0, 4));
    const int month = std::stoi(audit.edited_at.substr(5, 2));
    const int day = std::stoi(audit.edited_at.substr(8, 2));
    const int hour = std::stoi(audit.edited_at.substr(11, 2));
    const int minute = std::stoi(audit.edited_at.substr(14, 2));
    const int second = std::stoi(audit.edited_at.substr(17, 2));
    constexpr std::array<int, 12> kDays{{31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}};
    const bool leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    if (year == 0 || month < 1 || month > 12 || day < 1 ||
        day > kDays.at(static_cast<std::size_t>(month - 1)) + (month == 2 && leap ? 1 : 0) ||
        hour > 23 || minute > 59 || second > 59) {
      throw std::invalid_argument("manual override edited_at is not a valid UTC timestamp");
    }
  }
  if (!audit.editor.empty()) {
    static const std::regex name(R"(^[A-Za-z0-9_.@-]{1,64}$)");
    if (!std::regex_match(audit.editor, name)) {
      throw std::invalid_argument("manual override editor must be 1-64 safe ASCII characters");
    }
  }
}

float manual_final_confidence(float auto_confidence, ManualOverrideMode mode,
                              bool has_manual_value, float manual_value,
                              float force_low_value) {
  unit_interval(auto_confidence, "auto_confidence");
  unit_interval(force_low_value, "force_low_value");
  // The Phase 1 calculator ignored a stale manual value when the mode was
  // changed away from FORCE_LOW. Keep that behavior; YAML parser separately
  // rejects an explicit value for any other mode.
  switch (mode) {
    case ManualOverrideMode::AUTO: return auto_confidence;
    case ManualOverrideMode::FORCE_HIGH: return 1.0F;
    case ManualOverrideMode::FORCE_LOW:
      if (has_manual_value) unit_interval(manual_value, "manual_value");
      return has_manual_value ? manual_value : force_low_value;
    case ManualOverrideMode::IGNORE: return 0.0F;
    default: throw std::invalid_argument("unsupported manual override mode");
  }
}

bool stable_preview_selected(float final_confidence, ManualOverrideMode mode,
                             float stable_threshold) {
  unit_interval(final_confidence, "final_confidence");
  unit_interval(stable_threshold, "stable_threshold");
  switch (mode) {
    case ManualOverrideMode::AUTO:
    case ManualOverrideMode::FORCE_HIGH: return final_confidence >= stable_threshold;
    case ManualOverrideMode::FORCE_LOW:
    case ManualOverrideMode::IGNORE: return false;
    default: throw std::invalid_argument("unsupported manual override mode");
  }
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
  v.final_confidence = manual_final_confidence(
      v.auto_confidence, v.override_mode, v.has_manual_value,
      v.manual_value, p.force_low_value);
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
