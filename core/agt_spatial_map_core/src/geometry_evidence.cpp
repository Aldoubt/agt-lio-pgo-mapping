#include "agt_spatial_map_core/geometry_evidence.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>
#include <string>

namespace agt_spatial_map_core {
namespace {
namespace fs = std::filesystem;

void keys(const YAML::Node &node, const std::set<std::string> &expected) {
  if (!node || !node.IsMap() || node.size() != expected.size()) {
    throw std::invalid_argument("geometry config: missing/extra YAML mapping keys");
  }
  std::set<std::string> seen;
  for (const auto &item : node) {
    const auto key = item.first.as<std::string>();
    if (!expected.count(key) || !seen.insert(key).second) {
      throw std::invalid_argument("geometry config: duplicate or unknown key: " + key);
    }
  }
}
}  // namespace

void validate_geometry_parameters(const GeometryParameters &p) {
  if (!std::isfinite(p.normal_radius) || p.normal_radius <= 0.0F ||
      p.normal_min_points < 3 || !std::isfinite(p.observability_radius) ||
      p.observability_radius <= 0.0F || p.min_valid_normals < 3 ||
      !std::isfinite(p.normal_epsilon_m2) || p.normal_epsilon_m2 <= 0.0 ||
      !std::isfinite(p.translation_epsilon) || p.translation_epsilon <= 0.0 ||
      !std::isfinite(p.rotation_epsilon_m2) || p.rotation_epsilon_m2 <= 0.0) {
    throw std::invalid_argument("geometry radii/epsilon must be finite positive, "
                                "normal_min_points/min_valid_normals >= 3");
  }
}

GeometryParameters load_geometry_config(const fs::path &path) {
  GeometryParameters p;
  if (path.empty()) return p;
  if (fs::is_symlink(fs::symlink_status(path)) || !fs::is_regular_file(path) ||
      fs::file_size(path) == 0) {
    throw std::invalid_argument("geometry config must be a nonempty regular file, not a symlink");
  }
  const YAML::Node root = YAML::LoadFile(path.string());
  keys(root, {"schema_version", "normal", "observability", "epsilon"});
  keys(root["normal"], {"radius", "min_points"});
  keys(root["observability"], {"radius", "min_valid_normals"});
  keys(root["epsilon"], {"normal_covariance_m2", "translation", "rotation_m2"});
  if (root["schema_version"].as<int>() != 1) {
    throw std::invalid_argument("unsupported geometry config schema_version");
  }
  p.normal_radius = root["normal"]["radius"].as<float>();
  p.normal_min_points = root["normal"]["min_points"].as<std::uint32_t>();
  p.observability_radius = root["observability"]["radius"].as<float>();
  p.min_valid_normals = root["observability"]["min_valid_normals"].as<std::uint32_t>();
  p.normal_epsilon_m2 = root["epsilon"]["normal_covariance_m2"].as<double>();
  p.translation_epsilon = root["epsilon"]["translation"].as<double>();
  p.rotation_epsilon_m2 = root["epsilon"]["rotation_m2"].as<double>();
  validate_geometry_parameters(p);
  return p;
}

}  // namespace agt_spatial_map_core
