#include "agt_spatial_map_core/spatial_evidence_builder.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace agt_spatial_map_core {
namespace {

struct Accumulator {
  SpatialVoxelEvidence evidence;
  double x_sum = 0.0;
  double y_sum = 0.0;
  double z_sum = 0.0;
};

bool finite(const pcl::PointXYZI &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

}  // namespace

SpatialEvidenceMap SpatialEvidenceBuilder::build(
    const std::filesystem::path &map_package, const ConfidenceParameters &parameters,
    EvidenceBuildStats *stats) {
  validate_parameters(parameters);
  EvidenceBuildStats result_stats;
  if (!std::filesystem::is_directory(map_package)) {
    throw std::runtime_error("mapping package directory missing: " + map_package.string());
  }
  std::ifstream pose_stream(map_package / "poses_timed.txt");
  if (!pose_stream) {
    throw std::runtime_error("mapping package missing poses_timed.txt; a navigation release "
                             "without patches is not a mapping input");
  }
  if (!std::filesystem::is_directory(map_package / "patches")) {
    throw std::runtime_error("mapping package missing patches/");
  }

  std::unordered_map<VoxelKey, Accumulator, VoxelKeyHash> accumulators;
  std::unordered_set<std::string> patch_names;
  std::string line;
  std::uint64_t index = 0;
  while (std::getline(pose_stream, line)) {
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    if (index >= std::numeric_limits<std::uint32_t>::max()) {
      throw std::runtime_error("too many mapping keyframes for uint32 evidence indices");
    }
    std::istringstream record(line);
    std::string name;
    double stamp, tx, ty, tz, qw, qx, qy, qz;
    if (!(record >> name >> stamp >> tx >> ty >> tz >> qw >> qx >> qy >> qz)) {
      throw std::runtime_error("invalid poses_timed.txt record at keyframe " +
                               std::to_string(index));
    }
    std::string trailing;
    if (record >> trailing) {
      throw std::runtime_error("unexpected trailing token in poses_timed.txt at keyframe " +
                               std::to_string(index));
    }
    const std::filesystem::path relative(name);
    if (relative.empty() || relative.is_absolute() || relative.has_parent_path() ||
        relative == "." || relative == "..") {
      throw std::runtime_error("unsafe patch filename in poses_timed.txt: " + name);
    }
    if (!patch_names.insert(name).second) {
      throw std::runtime_error("duplicate patch filename cannot count as a new keyframe: " + name);
    }
    if (!std::isfinite(stamp) || !std::isfinite(tx) || !std::isfinite(ty) ||
        !std::isfinite(tz) || !std::isfinite(qw) || !std::isfinite(qx) ||
        !std::isfinite(qy) || !std::isfinite(qz)) {
      throw std::runtime_error("non-finite optimized pose at keyframe " + std::to_string(index));
    }
    Eigen::Quaternionf rotation(static_cast<float>(qw), static_cast<float>(qx),
                                static_cast<float>(qy), static_cast<float>(qz));
    const Eigen::Vector3f translation(static_cast<float>(tx), static_cast<float>(ty),
                                      static_cast<float>(tz));
    if (!rotation.coeffs().allFinite() || !translation.allFinite() ||
        !(rotation.norm() > std::numeric_limits<float>::epsilon())) {
      throw std::runtime_error("invalid T_map_body at keyframe " + std::to_string(index));
    }
    rotation.normalize();
    const auto patch_path = map_package / "patches" / relative;
    if (std::filesystem::is_symlink(patch_path)) {
      throw std::runtime_error("mapping patch may not be a symbolic link: " + patch_path.string());
    }
    pcl::PointCloud<pcl::PointXYZI> patch;
    if (pcl::io::loadPCDFile(patch_path.string(), patch) != 0) {
      throw std::runtime_error("cannot load keyframe patch: " + patch_path.string());
    }
    for (const auto &source : patch) {
      ++result_stats.input_points;
      if (!finite(source)) {
        ++result_stats.nonfinite_points;
        continue;
      }
      const Eigen::Vector3f transformed =
          rotation * Eigen::Vector3f(source.x, source.y, source.z) + translation;
      if (!transformed.allFinite()) {
        ++result_stats.nonfinite_points;
        continue;
      }
      ++result_stats.usable_points;
      const auto key = voxel_for(transformed, parameters.voxel_size);
      auto &a = accumulators[key];
      auto &v = a.evidence;
      if (v.point_count == std::numeric_limits<std::uint32_t>::max() ||
          v.observed_keyframes == std::numeric_limits<std::uint32_t>::max()) {
        throw std::runtime_error("voxel evidence count exceeds uint32 capacity");
      }
      v.key = key;
      ++v.point_count;
      if (v.observed_keyframes == 0U || v.last_keyframe != index) {
        if (v.observed_keyframes == 0U) v.first_keyframe = static_cast<std::uint32_t>(index);
        v.last_keyframe = static_cast<std::uint32_t>(index);
        ++v.observed_keyframes;
      }
      a.x_sum += transformed.x();
      a.y_sum += transformed.y();
      a.z_sum += transformed.z();
    }
    ++index;
  }
  if (index == 0 || result_stats.usable_points == 0 || accumulators.empty()) {
    throw std::runtime_error("mapping package contains no usable keyframe evidence");
  }
  result_stats.source_keyframes = static_cast<std::uint32_t>(index);
  result_stats.voxel_count = accumulators.size();

  SpatialEvidenceMap result;
  result.reserve(accumulators.size());
  for (auto &[key, a] : accumulators) {
    auto &v = a.evidence;
    const double n = static_cast<double>(v.point_count);
    v.centroid = Eigen::Vector3f(static_cast<float>(a.x_sum / n),
                                 static_cast<float>(a.y_sum / n),
                                 static_cast<float>(a.z_sum / n));
    if (!v.centroid.allFinite()) {
      throw std::runtime_error("voxel centroid exceeds float32 coordinate range");
    }
    v.keyframe_span = v.last_keyframe - v.first_keyframe;
    v.geometry_score = 1.0F;  // No single-session geometry estimator in V1.
    calculate_confidence(&v, parameters);
    result.emplace(key, std::move(v));
  }
  if (stats) *stats = result_stats;
  return result;
}

}  // namespace agt_spatial_map_core
