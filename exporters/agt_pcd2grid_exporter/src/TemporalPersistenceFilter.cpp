#include "agt_pcd2grid_exporter/TemporalPersistenceFilter.hpp"

#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace agt_pcd2grid_exporter {
namespace {

struct VoxelKey {
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;
  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelHash {
  std::size_t operator()(const VoxelKey &key) const {
    std::size_t seed = std::hash<std::int64_t>{}(key.x);
    seed ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    seed ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (seed << 6U) + (seed >> 2U);
    return seed;
  }
};

struct Observation {
  std::uint32_t count = 0U;
  std::uint32_t first = 0U;
  std::uint32_t last = 0U;
};

struct StoredPoint {
  pcl::PointXYZI point;
  VoxelKey voxel;
};

VoxelKey voxel_for(const pcl::PointXYZI &point, float size) {
  return {static_cast<std::int64_t>(std::floor(point.x / size)),
          static_cast<std::int64_t>(std::floor(point.y / size)),
          static_cast<std::int64_t>(std::floor(point.z / size))};
}

bool finite(const pcl::PointXYZI &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

}  // namespace

bool TemporalPersistenceFilter::filter_package(
    const std::string &package_dir, const ProjectionParameters &parameters,
    pcl::PCLPointCloud2 *cloud, TemporalFilterStats *stats, std::string *error) {
  if (!cloud || !stats) {
    if (error) *error = "cloud and stats must not be null";
    return false;
  }
  *cloud = pcl::PCLPointCloud2();
  *stats = TemporalFilterStats();
  if (!(parameters.temporal_voxel_size > 0.0F) ||
      parameters.temporal_min_observations == 0U) {
    if (error) *error = "temporal filter parameters are invalid";
    return false;
  }

  const std::filesystem::path root(package_dir);
  std::ifstream poses(root / "poses_timed.txt");
  if (!poses) {
    if (error) *error = "cannot open poses_timed.txt in mapping package";
    return false;
  }

  std::vector<StoredPoint> points;
  std::unordered_map<VoxelKey, Observation, VoxelHash> observations;
  std::string line;
  std::uint32_t keyframe_index = 0U;
  while (std::getline(poses, line)) {
    if (line.empty()) continue;
    std::istringstream record(line);
    std::string patch_name;
    double stamp = 0.0;
    double tx = 0.0;
    double ty = 0.0;
    double tz = 0.0;
    double qw = 1.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    if (!(record >> patch_name >> stamp >> tx >> ty >> tz >> qw >> qx >> qy >> qz)) {
      if (error) *error = "invalid poses_timed.txt record: " + line;
      return false;
    }
    const std::filesystem::path relative(patch_name);
    if (relative.empty() || relative.is_absolute() || relative.has_parent_path()) {
      if (error) *error = "unsafe patch path in poses_timed.txt: " + patch_name;
      return false;
    }
    const auto patch_path = root / "patches" / relative;
    pcl::PointCloud<pcl::PointXYZI> patch;
    if (pcl::io::loadPCDFile(patch_path.string(), patch) != 0) {
      if (error) *error = "cannot load keyframe patch: " + patch_path.string();
      return false;
    }
    Eigen::Quaternionf rotation(static_cast<float>(qw), static_cast<float>(qx),
                                static_cast<float>(qy), static_cast<float>(qz));
    if (rotation.norm() <= std::numeric_limits<float>::epsilon()) {
      if (error) *error = "zero quaternion in poses_timed.txt: " + patch_name;
      return false;
    }
    rotation.normalize();
    const Eigen::Vector3f translation(static_cast<float>(tx), static_cast<float>(ty),
                                      static_cast<float>(tz));
    std::unordered_set<VoxelKey, VoxelHash> observed_by_keyframe;
    observed_by_keyframe.reserve(patch.size());
    for (const auto &source : patch) {
      if (!finite(source)) continue;
      const Eigen::Vector3f transformed =
          rotation * Eigen::Vector3f(source.x, source.y, source.z) + translation;
      pcl::PointXYZI point;
      point.x = transformed.x();
      point.y = transformed.y();
      point.z = transformed.z();
      point.intensity = source.intensity;
      const VoxelKey voxel = voxel_for(point, parameters.temporal_voxel_size);
      points.push_back({point, voxel});
      observed_by_keyframe.insert(voxel);
    }
    for (const auto &voxel : observed_by_keyframe) {
      auto [iterator, inserted] = observations.emplace(
          voxel, Observation{1U, keyframe_index, keyframe_index});
      if (!inserted) {
        ++iterator->second.count;
        iterator->second.last = keyframe_index;
      }
    }
    ++keyframe_index;
  }
  stats->keyframes = keyframe_index;
  stats->input_points = points.size();
  if (keyframe_index == 0U || points.empty()) {
    if (error) *error = "mapping package contains no nonempty keyframe patches";
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI> retained;
  retained.reserve(points.size());
  for (const auto &stored : points) {
    const auto found = observations.find(stored.voxel);
    if (found == observations.end()) continue;
    const Observation &observation = found->second;
    if (observation.count >= parameters.temporal_min_observations &&
        observation.last - observation.first >= parameters.temporal_min_keyframe_span) {
      retained.push_back(stored.point);
    }
  }
  retained.width = static_cast<std::uint32_t>(retained.size());
  retained.height = 1U;
  retained.is_dense = false;
  stats->retained_points = retained.size();
  stats->removed_points = stats->input_points - stats->retained_points;
  if (retained.empty()) {
    if (error) *error = "temporal persistence filter removed every point";
    return false;
  }
  pcl::toPCLPointCloud2(retained, *cloud);
  return true;
}

}  // namespace agt_pcd2grid_exporter
