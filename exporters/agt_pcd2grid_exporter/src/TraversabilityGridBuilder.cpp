#include "agt_pcd2grid_exporter/TraversabilityGridBuilder.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace agt_pcd2grid_exporter {
namespace {

constexpr float kUnknownHeight = std::numeric_limits<float>::quiet_NaN();

enum GroundSource : std::uint8_t { kNone = 0U, kInferred = 1U, kObserved = 2U, kSeed = 3U };
enum PointKind : std::uint8_t { kUnclassified = 0U, kGround = 1U, kObstacle = 2U };

struct Keyframe {
  Eigen::Isometry3f map_from_body = Eigen::Isometry3f::Identity();
  Eigen::Vector3f base = Eigen::Vector3f::Zero();
  float base_yaw = 0.0F;
  float base_tilt = 0.0F;  // angle between base_footprint z and map z [rad]
};

struct MapPoint {
  float x;
  float y;
  float z;
  std::uint32_t keyframe;
  std::uint8_t persistent;
  std::uint8_t kind;
};

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
  bool far = false;  // seen at least once from >= obstacle_min_observation_range
};

struct TrajectorySample {
  float x;
  float y;
  float z;
  float yaw;
};

Eigen::Isometry3f base_from_body_transform(const RobotModel &robot) {
  const auto &rpy = robot.base_from_body_rpy;
  const auto &xyz = robot.base_from_body_xyz;
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.linear() = (Eigen::AngleAxisf(rpy[2], Eigen::Vector3f::UnitZ()) *
                        Eigen::AngleAxisf(rpy[1], Eigen::Vector3f::UnitY()) *
                        Eigen::AngleAxisf(rpy[0], Eigen::Vector3f::UnitX()))
                           .toRotationMatrix();
  transform.translation() = Eigen::Vector3f(xyz[0], xyz[1], xyz[2]);
  return transform;
}

bool point_in_polygon(float x, float y, const std::vector<std::pair<float, float>> &polygon) {
  bool inside = false;
  for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const float xi = polygon[i].first;
    const float yi = polygon[i].second;
    const float xj = polygon[j].first;
    const float yj = polygon[j].second;
    if (((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) inside = !inside;
  }
  return inside;
}

// Offsets every vertex away from (delta > 0) or towards (delta < 0) the polygon
// centroid by |delta| per axis; exact for axis-aligned rectangles, a close
// approximation for other convex footprints.  Shrinking stops at the centroid.
std::vector<std::pair<float, float>> offset_polygon(
    const std::vector<std::pair<float, float>> &polygon, float delta) {
  float cx = 0.0F;
  float cy = 0.0F;
  for (const auto &[x, y] : polygon) {
    cx += x;
    cy += y;
  }
  cx /= static_cast<float>(polygon.size());
  cy /= static_cast<float>(polygon.size());
  const auto offset = [delta](float d) {
    if (d == 0.0F) return 0.0F;
    const float magnitude = std::max(0.0F, std::abs(d) + delta);
    return std::copysign(magnitude, d);
  };
  std::vector<std::pair<float, float>> result;
  for (const auto &[x, y] : polygon) result.emplace_back(cx + offset(x - cx), cy + offset(y - cy));
  return result;
}

std::size_t remove_small_components(std::vector<std::uint8_t> *mask, std::uint32_t width,
                                    std::uint32_t height, std::uint32_t minimum_size) {
  if (!mask || minimum_size <= 1U) return 0U;
  std::vector<std::uint8_t> visited(mask->size(), 0U);
  std::vector<std::size_t> component;
  std::vector<std::size_t> stack;
  std::size_t removed = 0U;
  for (std::size_t start = 0; start < mask->size(); ++start) {
    if (!(*mask)[start] || visited[start]) continue;
    component.clear();
    stack.assign(1U, start);
    visited[start] = 1U;
    while (!stack.empty()) {
      const std::size_t index = stack.back();
      stack.pop_back();
      component.push_back(index);
      const auto cx = static_cast<std::int64_t>(index % width);
      const auto cy = static_cast<std::int64_t>(index / width);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          const auto nx = cx + dx;
          const auto ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
          const std::size_t next = static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx);
          if ((*mask)[next] && !visited[next]) {
            visited[next] = 1U;
            stack.push_back(next);
          }
        }
      }
    }
    if (component.size() < minimum_size) {
      removed += component.size();
      for (const auto index : component) (*mask)[index] = 0U;
    }
  }
  return removed;
}

std::vector<std::uint8_t> morph(const std::vector<std::uint8_t> &source, std::uint32_t width,
                                std::uint32_t height, std::uint32_t radius, bool dilate) {
  // Separable square structuring element; out-of-grid cells count as empty.
  if (radius == 0U) return source;
  const int r = static_cast<int>(radius);
  std::vector<std::uint8_t> pass(source.size(), 0U);
  std::vector<std::uint8_t> result(source.size(), 0U);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      bool value = !dilate;
      for (int d = -r; d <= r; ++d) {
        const auto nx = static_cast<std::int64_t>(x) + d;
        const bool cell = nx >= 0 && nx < width &&
                          source[static_cast<std::size_t>(y) * width + static_cast<std::size_t>(nx)];
        if (dilate && cell) { value = true; break; }
        if (!dilate && !cell) { value = false; break; }
      }
      pass[static_cast<std::size_t>(y) * width + x] = value ? 1U : 0U;
    }
  }
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      bool value = !dilate;
      for (int d = -r; d <= r; ++d) {
        const auto ny = static_cast<std::int64_t>(y) + d;
        const bool cell = ny >= 0 && ny < height &&
                          pass[static_cast<std::size_t>(ny) * width + x];
        if (dilate && cell) { value = true; break; }
        if (!dilate && !cell) { value = false; break; }
      }
      result[static_cast<std::size_t>(y) * width + x] = value ? 1U : 0U;
    }
  }
  return result;
}

bool load_keyframes(const std::filesystem::path &root, const Eigen::Isometry3f &base_from_body,
                    std::vector<Keyframe> *keyframes, std::vector<std::filesystem::path> *patches,
                    std::string *error) {
  std::ifstream poses(root / "poses_timed.txt");
  if (!poses) {
    if (error) *error = "cannot open poses_timed.txt in mapping package";
    return false;
  }
  const Eigen::Isometry3f body_from_base = base_from_body.inverse();
  std::string line;
  while (std::getline(poses, line)) {
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
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
    Eigen::Quaternionf rotation(static_cast<float>(qw), static_cast<float>(qx),
                                static_cast<float>(qy), static_cast<float>(qz));
    if (!std::isfinite(tx) || !std::isfinite(ty) || !std::isfinite(tz) ||
        !(rotation.norm() > std::numeric_limits<float>::epsilon())) {
      if (error) *error = "invalid pose in poses_timed.txt: " + patch_name;
      return false;
    }
    rotation.normalize();
    Keyframe keyframe;
    keyframe.map_from_body.linear() = rotation.toRotationMatrix();
    keyframe.map_from_body.translation() =
        Eigen::Vector3f(static_cast<float>(tx), static_cast<float>(ty), static_cast<float>(tz));
    const Eigen::Isometry3f map_from_base = keyframe.map_from_body * body_from_base;
    keyframe.base = map_from_base.translation();
    keyframe.base_yaw = std::atan2(map_from_base.linear()(1, 0), map_from_base.linear()(0, 0));
    keyframe.base_tilt = std::acos(std::clamp(map_from_base.linear()(2, 2), -1.0F, 1.0F));
    keyframes->push_back(keyframe);
    patches->push_back(root / "patches" / relative);
  }
  if (keyframes->empty()) {
    if (error) *error = "mapping package has no keyframes";
    return false;
  }
  return true;
}

std::vector<TrajectorySample> densify(const std::vector<Keyframe> &keyframes, float step,
                                      float max_gap) {
  std::vector<TrajectorySample> samples;
  samples.push_back({keyframes.front().base.x(), keyframes.front().base.y(),
                     keyframes.front().base.z(), keyframes.front().base_yaw});
  for (std::size_t i = 1; i < keyframes.size(); ++i) {
    const auto &a = keyframes[i - 1];
    const auto &b = keyframes[i];
    const float distance = (b.base.head<2>() - a.base.head<2>()).norm();
    const float yaw_delta = std::atan2(std::sin(b.base_yaw - a.base_yaw),
                                       std::cos(b.base_yaw - a.base_yaw));
    if (distance > max_gap) {
      samples.push_back({b.base.x(), b.base.y(), b.base.z(), b.base_yaw});
      continue;
    }
    const int count = std::max(1, static_cast<int>(std::ceil(
                                      std::max(distance / step, std::abs(yaw_delta) / 0.05F))));
    for (int k = 1; k <= count; ++k) {
      const float f = static_cast<float>(k) / static_cast<float>(count);
      const Eigen::Vector3f p = a.base + f * (b.base - a.base);
      samples.push_back({p.x(), p.y(), p.z(), a.base_yaw + f * yaw_delta});
    }
  }
  return samples;
}

// Axis-aligned raster helper used for both the coarse ground grid and the
// fine occupancy grid.
struct Raster {
  double origin_x = 0.0;
  double origin_y = 0.0;
  double resolution = 1.0;
  std::uint32_t width = 0U;
  std::uint32_t height = 0U;

  bool cell(double x, double y, std::int64_t *cx, std::int64_t *cy) const {
    *cx = static_cast<std::int64_t>(std::floor((x - origin_x) / resolution));
    *cy = static_cast<std::int64_t>(std::floor((y - origin_y) / resolution));
    return *cx >= 0 && *cy >= 0 && *cx < width && *cy < height;
  }
  std::size_t index(std::int64_t cx, std::int64_t cy) const {
    return static_cast<std::size_t>(cy) * width + static_cast<std::size_t>(cx);
  }
  std::size_t size() const { return static_cast<std::size_t>(width) * height; }
};

template <typename Visitor>
void rasterize_footprints(const std::vector<TrajectorySample> &samples,
                          const std::vector<std::pair<float, float>> &footprint,
                          const Raster &raster, Visitor visitor) {
  float radius = 0.0F;
  for (const auto &[x, y] : footprint) radius = std::max(radius, std::hypot(x, y));
  for (const auto &sample : samples) {
    const float c = std::cos(sample.yaw);
    const float s = std::sin(sample.yaw);
    std::int64_t x0 = 0;
    std::int64_t y0 = 0;
    std::int64_t x1 = 0;
    std::int64_t y1 = 0;
    raster.cell(sample.x - radius, sample.y - radius, &x0, &y0);
    raster.cell(sample.x + radius, sample.y + radius, &x1, &y1);
    for (std::int64_t cy = std::max<std::int64_t>(y0, 0);
         cy <= std::min<std::int64_t>(y1, static_cast<std::int64_t>(raster.height) - 1); ++cy) {
      for (std::int64_t cx = std::max<std::int64_t>(x0, 0);
           cx <= std::min<std::int64_t>(x1, static_cast<std::int64_t>(raster.width) - 1); ++cx) {
        const double wx = raster.origin_x + (static_cast<double>(cx) + 0.5) * raster.resolution;
        const double wy = raster.origin_y + (static_cast<double>(cy) + 0.5) * raster.resolution;
        const float dx = static_cast<float>(wx) - sample.x;
        const float dy = static_cast<float>(wy) - sample.y;
        const float lx = c * dx + s * dy;
        const float ly = -s * dx + c * dy;
        if (point_in_polygon(lx, ly, footprint)) visitor(raster.index(cx, cy), sample);
      }
    }
  }
}

bool write_pgm(const std::filesystem::path &path, const std::vector<std::uint8_t> &pixels,
               std::uint32_t width, std::uint32_t height) {
  std::ofstream stream(path, std::ios::binary);
  if (!stream) return false;
  stream << "P5\n" << width << ' ' << height << "\n255\n";
  for (std::int64_t y = static_cast<std::int64_t>(height) - 1; y >= 0; --y) {
    stream.write(reinterpret_cast<const char *>(pixels.data() + static_cast<std::size_t>(y) * width),
                 width);
  }
  return stream.good();
}

}  // namespace

bool TraversabilityGridBuilder::build(const std::string &package_dir,
                                      const ProjectionParameters &parameters,
                                      OccupancyGrid *grid, ProjectionStats *stats,
                                      TraversabilityStats *traversability, std::string *error,
                                      const std::string &debug_dir) {
  if (!grid || !stats || !traversability) {
    if (error) *error = "grid, stats and traversability stats must not be null";
    return false;
  }
  *grid = OccupancyGrid();
  *stats = ProjectionStats();
  *traversability = TraversabilityStats();
  const TraversabilityParameters &tp = parameters.traversability;
  if (parameters.robot.footprint.size() < 3U) {
    if (error) *error = "robot.footprint needs at least three vertices";
    return false;
  }

  // ---- 1. keyframes, self filter, temporal persistence ----------------------
  const Eigen::Isometry3f base_from_body = base_from_body_transform(parameters.robot);
  std::vector<Keyframe> keyframes;
  std::vector<std::filesystem::path> patch_paths;
  if (!load_keyframes(std::filesystem::path(package_dir), base_from_body, &keyframes,
                      &patch_paths, error)) {
    return false;
  }
  traversability->keyframes = keyframes.size();
  {
    constexpr float kDegPerRad = 57.29577951F;
    const float limit = tp.max_keyframe_tilt_deg / kDegPerRad;
    std::size_t tilted = 0U;
    std::size_t first_tilted = 0U;
    float worst = 0.0F;
    for (std::size_t k = 0; k < keyframes.size(); ++k) {
      if (keyframes[k].base_tilt > limit) {
        if (tilted == 0U) first_tilted = k;
        ++tilted;
      }
      worst = std::max(worst, keyframes[k].base_tilt);
    }
    if (tilted > 0U) {
      std::ostringstream message;
      message << tilted << " of " << keyframes.size() << " keyframes have a base tilt above "
              << tp.max_keyframe_tilt_deg << " deg (first: keyframe " << first_tilted
              << ", worst: " << worst * kDegPerRad
              << " deg); the mapping run looks diverged or the map frame is not gravity "
                 "aligned. Re-run mapping or check robot.base_from_body.";
      if (error) *error = message.str();
      return false;
    }
  }
  float fp_min_x = std::numeric_limits<float>::max();
  float fp_max_x = std::numeric_limits<float>::lowest();
  float fp_min_y = std::numeric_limits<float>::max();
  float fp_max_y = std::numeric_limits<float>::lowest();
  for (const auto &[x, y] : parameters.robot.footprint) {
    fp_min_x = std::min(fp_min_x, x);
    fp_max_x = std::max(fp_max_x, x);
    fp_min_y = std::min(fp_min_y, y);
    fp_max_y = std::max(fp_max_y, y);
  }
  const float margin = tp.self_filter_margin;

  std::vector<MapPoint> points;
  std::unordered_map<VoxelKey, Observation, VoxelHash> observations;
  const float voxel = parameters.temporal_voxel_size;
  const float near_field = tp.obstacle_min_observation_range;
  const bool track_voxels = parameters.temporal_filter_enabled || near_field > 0.0F;
  for (std::uint32_t k = 0; k < keyframes.size(); ++k) {
    pcl::PointCloud<pcl::PointXYZ> patch;
    if (pcl::io::loadPCDFile(patch_paths[k].string(), patch) != 0) {
      if (error) *error = "cannot load keyframe patch: " + patch_paths[k].string();
      return false;
    }
    std::vector<VoxelKey> seen;
    std::vector<VoxelKey> seen_far;
    for (const auto &point : patch.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      ++stats->input_points;
      const Eigen::Vector3f body(point.x, point.y, point.z);
      if (tp.self_filter_enabled) {
        const Eigen::Vector3f base = base_from_body * body;
        if (base.x() >= fp_min_x - margin && base.x() <= fp_max_x + margin &&
            base.y() >= fp_min_y - margin && base.y() <= fp_max_y + margin &&
            base.z() >= tp.self_filter_min_height && base.z() <= tp.self_filter_max_height) {
          ++traversability->self_filtered_points;
          continue;
        }
      }
      const Eigen::Vector3f world = keyframes[k].map_from_body * body;
      points.push_back({world.x(), world.y(), world.z(), k, 0U, kUnclassified});
      if (track_voxels) {
        const VoxelKey key{static_cast<std::int64_t>(std::floor(world.x() / voxel)),
                           static_cast<std::int64_t>(std::floor(world.y() / voxel)),
                           static_cast<std::int64_t>(std::floor(world.z() / voxel))};
        seen.push_back(key);
        if (near_field > 0.0F && body.norm() >= near_field) seen_far.push_back(key);
      }
    }
    if (track_voxels) {
      const auto less = [](const VoxelKey &a, const VoxelKey &b) {
        return std::tie(a.x, a.y, a.z) < std::tie(b.x, b.y, b.z);
      };
      std::sort(seen.begin(), seen.end(), less);
      seen.erase(std::unique(seen.begin(), seen.end()), seen.end());
      for (const auto &key : seen) {
        auto &observation = observations[key];
        if (observation.count == 0U) observation.first = k;
        observation.last = k;
        ++observation.count;
      }
      for (const auto &key : seen_far) observations[key].far = true;
    }
  }
  if (points.empty()) {
    if (error) *error = "mapping package contains no usable points";
    return false;
  }
  std::size_t persistent_points = 0U;
  for (auto &point : points) {
    bool persistent = true;
    if (track_voxels) {
      const VoxelKey key{static_cast<std::int64_t>(std::floor(point.x / voxel)),
                         static_cast<std::int64_t>(std::floor(point.y / voxel)),
                         static_cast<std::int64_t>(std::floor(point.z / voxel))};
      const auto &observation = observations[key];
      if (parameters.temporal_filter_enabled) {
        persistent = observation.count >= parameters.temporal_min_observations &&
                     observation.last - observation.first >= parameters.temporal_min_keyframe_span;
      }
      if (persistent && near_field > 0.0F && !observation.far) {
        persistent = false;
        ++traversability->near_field_only_points;
      }
    }
    point.persistent = persistent ? 1U : 0U;
    if (persistent) ++persistent_points;
  }
  observations.clear();
  stats->temporal_input_points = parameters.temporal_filter_enabled ? points.size() : 0U;
  stats->temporal_retained_points = parameters.temporal_filter_enabled ? persistent_points : 0U;
  stats->temporal_removed_points =
      parameters.temporal_filter_enabled ? points.size() - persistent_points : 0U;

  const std::vector<TrajectorySample> trajectory = densify(keyframes, 0.05F, 1.5F);

  // ---- 2. coarse ground surface grown from the trajectory -------------------
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  for (const auto &point : points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }
  for (const auto &sample : trajectory) {
    min_x = std::min(min_x, sample.x - 1.0F);
    max_x = std::max(max_x, sample.x + 1.0F);
    min_y = std::min(min_y, sample.y - 1.0F);
    max_y = std::max(max_y, sample.y + 1.0F);
  }
  Raster coarse;
  coarse.resolution = tp.ground_cell_size;
  coarse.origin_x = std::floor(min_x / tp.ground_cell_size) * tp.ground_cell_size;
  coarse.origin_y = std::floor(min_y / tp.ground_cell_size) * tp.ground_cell_size;
  coarse.width = static_cast<std::uint32_t>(std::floor((max_x - coarse.origin_x) / coarse.resolution)) + 1U;
  coarse.height = static_cast<std::uint32_t>(std::floor((max_y - coarse.origin_y) / coarse.resolution)) + 1U;
  if (static_cast<std::uint64_t>(coarse.width) * coarse.height > 400000000ULL) {
    if (error) *error = "ground grid is too large; check the mapping package extent";
    return false;
  }
  std::vector<std::uint32_t> coarse_count(coarse.size() + 1U, 0U);
  std::vector<std::uint32_t> point_coarse(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    std::int64_t cx = 0;
    std::int64_t cy = 0;
    coarse.cell(points[i].x, points[i].y, &cx, &cy);
    point_coarse[i] = static_cast<std::uint32_t>(coarse.index(cx, cy));
    ++coarse_count[point_coarse[i] + 1U];
  }
  for (std::size_t i = 1; i < coarse_count.size(); ++i) coarse_count[i] += coarse_count[i - 1];
  std::vector<float> z_values(points.size());
  {
    std::vector<std::uint32_t> cursor(coarse_count.begin(), coarse_count.end() - 1);
    for (std::size_t i = 0; i < points.size(); ++i) z_values[cursor[point_coarse[i]]++] = points[i].z;
  }
  std::vector<float> candidate(coarse.size(), kUnknownHeight);
  for (std::size_t c = 0; c < coarse.size(); ++c) {
    const std::uint32_t begin = coarse_count[c];
    const std::uint32_t end = coarse_count[c + 1U];
    if (end == begin) continue;
    const auto nth = z_values.begin() + begin +
                     static_cast<std::ptrdiff_t>(std::floor(tp.ground_percentile *
                                                            static_cast<float>(end - begin - 1U)));
    std::nth_element(z_values.begin() + begin, nth, z_values.begin() + end);
    candidate[c] = *nth;
  }
  z_values.clear();
  z_values.shrink_to_fit();

  std::vector<float> ground(coarse.size(), kUnknownHeight);
  std::vector<std::uint8_t> ground_source(coarse.size(), kNone);
  std::vector<float> ground_gap(coarse.size(), 0.0F);
  {
    std::vector<double> seed_sum(coarse.size(), 0.0);
    std::vector<std::uint32_t> seed_count(coarse.size(), 0U);
    rasterize_footprints(trajectory, parameters.robot.footprint, coarse,
                         [&](std::size_t index, const TrajectorySample &sample) {
                           seed_sum[index] += sample.z;
                           ++seed_count[index];
                         });
    for (const auto &sample : trajectory) {
      std::int64_t cx = 0;
      std::int64_t cy = 0;
      if (coarse.cell(sample.x, sample.y, &cx, &cy) && seed_count[coarse.index(cx, cy)] == 0U) {
        seed_sum[coarse.index(cx, cy)] += sample.z;
        ++seed_count[coarse.index(cx, cy)];
      }
    }
    std::deque<std::size_t> queue;
    for (std::size_t c = 0; c < coarse.size(); ++c) {
      if (seed_count[c] == 0U) continue;
      ground[c] = static_cast<float>(seed_sum[c] / seed_count[c]);
      ground_source[c] = kSeed;
      queue.push_back(c);
      ++traversability->ground_seed_cells;
    }
    const float step = tp.ground_max_step;
    std::vector<std::uint8_t> rejected(coarse.size(), 0U);
    while (!queue.empty()) {
      const std::size_t c = queue.front();
      queue.pop_front();
      const auto cx = static_cast<std::int64_t>(c % coarse.width);
      const auto cy = static_cast<std::int64_t>(c / coarse.width);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          const auto nx = cx + dx;
          const auto ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= coarse.width || ny >= coarse.height) continue;
          const std::size_t n = coarse.index(nx, ny);
          if (ground_source[n] != kNone) continue;
          const float distance = coarse.resolution * ((dx != 0 && dy != 0) ? 1.41421356F : 1.0F);
          const float h = candidate[n];
          if (std::isfinite(h) && std::abs(h - ground[c]) <= step * distance / coarse.resolution) {
            ground[n] = h;
            ground_source[n] = kObserved;
            ground_gap[n] = 0.0F;
            queue.push_back(n);
          } else if (std::isfinite(h) && h < ground[c] - step) {
            rejected[n] = 1U;  // drop-off / negative obstacle: never bridge it
          } else if (ground_gap[c] + distance <= tp.ground_fill_max_distance) {
            // no data, or only elevated structure above the surface: keep the
            // surface flat for a bounded distance.
            ground[n] = ground[c];
            ground_source[n] = kInferred;
            ground_gap[n] = ground_gap[c] + distance;
            queue.push_back(n);
          }
        }
      }
    }
    // Drop-off guard: never keep a flat extrapolation right next to a cell
    // whose observed surface is lower than the grown surface can step down.
    for (std::size_t c = 0; c < coarse.size(); ++c) {
      if (!rejected[c] || ground_source[c] != kNone) continue;
      const auto cx = static_cast<std::int64_t>(c % coarse.width);
      const auto cy = static_cast<std::int64_t>(c / coarse.width);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          const auto nx = cx + dx;
          const auto ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= coarse.width || ny >= coarse.height) continue;
          const std::size_t n = coarse.index(nx, ny);
          if (ground_source[n] == kInferred) {
            ground_source[n] = kNone;
            ground[n] = kUnknownHeight;
          }
        }
      }
    }
    for (std::size_t c = 0; c < coarse.size(); ++c) {
      if (ground_source[c] == kObserved) ++traversability->ground_observed_cells;
      if (ground_source[c] == kInferred) ++traversability->ground_inferred_cells;
      if (ground_source[c] == kNone && rejected[c]) ++traversability->ground_rejected_cells;
    }
  }
  // Fallback reference for classifying points outside the grown surface: the
  // lower quartile of the 3x3 candidate neighbourhood (as in local_ground).
  std::vector<float> fallback(coarse.size(), kUnknownHeight);
  {
    std::vector<float> values;
    for (std::int64_t cy = 0; cy < coarse.height; ++cy) {
      for (std::int64_t cx = 0; cx < coarse.width; ++cx) {
        values.clear();
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const auto nx = cx + dx;
            const auto ny = cy + dy;
            if (nx < 0 || ny < 0 || nx >= coarse.width || ny >= coarse.height) continue;
            const float h = candidate[coarse.index(nx, ny)];
            if (std::isfinite(h)) values.push_back(h);
          }
        }
        if (values.empty()) continue;
        std::sort(values.begin(), values.end());
        fallback[coarse.index(cx, cy)] = values[(values.size() - 1U) / 4U];
      }
    }
  }

  // Classification reference: the grown surface, extended flat for a bounded
  // distance so structures next to it are measured against the real floor.
  std::vector<float> reference = ground;
  if (tp.ground_reference_extension > 0.0F) {
    std::vector<float> distance(coarse.size(), std::numeric_limits<float>::infinity());
    std::deque<std::size_t> queue;
    for (std::size_t c = 0; c < coarse.size(); ++c) {
      if (!std::isfinite(ground[c])) continue;
      distance[c] = 0.0F;
      queue.push_back(c);
    }
    while (!queue.empty()) {
      const std::size_t c = queue.front();
      queue.pop_front();
      const auto cx = static_cast<std::int64_t>(c % coarse.width);
      const auto cy = static_cast<std::int64_t>(c / coarse.width);
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;
          const auto nx = cx + dx;
          const auto ny = cy + dy;
          if (nx < 0 || ny < 0 || nx >= coarse.width || ny >= coarse.height) continue;
          const std::size_t n = coarse.index(nx, ny);
          const float d = distance[c] +
                          coarse.resolution * ((dx != 0 && dy != 0) ? 1.41421356F : 1.0F);
          if (d < distance[n] && d <= tp.ground_reference_extension) {
            distance[n] = d;
            reference[n] = reference[c];
            queue.push_back(n);
          }
        }
      }
    }
  }

  // ---- 3. classify points ---------------------------------------------------
  bool have_bounds = false;
  for (std::size_t i = 0; i < points.size(); ++i) {
    auto &point = points[i];
    const std::size_t c = point_coarse[i];
    const float reference_height = std::isfinite(reference[c]) ? reference[c] : fallback[c];
    if (!std::isfinite(reference_height)) {
      ++stats->z_filtered_points;
      continue;
    }
    const float h = point.z - reference_height;
    if (std::abs(h) <= tp.ground_tolerance) {
      point.kind = kGround;
      ++stats->ground_points;
    } else if (h >= parameters.obstacle_min_height && h <= parameters.obstacle_max_height) {
      point.kind = kObstacle;
      if (point.persistent) ++stats->obstacle_points;
    } else {
      ++stats->z_filtered_points;
      continue;
    }
    if (point.kind == kObstacle && !point.persistent) continue;
    if (!have_bounds) {
      stats->accepted_x_min = stats->accepted_x_max = point.x;
      stats->accepted_y_min = stats->accepted_y_max = point.y;
      have_bounds = true;
    }
    stats->accepted_x_min = std::min(stats->accepted_x_min, point.x);
    stats->accepted_x_max = std::max(stats->accepted_x_max, point.x);
    stats->accepted_y_min = std::min(stats->accepted_y_min, point.y);
    stats->accepted_y_max = std::max(stats->accepted_y_max, point.y);
  }
  stats->accepted_points = stats->ground_points + stats->obstacle_points;
  if (!have_bounds) {
    if (error) *error = "no points remain after traversability classification";
    return false;
  }
  for (const auto &sample : trajectory) {
    stats->accepted_x_min = std::min(stats->accepted_x_min, sample.x);
    stats->accepted_x_max = std::max(stats->accepted_x_max, sample.x);
    stats->accepted_y_min = std::min(stats->accepted_y_min, sample.y);
    stats->accepted_y_max = std::max(stats->accepted_y_max, sample.y);
  }

  // ---- 4. fine grid ---------------------------------------------------------
  const double res = parameters.resolution;
  if (parameters.origin_auto) {
    grid->origin_x = static_cast<float>(std::floor(stats->accepted_x_min / res) * res);
    grid->origin_y = static_cast<float>(std::floor(stats->accepted_y_min / res) * res);
  } else {
    grid->origin_x = parameters.origin_x;
    grid->origin_y = parameters.origin_y;
  }
  grid->resolution = parameters.resolution;
  const double width_cells = std::floor((stats->accepted_x_max - grid->origin_x) / res) + 1.0;
  const double height_cells = std::floor((stats->accepted_y_max - grid->origin_y) / res) + 1.0;
  if (width_cells < 1.0 || height_cells < 1.0 || width_cells * height_cells > 400000000.0) {
    std::ostringstream message;
    message << "computed grid dimensions are invalid (" << width_cells << " x " << height_cells
            << " cells covering x [" << stats->accepted_x_min << ", " << stats->accepted_x_max
            << "] y [" << stats->accepted_y_min << ", " << stats->accepted_y_max << "])";
    if (error) *error = message.str();
    return false;
  }
  grid->width = static_cast<std::uint32_t>(width_cells);
  grid->height = static_cast<std::uint32_t>(height_cells);
  Raster fine;
  fine.origin_x = grid->origin_x;
  fine.origin_y = grid->origin_y;
  fine.resolution = res;
  fine.width = grid->width;
  fine.height = grid->height;
  const std::size_t cells = fine.size();
  grid->hit_count.assign(cells, 0U);
  grid->free_count.assign(cells, 0U);
  std::vector<std::uint16_t> ground_hits(cells, 0U);
  std::vector<std::uint16_t> passes(cells, 0U);
  // keyframe range of the persistent obstacle hits per cell (support rule)
  std::vector<std::uint32_t> hit_first_keyframe(cells, std::numeric_limits<std::uint32_t>::max());
  std::vector<std::uint32_t> hit_last_keyframe(cells, 0U);
  // fine column/row -> coarse column/row (-1 outside the coarse raster)
  std::vector<std::int32_t> fine_to_coarse_x(fine.width);
  std::vector<std::int32_t> fine_to_coarse_y(fine.height);
  for (std::uint32_t x = 0; x < fine.width; ++x) {
    const double wx = fine.origin_x + (x + 0.5) * res;
    const auto cx = static_cast<std::int64_t>(std::floor((wx - coarse.origin_x) / coarse.resolution));
    fine_to_coarse_x[x] = (cx >= 0 && cx < coarse.width) ? static_cast<std::int32_t>(cx) : -1;
  }
  for (std::uint32_t y = 0; y < fine.height; ++y) {
    const double wy = fine.origin_y + (y + 0.5) * res;
    const auto cy = static_cast<std::int64_t>(std::floor((wy - coarse.origin_y) / coarse.resolution));
    fine_to_coarse_y[y] = (cy >= 0 && cy < coarse.height) ? static_cast<std::int32_t>(cy) : -1;
  }
  const auto grown_ground_at = [&](std::uint32_t x, std::uint32_t y) -> float {
    const std::int32_t cx = fine_to_coarse_x[x];
    const std::int32_t cy = fine_to_coarse_y[y];
    if (cx < 0 || cy < 0) return kUnknownHeight;
    return ground[static_cast<std::size_t>(cy) * coarse.width + static_cast<std::size_t>(cx)];
  };

  for (const auto &point : points) {
    std::int64_t cx = 0;
    std::int64_t cy = 0;
    if (!fine.cell(point.x, point.y, &cx, &cy)) continue;
    const std::size_t index = fine.index(cx, cy);
    if (point.kind == kObstacle && point.persistent) {
      ++grid->hit_count[index];
      hit_first_keyframe[index] = std::min(hit_first_keyframe[index], point.keyframe);
      hit_last_keyframe[index] = std::max(hit_last_keyframe[index], point.keyframe);
    } else if (point.kind == kGround && std::isfinite(grown_ground_at(static_cast<std::uint32_t>(cx),
                                                                      static_cast<std::uint32_t>(cy)))) {
      if (ground_hits[index] < 65535U) ++ground_hits[index];
    }
  }

  // ---- 5. carve free space along sensor rays --------------------------------
  if (tp.raycast_enabled) {
    const double max_range_sq = static_cast<double>(tp.raycast_max_range) * tp.raycast_max_range;
    for (const auto &point : points) {
      const Eigen::Vector3f origin = keyframes[point.keyframe].map_from_body.translation();
      const double dxw = point.x - origin.x();
      const double dyw = point.y - origin.y();
      const double dzw = point.z - origin.z();
      if (dxw * dxw + dyw * dyw + dzw * dzw > max_range_sq) continue;
      const double gx0 = (origin.x() - fine.origin_x) / res;
      const double gy0 = (origin.y() - fine.origin_y) / res;
      const double gx1 = (point.x - fine.origin_x) / res;
      const double gy1 = (point.y - fine.origin_y) / res;
      auto cx = static_cast<std::int64_t>(std::floor(gx0));
      auto cy = static_cast<std::int64_t>(std::floor(gy0));
      const auto ex = static_cast<std::int64_t>(std::floor(gx1));
      const auto ey = static_cast<std::int64_t>(std::floor(gy1));
      const double ddx = gx1 - gx0;
      const double ddy = gy1 - gy0;
      const int step_x = ddx > 0.0 ? 1 : -1;
      const int step_y = ddy > 0.0 ? 1 : -1;
      const double inf = std::numeric_limits<double>::infinity();
      const double t_delta_x = ddx != 0.0 ? std::abs(1.0 / ddx) : inf;
      const double t_delta_y = ddy != 0.0 ? std::abs(1.0 / ddy) : inf;
      double t_max_x = ddx != 0.0 ? (step_x > 0 ? (cx + 1.0 - gx0) : (gx0 - cx)) * t_delta_x : inf;
      double t_max_y = ddy != 0.0 ? (step_y > 0 ? (cy + 1.0 - gy0) : (gy0 - cy)) * t_delta_y : inf;
      std::int64_t remaining = std::abs(ex - cx) + std::abs(ey - cy);
      if (point.kind != kGround) remaining -= static_cast<std::int64_t>(tp.carve_end_margin_cells);
      ++traversability->rays_cast;
      double t = 0.0;
      for (; remaining > 0; --remaining) {
        const double t_next = std::min(t_max_x, t_max_y);
        if (cx >= 0 && cy >= 0 && cx < fine.width && cy < fine.height) {
          const float g = grown_ground_at(static_cast<std::uint32_t>(cx), static_cast<std::uint32_t>(cy));
          if (std::isfinite(g)) {
            const double z = origin.z() + 0.5 * (t + std::min(t_next, 1.0)) * dzw;
            const double h = z - g;
            if (h >= -tp.ground_tolerance && h <= tp.carve_max_height) {
              std::uint16_t &count = passes[fine.index(cx, cy)];
              if (count < 65535U) ++count;
            }
          }
          ++traversability->ray_cells_visited;
        }
        t = t_next;
        if (t_max_x < t_max_y) {
          t_max_x += t_delta_x;
          cx += step_x;
        } else {
          t_max_y += t_delta_y;
          cy += step_y;
        }
      }
    }
  }

  // ---- 6. occupied / free decision -------------------------------------------
  std::vector<std::uint8_t> occupied(cells, 0U);
  {
    const auto r = static_cast<std::int64_t>(tp.obstacle_support_radius_cells);
    for (std::int64_t y = 0; y < fine.height; ++y) {
      for (std::int64_t x = 0; x < fine.width; ++x) {
        const std::size_t i = fine.index(x, y);
        const std::uint32_t hits = grid->hit_count[i];
        if (hits >= parameters.occupied_threshold) {
          occupied[i] = 1U;
        } else if (hits > 0U && r > 0) {
          // sparse (porous) structure: enough hits in the neighbourhood, seen
          // from keyframes at least temporal_min_keyframe_span apart
          std::uint32_t support = 0U;
          std::uint32_t first = std::numeric_limits<std::uint32_t>::max();
          std::uint32_t last = 0U;
          for (std::int64_t ny = std::max<std::int64_t>(y - r, 0);
               ny <= std::min<std::int64_t>(y + r, static_cast<std::int64_t>(fine.height) - 1); ++ny) {
            for (std::int64_t nx = std::max<std::int64_t>(x - r, 0);
                 nx <= std::min<std::int64_t>(x + r, static_cast<std::int64_t>(fine.width) - 1); ++nx) {
              const std::size_t n = fine.index(nx, ny);
              if (grid->hit_count[n] == 0U) continue;
              support += grid->hit_count[n];
              first = std::min(first, hit_first_keyframe[n]);
              last = std::max(last, hit_last_keyframe[n]);
            }
          }
          if (support >= parameters.occupied_threshold &&
              last - first >= std::max<std::uint32_t>(parameters.temporal_min_keyframe_span, 1U)) {
            occupied[i] = 1U;
            ++traversability->support_occupied_cells;
          }
        }
      }
    }
  }
  stats->removed_small_component_cells +=
      remove_small_components(&occupied, fine.width, fine.height, parameters.min_component_cells);
  {
    const auto before = occupied;
    occupied = morph(morph(occupied, fine.width, fine.height, parameters.closing_radius_cells, true),
                     fine.width, fine.height, parameters.closing_radius_cells, false);
    for (std::size_t i = 0; i < cells; ++i) {
      if (occupied[i] && !before[i]) ++stats->closing_added_cells;
    }
  }
  stats->removed_small_component_cells +=
      remove_small_components(&occupied, fine.width, fine.height, parameters.min_component_cells);

  std::vector<std::uint8_t> free_space(cells, 0U);
  for (std::uint32_t y = 0; y < fine.height; ++y) {
    for (std::uint32_t x = 0; x < fine.width; ++x) {
      const std::size_t i = static_cast<std::size_t>(y) * fine.width + x;
      grid->free_count[i] = static_cast<std::uint32_t>(ground_hits[i]) + passes[i];
      if (ground_hits[i] > 0U) ++traversability->ground_hit_cells;
      if (passes[i] >= tp.free_min_passes) ++traversability->carved_cells;
      if (occupied[i] || !std::isfinite(grown_ground_at(x, y))) continue;
      if (ground_hits[i] >= parameters.free_threshold || passes[i] >= tp.free_min_passes) {
        if (tp.free_requires_no_hits && grid->hit_count[i] > 0U) {
          ++traversability->hit_vetoed_cells;
          continue;
        }
        free_space[i] = 1U;
      }
    }
  }

  if (tp.sweep_enabled) {
    const auto shrunk = offset_polygon(parameters.robot.footprint, tp.sweep_padding);
    std::vector<std::uint8_t> swept(cells, 0U);
    rasterize_footprints(trajectory, shrunk, fine,
                         [&](std::size_t index, const TrajectorySample &) { swept[index] = 1U; });
    for (std::size_t i = 0; i < cells; ++i) {
      if (!swept[i]) continue;
      ++traversability->sweep_cells;
      if (occupied[i]) {
        occupied[i] = 0U;
        ++traversability->sweep_cleared_occupied_cells;
      }
      if (!free_space[i]) {
        free_space[i] = 1U;
        ++stats->free_expanded_cells;
      }
    }
  }

  if (tp.hole_fill_max_area > 0.0F) {
    const auto max_cells = static_cast<std::size_t>(tp.hole_fill_max_area / (res * res));
    std::vector<std::uint8_t> visited(cells, 0U);
    std::vector<std::size_t> component;
    std::vector<std::size_t> stack;
    for (std::size_t start = 0; start < cells; ++start) {
      if (visited[start] || free_space[start] || occupied[start]) continue;
      component.clear();
      stack.assign(1U, start);
      visited[start] = 1U;
      bool enclosed = true;
      while (!stack.empty()) {
        const std::size_t index = stack.back();
        stack.pop_back();
        component.push_back(index);
        const auto x = static_cast<std::int64_t>(index % fine.width);
        const auto y = static_cast<std::int64_t>(index / fine.width);
        const std::int64_t neighbours[4][2] = {{x + 1, y}, {x - 1, y}, {x, y + 1}, {x, y - 1}};
        for (const auto &neighbour : neighbours) {
          if (neighbour[0] < 0 || neighbour[1] < 0 || neighbour[0] >= fine.width ||
              neighbour[1] >= fine.height) {
            enclosed = false;
            continue;
          }
          const std::size_t next = fine.index(neighbour[0], neighbour[1]);
          if (occupied[next]) {
            enclosed = false;
          } else if (!free_space[next] && !visited[next]) {
            visited[next] = 1U;
            stack.push_back(next);
          }
        }
      }
      if (enclosed && component.size() <= max_cells) {
        for (const auto index : component) {
          free_space[index] = 1U;
          ++traversability->hole_filled_cells;
          ++stats->free_expanded_cells;
        }
      }
    }
  }

  grid->occupancy.assign(cells, -1);
  for (std::size_t i = 0; i < cells; ++i) {
    if (occupied[i]) {
      grid->occupancy[i] = 100;
      ++stats->occupied_cells;
    } else if (free_space[i] || parameters.empty_cell == EmptyCellPolicy::Free) {
      grid->occupancy[i] = 0;
      ++stats->free_cells;
    } else {
      ++stats->unknown_cells;
    }
  }
  stats->empty_cells = stats->free_cells + stats->unknown_cells;
  traversability->valid = true;

  if (tp.debug_layers && !debug_dir.empty()) {
    try {
      std::filesystem::create_directories(debug_dir);
    } catch (const std::exception &exception) {
      if (error) *error = exception.what();
      return false;
    }
    std::vector<std::uint8_t> source_pixels(coarse.size(), 0U);
    for (std::size_t c = 0; c < coarse.size(); ++c) {
      source_pixels[c] = ground_source[c] == kSeed ? 255U
                         : ground_source[c] == kObserved ? 170U
                         : ground_source[c] == kInferred ? 90U : 0U;
    }
    std::vector<std::uint8_t> evidence(cells, 0U);
    for (std::size_t i = 0; i < cells; ++i) {
      const float ratio = std::min(1.0F, std::log1p(static_cast<float>(passes[i])) / std::log1p(200.0F));
      evidence[i] = static_cast<std::uint8_t>(ratio * 255.0F);
    }
    if (!write_pgm(std::filesystem::path(debug_dir) / "ground_source.pgm", source_pixels,
                   coarse.width, coarse.height) ||
        !write_pgm(std::filesystem::path(debug_dir) / "ray_passes.pgm", evidence, fine.width,
                   fine.height)) {
      if (error) *error = "failed writing traversability debug layers";
      return false;
    }
    {
      // raw little-endian float32, row 0 = lowest y, NaN = unknown
      std::ofstream grown(std::filesystem::path(debug_dir) / "ground_height.f32", std::ios::binary);
      grown.write(reinterpret_cast<const char *>(ground.data()),
                  static_cast<std::streamsize>(ground.size() * sizeof(float)));
      std::ofstream classified(std::filesystem::path(debug_dir) / "reference_height.f32",
                               std::ios::binary);
      classified.write(reinterpret_cast<const char *>(reference.data()),
                       static_cast<std::streamsize>(reference.size() * sizeof(float)));
      if (!grown.good() || !classified.good()) {
        if (error) *error = "failed writing traversability height layers";
        return false;
      }
    }
    std::ofstream info(std::filesystem::path(debug_dir) / "layers.yaml");
    info << "ground_source.pgm: {resolution: " << coarse.resolution << ", origin: [" << coarse.origin_x
         << ", " << coarse.origin_y << "], values: {0: none, 90: inferred, 170: observed, 255: trajectory}}\n"
         << "ray_passes.pgm: {resolution: " << res << ", origin: [" << fine.origin_x << ", "
         << fine.origin_y << "], scale: log1p(passes)/log1p(200)}\n"
         << "ground_height.f32: {resolution: " << coarse.resolution << ", origin: ["
         << coarse.origin_x << ", " << coarse.origin_y << "], width: " << coarse.width
         << ", height: " << coarse.height << ", dtype: float32, row0: min_y, nan: unknown}\n"
         << "reference_height.f32: {resolution: " << coarse.resolution << ", origin: ["
         << coarse.origin_x << ", " << coarse.origin_y << "], width: " << coarse.width
         << ", height: " << coarse.height << ", dtype: float32, row0: min_y, nan: fallback}\n";
  }
  return true;
}

}  // namespace agt_pcd2grid_exporter
