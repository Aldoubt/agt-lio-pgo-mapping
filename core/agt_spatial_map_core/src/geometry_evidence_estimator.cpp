#include "agt_spatial_map_core/geometry_evidence.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace agt_spatial_map_core {
namespace {
namespace fs = std::filesystem;
using Matrix3 = Eigen::Matrix3d;

Eigen::Vector3f canonical_direction(Eigen::Vector3d direction) {
  Eigen::Index largest = 0;
  direction.cwiseAbs().maxCoeff(&largest);
  if (direction[largest] < 0.0) direction = -direction;
  return direction.cast<float>();
}

pcl::PointXYZ pcl_point(const Eigen::Vector3f &p) {
  pcl::PointXYZ out;
  out.x = p.x(); out.y = p.y(); out.z = p.z();
  return out;
}

GeometrySpectrum spectrum(const Matrix3 &matrix, std::uint32_t support,
                          std::uint32_t min_valid_normals, double epsilon) {
  GeometrySpectrum out;
  if (support < min_valid_normals || !matrix.allFinite() ||
      !(matrix.trace() > epsilon)) return out; // invalid: leave ALL metrics NaN
  Eigen::SelfAdjointEigenSolver<Matrix3> solve(matrix);
  if (solve.info() != Eigen::Success) return out;
  Eigen::Vector3d lambda = solve.eigenvalues(); // Eigen orders min/mid/max
  if (lambda.minCoeff() < -1e-9 * std::max(1.0, matrix.trace())) {
    throw std::runtime_error("observability matrix is not positive semidefinite");
  }
  lambda = lambda.cwiseMax(0.0);
  const double trace = lambda.sum();
  if (!(trace > epsilon)) return out;
  const double q = 3.0 * lambda.x() / (trace + epsilon);
  const double condition = lambda.z() / std::max(lambda.x(), epsilon);
  if (!lambda.allFinite() || !std::isfinite(q) || !std::isfinite(condition) ||
      lambda.z() > std::numeric_limits<float>::max() ||
      condition > std::numeric_limits<float>::max()) return out;
  out.valid = true;
  out.eigenvalues = lambda.cast<float>();
  out.weak_direction = canonical_direction(solve.eigenvectors().col(0));
  out.isotropy = static_cast<float>(q);
  out.condition = static_cast<float>(condition);
  return out;
}

void estimate_normal(const pcl::KdTreeFLANN<pcl::PointXYZ> &tree,
                     const pcl::PointXYZ &query, const GeometryParameters &p,
                     GeometryVoxelEvidence *v, std::vector<int> *indices,
                     std::vector<float> *squared_distances,
                     const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  indices->clear(); squared_distances->clear();
  const auto found = tree.radiusSearch(query, p.normal_radius, *indices, *squared_distances);
  if (found < 0 || static_cast<std::uint64_t>(found) >
                       std::numeric_limits<std::uint32_t>::max()) {
    throw std::overflow_error("raw normal support count exceeds uint32 capacity");
  }
  v->normal_support_points = static_cast<std::uint32_t>(found);
  if (v->normal_support_points < p.normal_min_points) return;
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (int i : *indices) mean += cloud[static_cast<std::size_t>(i)].getVector3fMap().cast<double>();
  mean /= static_cast<double>(found);
  Matrix3 covariance = Matrix3::Zero();
  for (int i : *indices) {
    const Eigen::Vector3d d = cloud[static_cast<std::size_t>(i)].getVector3fMap().cast<double>() - mean;
    covariance.noalias() += d * d.transpose();
  }
  covariance /= static_cast<double>(found); // population covariance, units m^2
  if (!covariance.allFinite()) return;
  Eigen::SelfAdjointEigenSolver<Matrix3> solve(covariance);
  if (solve.info() != Eigen::Success) return;
  Eigen::Vector3d lambda = solve.eigenvalues();
  if (lambda.minCoeff() < -1e-9 * std::max(1.0, covariance.trace())) return;
  lambda = lambda.cwiseMax(0.0);
  // A collinear neighborhood has no unique surface normal. A planar one has
  // lambda_min=0 but lambda_mid>0, and is a valid (possibly anisotropic) plane.
  if (!(lambda.y() > p.normal_epsilon_m2) || !(lambda.z() > p.normal_epsilon_m2)) return;
  v->normal_valid = true;
  v->covariance_eigenvalues = lambda.cast<float>();
  v->normal = canonical_direction(solve.eigenvectors().col(0));
  v->linearity = static_cast<float>((lambda.z() - lambda.y()) / lambda.z());
  v->planarity = static_cast<float>((lambda.y() - lambda.x()) / lambda.z());
  v->scattering = static_cast<float>(lambda.x() / lambda.z());
}

void add_checked(std::uint64_t *sum, std::uint64_t number) {
  if (*sum > std::numeric_limits<std::uint64_t>::max() - number) {
    throw std::overflow_error("geometry observation support exceeds uint64 capacity");
  }
  *sum += number;
}

}  // namespace

GeometryEvidence GeometryEvidenceEstimator::estimate(
    const SpatialEvidenceMap &confidence,
    const std::vector<GeometryObservation> &observations, float voxel_size,
    const GeometryParameters &parameters, std::uint32_t keyframes) {
  validate_geometry_parameters(parameters);
  if (!std::isfinite(voxel_size) || voxel_size <= 0 || confidence.empty() ||
      observations.empty() || observations.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("empty/oversized geometry input or invalid V1 voxel_size");
  }
  GeometryEvidence result;
  result.voxel_size = voxel_size;
  result.parameters = parameters;
  result.source_observations = observations.size();
  result.source_keyframes = keyframes;
  result.voxels.reserve(confidence.size());
  for (const auto &[key, source] : confidence) {
    if (!(key == source.key) || !source.centroid.allFinite() ||
        source.point_count == 0 || !(voxel_for(source.centroid, voxel_size) == key)) {
      throw std::invalid_argument("V1 confidence source has inconsistent voxel key/centroid");
    }
    GeometryVoxelEvidence v;
    v.key = key;
    v.center = source.centroid;
    result.voxels.push_back(std::move(v));
  }
  std::sort(result.voxels.begin(), result.voxels.end(), [](const auto &a, const auto &b) {
    return a.key < b.key;
  });
  if (result.voxels.size() > std::numeric_limits<int>::max()) {
    throw std::overflow_error("geometry voxel index exceeds PCL KD-tree capacity");
  }
  std::unordered_map<VoxelKey, std::uint32_t, VoxelKeyHash> by_key;
  by_key.reserve(result.voxels.size());
  for (std::uint32_t i = 0; i < result.voxels.size(); ++i) {
    by_key.emplace(result.voxels[i].key, i);
  }
  std::vector<std::uint32_t> counts(result.voxels.size(), 0);
  std::vector<Eigen::Vector3d> sums(result.voxels.size(), Eigen::Vector3d::Zero());
  std::vector<std::uint32_t> point_voxel;
  point_voxel.reserve(observations.size());
  auto raw = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  raw->reserve(observations.size());
  for (const auto &o : observations) {
    if (!o.position_map.allFinite() || !o.body_origin_map.allFinite()) {
      throw std::invalid_argument("nonfinite transformed geometry observation or T_map_body origin");
    }
    const auto found = by_key.find(voxel_for(o.position_map, voxel_size));
    if (found == by_key.end()) throw std::invalid_argument("raw patch point absent from V1 voxel keys");
    const auto i = found->second;
    if (counts[i] == std::numeric_limits<std::uint32_t>::max()) {
      throw std::overflow_error("geometry voxel point count exceeds uint32 capacity");
    }
    ++counts[i];
    sums[i] += o.position_map.cast<double>();
    point_voxel.push_back(i);
    raw->push_back(pcl_point(o.position_map));
  }
  for (std::size_t i = 0; i < result.voxels.size(); ++i) {
    const auto &source = confidence.at(result.voxels[i].key);
    if (counts[i] != source.point_count) {
      throw std::invalid_argument("raw patch point count conflicts with V1 confidence voxel");
    }
    const Eigen::Vector3f centroid = (sums[i] / static_cast<double>(counts[i])).cast<float>();
    for (Eigen::Index k = 0; k < 3; ++k) {
      if (std::abs(centroid[k] - source.centroid[k]) >
          1e-4F + 1e-6F * std::abs(source.centroid[k])) {
        throw std::invalid_argument("raw patch centroid conflicts with V1 confidence voxel");
      }
    }
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> raw_tree(false); // do not sort equal-distance neighbors
  raw_tree.setInputCloud(raw);
  std::vector<int> indices;
  std::vector<float> distances;
  for (auto &v : result.voxels) {
    estimate_normal(raw_tree, pcl_point(v.center), parameters, &v, &indices, &distances, *raw);
  }
  // The origin is NOT base_link or the global origin. Each raw point retains
  // the optimized body translation of its own keyframe observation.
  std::vector<Matrix3> mean_rotation(result.voxels.size(), Matrix3::Zero());
  for (std::size_t j = 0; j < observations.size(); ++j) {
    const auto i = point_voxel[j];
    const auto &v = result.voxels[i];
    if (!v.normal_valid) continue;
    const Eigen::Vector3d delta = (observations[j].position_map.cast<double>() -
                                   observations[j].body_origin_map.cast<double>());
    const Eigen::Vector3d g = delta.cross(v.normal.cast<double>());
    mean_rotation[i].noalias() += g * g.transpose();
  }
  for (std::size_t i = 0; i < result.voxels.size(); ++i) {
    if (result.voxels[i].normal_valid) mean_rotation[i] /= static_cast<double>(counts[i]);
  }
  point_voxel.clear(); point_voxel.shrink_to_fit(); raw.reset();
  auto valid_centers = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<std::uint32_t> valid_indices;
  valid_centers->reserve(result.voxels.size());
  valid_indices.reserve(result.voxels.size());
  for (std::uint32_t i = 0; i < result.voxels.size(); ++i) {
    if (result.voxels[i].normal_valid) {
      valid_centers->push_back(pcl_point(result.voxels[i].center));
      valid_indices.push_back(i);
    }
  }
  if (valid_centers->empty()) return result; // all invalid, no fabricated Ht/Hr
  pcl::KdTreeFLANN<pcl::PointXYZ> center_tree(false);
  center_tree.setInputCloud(valid_centers);
  for (auto &v : result.voxels) {
    indices.clear(); distances.clear();
    const auto found = center_tree.radiusSearch(pcl_point(v.center),
        parameters.observability_radius, indices, distances);
    if (found < 0 || static_cast<std::uint64_t>(found) >
                         std::numeric_limits<std::uint32_t>::max()) {
      throw std::overflow_error("valid voxel normal support exceeds uint32 capacity");
    }
    v.valid_normal_voxels = static_cast<std::uint32_t>(found);
    Matrix3 ht = Matrix3::Zero(), hr = Matrix3::Zero();
    for (int search_index : indices) {
      const std::uint32_t i = valid_indices[static_cast<std::size_t>(search_index)];
      const Eigen::Vector3d n = result.voxels[i].normal.cast<double>();
      ht.noalias() += n * n.transpose(); // one vote per valid normal VOXEL
      hr += mean_rotation[i];            // one vote per voxel's MEAN of g*g^T
      add_checked(&v.supporting_observations, counts[i]);
    }
    v.translation = spectrum(ht, v.valid_normal_voxels, parameters.min_valid_normals,
                             parameters.translation_epsilon);
    v.rotation = spectrum(hr, v.valid_normal_voxels, parameters.min_valid_normals,
                          parameters.rotation_epsilon_m2);
  }
  return result;
}

GeometryEvidence GeometryEvidenceEstimator::build(
    const fs::path &parent_package, const SpatialEvidenceMap &confidence,
    float voxel_size, const GeometryParameters &parameters,
    std::uint32_t expected_keyframes, std::uint64_t expected_usable_points) {
  if (fs::is_symlink(fs::symlink_status(parent_package)) ||
      fs::is_symlink(fs::symlink_status(parent_package / "patches")) ||
      fs::is_symlink(fs::symlink_status(parent_package / "poses_timed.txt")) ||
      !fs::is_directory(parent_package / "patches") ||
      !fs::is_regular_file(parent_package / "poses_timed.txt")) {
    throw std::invalid_argument("geometry requires a regular optimized PGO package with patches/poses");
  }
  std::ifstream poses(parent_package / "poses_timed.txt");
  if (!poses) throw std::runtime_error("cannot open PGO poses_timed.txt");
  std::vector<GeometryObservation> observations;
  if (expected_usable_points > observations.max_size()) {
    throw std::overflow_error("source points exceed memory vector capacity");
  }
  observations.reserve(static_cast<std::size_t>(expected_usable_points));
  std::unordered_set<std::string> names;
  std::uint64_t index = 0;
  std::string line;
  while (std::getline(poses, line)) {
    if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
    if (index >= std::numeric_limits<std::uint32_t>::max()) {
      throw std::overflow_error("too many PGO keyframes");
    }
    std::istringstream record(line);
    std::string filename, trailing;
    double stamp, tx, ty, tz, qw, qx, qy, qz;
    if (!(record >> filename >> stamp >> tx >> ty >> tz >> qw >> qx >> qy >> qz) ||
        (record >> trailing)) {
      throw std::invalid_argument("invalid optimized pose record");
    }
    const fs::path relative(filename);
    if (relative.empty() || relative.is_absolute() || relative.has_parent_path() ||
        relative == "." || relative == ".." || !names.insert(filename).second) {
      throw std::invalid_argument("unsafe or duplicate PGO patch reference");
    }
    if (!std::isfinite(stamp) || !std::isfinite(tx) || !std::isfinite(ty) ||
        !std::isfinite(tz) || !std::isfinite(qw) || !std::isfinite(qx) ||
        !std::isfinite(qy) || !std::isfinite(qz)) {
      throw std::invalid_argument("nonfinite optimized PGO pose");
    }
    Eigen::Quaternionf rotation(static_cast<float>(qw), static_cast<float>(qx),
                                static_cast<float>(qy), static_cast<float>(qz));
    const Eigen::Vector3f origin(static_cast<float>(tx), static_cast<float>(ty),
                                 static_cast<float>(tz));
    if (!rotation.coeffs().allFinite() || !origin.allFinite() ||
        !(rotation.norm() > std::numeric_limits<float>::epsilon())) {
      throw std::invalid_argument("invalid T_map_body optimized pose");
    }
    rotation.normalize(); // same float32 T_map_body convention as Phase 1
    const fs::path patch_path = parent_package / "patches" / relative;
    if (fs::is_symlink(fs::symlink_status(patch_path)) || !fs::is_regular_file(patch_path)) {
      throw std::invalid_argument("missing or unsafe PGO patch");
    }
    pcl::PointCloud<pcl::PointXYZI> patch;
    if (pcl::io::loadPCDFile(patch_path.string(), patch) != 0) {
      throw std::runtime_error("cannot read PGO patch: " + patch_path.string());
    }
    for (const auto &point : patch) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
      const Eigen::Vector3f mapped = rotation * Eigen::Vector3f(point.x, point.y, point.z) + origin;
      if (mapped.allFinite()) observations.push_back({mapped, origin});
    }
    ++index;
  }
  if (poses.bad() || index != expected_keyframes ||
      observations.size() != expected_usable_points) {
    throw std::invalid_argument("PGO keyframe/usable-point counts differ from verified V1 derivative");
  }
  return estimate(confidence, observations, voxel_size, parameters,
                  static_cast<std::uint32_t>(index));
}

}  // namespace agt_spatial_map_core
