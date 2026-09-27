#include "agt_spatial_map_core/geometry_evidence.hpp"
#include "agt_spatial_map_core/spatial_export.hpp"
#include "spatial_review_source.hpp"

#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>

namespace agt_spatial_map_core {
namespace {
namespace fs = std::filesystem;

struct Field {
  const char *name;
  std::uint8_t type;
  std::uint32_t bytes;
};
constexpr std::array<Field, 37> kFields{{
    {"x", 7, 4}, {"y", 7, 4}, {"z", 7, 4},
    {"voxel_x", 8, 8}, {"voxel_y", 8, 8}, {"voxel_z", 8, 8},
    {"normal_valid", 6, 4}, {"normal_support_points", 6, 4},
    {"normal_x", 7, 4}, {"normal_y", 7, 4}, {"normal_z", 7, 4},
    {"cov_lambda_min", 7, 4}, {"cov_lambda_mid", 7, 4}, {"cov_lambda_max", 7, 4},
    {"linearity", 7, 4}, {"planarity", 7, 4}, {"scattering", 7, 4},
    {"valid_normal_voxels", 6, 4}, {"supporting_observations", 6, 4},
    {"translation_valid", 6, 4},
    {"translation_lambda_min", 7, 4}, {"translation_lambda_mid", 7, 4},
    {"translation_lambda_max", 7, 4},
    {"translation_weak_x", 7, 4}, {"translation_weak_y", 7, 4},
    {"translation_weak_z", 7, 4},
    {"translation_q", 7, 4}, {"translation_condition", 7, 4},
    {"rotation_valid", 6, 4},
    {"rotation_lambda_min", 7, 4}, {"rotation_lambda_mid", 7, 4},
    {"rotation_lambda_max", 7, 4},
    {"rotation_weak_x", 7, 4}, {"rotation_weak_y", 7, 4},
    {"rotation_weak_z", 7, 4},
    {"rotation_q", 7, 4}, {"rotation_condition", 7, 4},
}};
constexpr std::array<const char *, 3> kFiles{{
    "geometry_voxels.pcd", "geometry_metadata.yaml", "checksums.sha256"}};

void exact_keys(const YAML::Node &node, const std::set<std::string> &expected) {
  if (!node || !node.IsMap() || node.size() != expected.size()) {
    throw std::invalid_argument("geometry metadata YAML mapping has missing/extra keys");
  }
  std::set<std::string> seen;
  for (const auto &entry : node) {
    const auto name = entry.first.as<std::string>();
    if (!expected.count(name) || !seen.insert(name).second) {
      throw std::invalid_argument("duplicate/unknown geometry metadata key: " + name);
    }
  }
}

bool inside(const fs::path &candidate, const fs::path &base) {
  auto current = candidate.begin();
  for (auto b = base.begin(); b != base.end(); ++b, ++current) {
    if (current == candidate.end() || *current != *b) return false;
  }
  return true;
}

void regular(const fs::path &path) {
  if (fs::is_symlink(fs::symlink_status(path)) || !fs::is_regular_file(path) ||
      fs::file_size(path) == 0) {
    throw std::invalid_argument("missing, empty or unsafe geometry file: " + path.string());
  }
}

fs::path checked_output(const GeometryExportOptions &options) {
  if (fs::is_symlink(fs::symlink_status(options.parent_package)) ||
      fs::is_symlink(fs::symlink_status(options.confidence_source))) {
    throw std::invalid_argument("PGO/confidence input directory cannot be a symlink");
  }
  const auto parent = fs::canonical(options.parent_package);
  const auto source = fs::canonical(options.confidence_source);
  regular(parent / "manifest.yaml"); regular(parent / "checksums.sha256");
  regular(source / "checksums.sha256");
  const auto target = options.output_directory.lexically_normal();
  if (target.empty() || target.filename().empty() || target.filename() == "." ||
      target.filename() == "..") {
    throw std::invalid_argument("geometry output must name a separate directory");
  }
  const auto output_parent = target.has_parent_path() ? target.parent_path() : fs::path(".");
  if (!fs::is_directory(output_parent)) {
    throw std::invalid_argument("geometry output parent directory must already exist");
  }
  const auto output = fs::canonical(output_parent) / target.filename();
  if (inside(output, parent) || inside(parent, output) ||
      inside(output, source) || inside(source, output)) {
    throw std::invalid_argument("geometry output must be separate from both immutable sources");
  }
  if (fs::is_symlink(fs::symlink_status(output)) ||
      (fs::exists(output) && (!fs::is_directory(output) || !fs::is_empty(output)))) {
    throw std::invalid_argument("geometry target is a symlink, non-directory, or nonempty");
  }
  return output;
}

void write_file(const fs::path &path, const std::string &content) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out || !(out << content << '\n')) {
    throw std::runtime_error("cannot write geometry derivative: " + path.string());
  }
  out.close();
  if (!out) throw std::runtime_error("cannot close geometry derivative: " + path.string());
}

struct StageGuard {
  fs::path path;
  ~StageGuard() {
    if (!path.empty()) {
      std::error_code ignored;
      fs::remove_all(path, ignored);
    }
  }
};

bool near(double a, double b, double relative = 1e-4) {
  return std::isfinite(a) && std::isfinite(b) &&
         std::abs(a - b) <= 1e-5 + relative * std::max(std::abs(a), std::abs(b));
}

bool all_nan(const Eigen::Vector3f &x) {
  return std::isnan(x.x()) && std::isnan(x.y()) && std::isnan(x.z());
}

void validate_spectrum(const GeometrySpectrum &s, std::uint32_t support,
                       const GeometryParameters &p, double epsilon) {
  if (!s.valid) {
    if (!all_nan(s.eigenvalues) || !all_nan(s.weak_direction) ||
        !std::isnan(s.isotropy) || !std::isnan(s.condition)) {
      throw std::invalid_argument("invalid geometry spectrum has fabricated numeric metrics");
    }
    return;
  }
  const auto &e = s.eigenvalues;
  const double trace = e.cast<double>().sum();
  if (support < p.min_valid_normals || !e.allFinite() || e.minCoeff() < 0.0F ||
      e.x() > e.y() || e.y() > e.z() || !(trace > epsilon) ||
      !s.weak_direction.allFinite() || !near(s.weak_direction.norm(), 1.0, 1e-3) ||
      !std::isfinite(s.isotropy) || s.isotropy < 0.0F || s.isotropy > 1.0F ||
      !std::isfinite(s.condition) || s.condition < 0.0F ||
      !near(s.isotropy, 3.0 * e.x() / (trace + epsilon), 1e-3) ||
      !near(s.condition, e.z() / std::max(static_cast<double>(e.x()), epsilon), 1e-3)) {
    throw std::invalid_argument("invalid geometry eigenvalues, weak axis, Q or condition");
  }
}

void validate_voxel(const GeometryVoxelEvidence &v, const SpatialVoxelEvidence &source,
                    const GeometryEvidence &geometry) {
  if (!(v.key == source.key) || !v.center.allFinite() ||
      v.center.x() != source.centroid.x() || v.center.y() != source.centroid.y() ||
      v.center.z() != source.centroid.z() ||
      v.valid_normal_voxels > geometry.voxels.size() ||
      v.supporting_observations > geometry.source_observations ||
      v.supporting_observations > std::numeric_limits<std::uint32_t>::max() ||
      (v.valid_normal_voxels == 0) != (v.supporting_observations == 0)) {
    throw std::invalid_argument("geometry VoxelKey/centroid/support disagrees with source");
  }
  if (!v.normal_valid) {
    if (!all_nan(v.normal) || !all_nan(v.covariance_eigenvalues) ||
        !std::isnan(v.linearity) || !std::isnan(v.planarity) ||
        !std::isnan(v.scattering)) {
      throw std::invalid_argument("invalid normal has fabricated shape/normal metrics");
    }
  } else {
    const auto &e = v.covariance_eigenvalues;
    if (v.normal_support_points < geometry.parameters.normal_min_points ||
        !v.normal.allFinite() || !near(v.normal.norm(), 1.0, 1e-3) ||
        !e.allFinite() || e.x() < 0.0F || e.x() > e.y() || e.y() > e.z() ||
        !(e.y() > geometry.parameters.normal_epsilon_m2) ||
        !(e.z() > geometry.parameters.normal_epsilon_m2) ||
        !near(v.linearity, (e.z() - e.y()) / e.z(), 1e-3) ||
        !near(v.planarity, (e.y() - e.x()) / e.z(), 1e-3) ||
        !near(v.scattering, e.x() / e.z(), 1e-3) ||
        !near(v.linearity + v.planarity + v.scattering, 1.0, 1e-3)) {
      throw std::invalid_argument("invalid normal PCA covariance/shape evidence");
    }
  }
  validate_spectrum(v.translation, v.valid_normal_voxels, geometry.parameters,
                    geometry.parameters.translation_epsilon);
  validate_spectrum(v.rotation, v.valid_normal_voxels, geometry.parameters,
                    geometry.parameters.rotation_epsilon_m2);
}

void validate_evidence(const GeometryEvidence &geometry, const VerifiedReviewSource &source) {
  validate_geometry_parameters(geometry.parameters);
  if (geometry.voxel_size != source.parameters.voxel_size ||
      geometry.source_observations != source.stats.usable_points ||
      geometry.source_keyframes != source.stats.source_keyframes ||
      geometry.voxels.size() != source.evidence.size() ||
      geometry.voxels.empty() ||
      geometry.voxels.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("geometry and verified confidence source parameters/counts disagree");
  }
  for (std::size_t i = 0; i < geometry.voxels.size(); ++i) {
    const auto &v = geometry.voxels[i];
    if (i && !(geometry.voxels[i - 1].key < v.key)) {
      throw std::invalid_argument("geometry voxels must have unique, sorted VoxelKeys");
    }
    const auto found = source.evidence.find(v.key);
    if (found == source.evidence.end()) {
      throw std::invalid_argument("geometry has a VoxelKey absent from V1 confidence");
    }
    validate_voxel(v, found->second, geometry);
  }
}

pcl::PCLPointCloud2 make_cloud(const GeometryEvidence &geometry) {
  pcl::PCLPointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(geometry.voxels.size());
  cloud.is_dense = false; // NaN is deliberate for invalid evidence, never 0 or a fake score
  for (const auto &item : kFields) {
    pcl::PCLPointField f;
    f.name = item.name; f.offset = cloud.point_step;
    f.datatype = item.type; f.count = 1;
    cloud.fields.push_back(f);
    cloud.point_step += item.bytes;
  }
  if (cloud.width > std::numeric_limits<std::uint32_t>::max() / cloud.point_step) {
    throw std::overflow_error("geometry PCD exceeds uint32 row-size capacity");
  }
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  for (std::size_t i = 0; i < geometry.voxels.size(); ++i) {
    const auto &v = geometry.voxels[i];
    constexpr std::int64_t exact_double = std::int64_t{1} << 53;
    for (auto key : {v.key.x, v.key.y, v.key.z}) {
      if (key < -exact_double || key > exact_double) {
        throw std::out_of_range("VoxelKey cannot be represented exactly in float64 PCD");
      }
    }
    auto *bytes = cloud.data.data() + i * cloud.point_step;
    std::size_t offset = 0;
    auto put = [&bytes, &offset](auto value) {
      std::memcpy(bytes + offset, &value, sizeof(value)); offset += sizeof(value);
    };
    auto put_vec = [&put](const Eigen::Vector3f &x) {
      put(x.x()); put(x.y()); put(x.z());
    };
    auto put_spectrum = [&put, &put_vec](const GeometrySpectrum &s) {
      put(static_cast<std::uint32_t>(s.valid));
      put_vec(s.eigenvalues); put_vec(s.weak_direction);
      put(s.isotropy); put(s.condition);
    };
    put_vec(v.center);
    put(static_cast<double>(v.key.x)); put(static_cast<double>(v.key.y));
    put(static_cast<double>(v.key.z));
    put(static_cast<std::uint32_t>(v.normal_valid)); put(v.normal_support_points);
    put_vec(v.normal); put_vec(v.covariance_eigenvalues);
    put(v.linearity); put(v.planarity); put(v.scattering);
    put(v.valid_normal_voxels);
    put(static_cast<std::uint32_t>(v.supporting_observations));
    put_spectrum(v.translation); put_spectrum(v.rotation);
    if (offset != cloud.point_step) throw std::logic_error("geometry PCD field layout mismatch");
  }
  return cloud;
}

void write_metadata(const fs::path &file, const GeometryEvidence &geometry,
                    const GeometryExportOptions &options,
                    const std::string &parent_manifest_digest,
                    const std::string &parent_index_digest,
                    const std::string &confidence_index_digest) {
  std::size_t normals = 0, translation = 0, rotation = 0;
  for (const auto &v : geometry.voxels) {
    normals += v.normal_valid;
    translation += v.translation.valid;
    rotation += v.rotation.valid;
  }
  const auto &p = geometry.parameters;
  YAML::Emitter yaml;
  yaml << YAML::BeginMap
       << YAML::Key << "schema_version" << YAML::Value << 1
       << YAML::Key << "artifact_type" << YAML::Value << "spatial_geometry_evidence_v1"
       << YAML::Key << "evidence_semantics" << YAML::Value
       << "single_session_directional_observability_not_confidence_or_probability"
       << YAML::Key << "frame_id" << YAML::Value << "map"
       << YAML::Key << "voxel_size" << YAML::Value << geometry.voxel_size
       << YAML::Key << "voxel_coordinate_system" << YAML::Value << "voxel_index_floor_float32"
       << YAML::Key << "pose_convention" << YAML::Value
       << "T_map_body; translation is body origin; no base_link/TF assumption"
       << YAML::Key << "source" << YAML::Value << YAML::BeginMap
       << YAML::Key << "map_package" << YAML::Value << fs::canonical(options.parent_package).string()
       << YAML::Key << "parent_manifest_sha256" << YAML::Value << parent_manifest_digest
       << YAML::Key << "parent_checksums_sha256" << YAML::Value << parent_index_digest
       << YAML::Key << "confidence_derivative" << YAML::Value
       << fs::canonical(options.confidence_source).string()
       << YAML::Key << "confidence_checksums_sha256" << YAML::Value << confidence_index_digest
       << YAML::EndMap
       << YAML::Key << "parameters" << YAML::Value << YAML::BeginMap
       << YAML::Key << "normal_radius_m" << YAML::Value << p.normal_radius
       << YAML::Key << "normal_min_points" << YAML::Value << p.normal_min_points
       << YAML::Key << "observability_radius_m" << YAML::Value << p.observability_radius
       << YAML::Key << "min_valid_normals" << YAML::Value << p.min_valid_normals
       << YAML::Key << "epsilon" << YAML::Value << YAML::BeginMap
       << YAML::Key << "normal_covariance_m2" << YAML::Value << p.normal_epsilon_m2
       << YAML::Key << "translation" << YAML::Value << p.translation_epsilon
       << YAML::Key << "rotation_m2" << YAML::Value << p.rotation_epsilon_m2
       << YAML::EndMap << YAML::EndMap
       << YAML::Key << "method" << YAML::Value << YAML::BeginMap
       << YAML::Key << "normal" << YAML::Value
       << "PCA population covariance of raw map-frame patch points in normal_radius; "
          "min eigenvector; require lambda_mid > normal_covariance_m2"
       << YAML::Key << "weighting" << YAML::Value << "one equal vote per valid-normal voxel; "
          "rotation is its mean over raw observations"
       << YAML::Key << "translation" << YAML::Value << "Ht=sum(n*n^T); dimensionless"
       << YAML::Key << "rotation" << YAML::Value
       << "Hr=sum(mean(g*g^T)); g=(p_map-t_body_map) cross n; units m^2"
       << YAML::Key << "quality" << YAML::Value
       << "Q=3*lambda_min/(trace+unit_specific_epsilon); "
          "condition=lambda_max/max(lambda_min,unit_specific_epsilon)"
       << YAML::Key << "invalid" << YAML::Value
       << "insufficient support or zero trace => valid=0, numeric metrics=NaN"
       << YAML::EndMap
       << YAML::Key << "geometry_voxels_pcd_fields" << YAML::Value
       << YAML::Flow << YAML::BeginSeq;
  for (const auto &f : kFields) yaml << f.name;
  yaml << YAML::EndSeq
       << YAML::Key << "voxel_index_pcd_encoding" << YAML::Value
       << "float64 exact integer for |index| <= 2^53"
       << YAML::Key << "counts" << YAML::Value << YAML::BeginMap
       << YAML::Key << "source_keyframes" << YAML::Value << geometry.source_keyframes
       << YAML::Key << "usable_observations" << YAML::Value << geometry.source_observations
       << YAML::Key << "geometry_voxels" << YAML::Value << geometry.voxels.size()
       << YAML::Key << "valid_normals" << YAML::Value << normals
       << YAML::Key << "valid_translation" << YAML::Value << translation
       << YAML::Key << "valid_rotation" << YAML::Value << rotation
       << YAML::EndMap << YAML::EndMap;
  if (!yaml.good()) throw std::runtime_error("cannot emit geometry metadata YAML");
  write_file(file, yaml.c_str());
}

fs::path validated_directory(const fs::path &sidecar) {
  if (fs::is_symlink(fs::symlink_status(sidecar)) || !fs::is_directory(sidecar)) {
    throw std::invalid_argument("geometry sidecar must be a regular directory");
  }
  const auto root = fs::canonical(sidecar);
  std::set<std::string> names(kFiles.begin(), kFiles.end());
  for (const auto &entry : fs::directory_iterator(root)) {
    if (entry.is_symlink() || !entry.is_regular_file() ||
        !names.count(entry.path().filename().string())) {
      throw std::invalid_argument("geometry sidecar has unexpected or unsafe entry");
    }
  }
  for (const auto *name : kFiles) regular(root / name);
  std::ifstream index(root / "checksums.sha256");
  std::set<std::string> seen;
  const std::regex pattern(R"(^([0-9a-f]{64})  ([^/\\]+)$)");
  std::string line;
  while (std::getline(index, line)) {
    std::smatch matched;
    if (!std::regex_match(line, matched, pattern) ||
        !names.count(matched[2].str()) || matched[2].str() == "checksums.sha256" ||
        !seen.insert(matched[2].str()).second ||
        sha256_file(root / matched[2].str()) != matched[1].str()) {
      throw std::invalid_argument("geometry sidecar checksum mismatch or unsafe index");
    }
  }
  if (index.bad() || seen.size() != kFiles.size() - 1) {
    throw std::invalid_argument("geometry sidecar checksum coverage incomplete");
  }
  return root;
}

std::string digest(const YAML::Node &node) {
  if (!node || !node.IsScalar()) throw std::invalid_argument("geometry SHA-256 digest absent");
  const auto value = node.as<std::string>();
  if (!std::regex_match(value, std::regex("[a-f0-9]{64}"))) {
    throw std::invalid_argument("geometry SHA-256 digest has invalid syntax");
  }
  return value;
}

std::uint32_t flag(const std::uint8_t *bytes, std::size_t *offset) {
  std::uint32_t value;
  std::memcpy(&value, bytes + *offset, sizeof(value)); *offset += sizeof(value);
  if (value > 1) throw std::invalid_argument("geometry validity flag must be 0/1");
  return value;
}

GeometryEvidence read_cloud(const fs::path &file, const YAML::Node &meta,
                            const VerifiedReviewSource &source, GeometryEvidence geometry) {
  pcl::PCLPointCloud2 cloud;
  if (pcl::io::loadPCDFile(file.string(), cloud) != 0 || cloud.height != 1 ||
      cloud.width != source.evidence.size() || cloud.fields.size() != kFields.size() ||
      cloud.point_step != 160 || cloud.row_step != cloud.width * cloud.point_step ||
      cloud.data.size() != cloud.row_step) {
    throw std::invalid_argument("invalid geometry PCD dimensions/point layout");
  }
  std::uint32_t offset = 0;
  for (std::size_t j = 0; j < kFields.size(); ++j) {
    const auto &f = cloud.fields[j];
    if (f.name != kFields[j].name || f.datatype != kFields[j].type ||
        f.count != 1 || f.offset != offset) {
      throw std::invalid_argument("geometry PCD field name/type/offset mismatch");
    }
    offset += kFields[j].bytes;
  }
  const auto fields = meta["geometry_voxels_pcd_fields"];
  if (!fields || !fields.IsSequence() || fields.size() != kFields.size()) {
    throw std::invalid_argument("geometry metadata PCD field declaration mismatch");
  }
  for (std::size_t j = 0; j < kFields.size(); ++j) {
    if (fields[j].as<std::string>() != kFields[j].name) {
      throw std::invalid_argument("geometry metadata PCD field order mismatch");
    }
  }
  geometry.voxels.reserve(cloud.width);
  for (std::size_t i = 0; i < cloud.width; ++i) {
    const auto *bytes = cloud.data.data() + i * cloud.point_step;
    std::size_t pos = 0;
    auto read_float = [&bytes, &pos]() {
      float result; std::memcpy(&result, bytes + pos, sizeof(result));
      pos += sizeof(result); return result;
    };
    auto read_double = [&bytes, &pos]() {
      double result; std::memcpy(&result, bytes + pos, sizeof(result));
      pos += sizeof(result); return result;
    };
    auto read_uint = [&bytes, &pos]() {
      std::uint32_t result; std::memcpy(&result, bytes + pos, sizeof(result));
      pos += sizeof(result); return result;
    };
    auto read_vec = [&read_float]() {
      const float x = read_float(), y = read_float(), z = read_float();
      return Eigen::Vector3f(x, y, z);
    };
    GeometryVoxelEvidence v;
    v.center = read_vec();
    auto read_key = [&read_double]() {
      const auto number = read_double();
      constexpr std::int64_t exact = std::int64_t{1} << 53;
      if (!std::isfinite(number) || number < -static_cast<double>(exact) ||
          number > static_cast<double>(exact) || std::trunc(number) != number) {
        throw std::invalid_argument("non-integral or inexact PCD VoxelKey");
      }
      return static_cast<std::int64_t>(number);
    };
    v.key = {read_key(), read_key(), read_key()};
    v.normal_valid = flag(bytes, &pos) == 1;
    v.normal_support_points = read_uint();
    v.normal = read_vec(); v.covariance_eigenvalues = read_vec();
    v.linearity = read_float(); v.planarity = read_float(); v.scattering = read_float();
    v.valid_normal_voxels = read_uint();
    v.supporting_observations = read_uint();
    auto read_spectrum = [&bytes, &pos, &read_vec, &read_float]() {
      GeometrySpectrum s;
      s.valid = flag(bytes, &pos) == 1;
      s.eigenvalues = read_vec(); s.weak_direction = read_vec();
      s.isotropy = read_float(); s.condition = read_float();
      return s;
    };
    v.translation = read_spectrum(); v.rotation = read_spectrum();
    if (pos != cloud.point_step) throw std::logic_error("geometry PCD reader offset mismatch");
    geometry.voxels.push_back(std::move(v));
  }
  return geometry;
}

}  // namespace

void export_geometry_evidence(const GeometryEvidence &geometry,
                              const GeometryExportOptions &options) {
  const auto output = checked_output(options);
  const auto source = load_verified_review_source(options.confidence_source, options.parent_package);
  validate_evidence(geometry, source);
  const auto manifest_digest = sha256_file(options.parent_package / "manifest.yaml");
  const auto parent_index_digest = sha256_file(options.parent_package / "checksums.sha256");
  const auto source_index_digest = source.checksums_sha256;
  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto stage = output.parent_path() / ("." + output.filename().string() +
      ".staging-" + std::to_string(getpid()) + "-" + std::to_string(tick));
  if (!fs::create_directory(stage)) throw std::runtime_error("geometry staging path exists");
  StageGuard guard{stage};
  if (pcl::PCDWriter{}.writeBinary((stage / "geometry_voxels.pcd").string(),
                                   make_cloud(geometry)) != 0) {
    throw std::runtime_error("cannot write geometry PCD");
  }
  write_metadata(stage / "geometry_metadata.yaml", geometry, options,
                 manifest_digest, parent_index_digest, source_index_digest);
  std::string index;
  for (auto *name : {"geometry_metadata.yaml", "geometry_voxels.pcd"}) {
    index += sha256_file(stage / name) + "  " + name + '\n';
  }
  write_file(stage / "checksums.sha256", index.substr(0, index.size() - 1));
  if (options.before_publish) options.before_publish();
  // The two source trees must remain byte-for-byte verified up to publication,
  // not merely match metadata recorded before a long estimator run.
  verify_review_source_integrity(options.confidence_source, source_index_digest);
  if (sha256_file(options.parent_package / "manifest.yaml") != manifest_digest ||
      sha256_file(options.parent_package / "checksums.sha256") != parent_index_digest) {
    throw std::runtime_error("PGO parent changed during geometry build");
  }
  if (fs::is_symlink(fs::symlink_status(output)) ||
      (fs::exists(output) && (!fs::is_directory(output) || !fs::is_empty(output)))) {
    throw std::runtime_error("geometry output changed into a symlink/nonempty target");
  }
  fs::rename(stage, output); // same-filesystem atomic publish; empty target is allowed
  guard.path.clear();
}

GeometryEvidence load_geometry_evidence(const fs::path &sidecar,
                                        const fs::path &parent_package,
                                        const fs::path &confidence_source) {
  const auto root = validated_directory(sidecar);
  if (fs::is_symlink(fs::symlink_status(parent_package)) ||
      fs::is_symlink(fs::symlink_status(confidence_source))) {
    throw std::invalid_argument("geometry provenance source directory cannot be a symlink");
  }
  const auto source = load_verified_review_source(confidence_source, parent_package);
  const auto meta = YAML::LoadFile((root / "geometry_metadata.yaml").string());
  exact_keys(meta, {"schema_version", "artifact_type", "evidence_semantics", "frame_id",
                    "voxel_size", "voxel_coordinate_system", "pose_convention", "source",
                    "parameters", "method", "geometry_voxels_pcd_fields",
                    "voxel_index_pcd_encoding", "counts"});
  if (meta["schema_version"].as<int>() != 1 ||
      meta["artifact_type"].as<std::string>() != "spatial_geometry_evidence_v1" ||
      meta["evidence_semantics"].as<std::string>() !=
          "single_session_directional_observability_not_confidence_or_probability" ||
      meta["frame_id"].as<std::string>() != "map" ||
      meta["voxel_coordinate_system"].as<std::string>() != "voxel_index_floor_float32" ||
      meta["pose_convention"].as<std::string>() !=
          "T_map_body; translation is body origin; no base_link/TF assumption" ||
      meta["voxel_index_pcd_encoding"].as<std::string>() !=
          "float64 exact integer for |index| <= 2^53") {
    throw std::invalid_argument("unsupported geometry schema, frame or semantics");
  }
  exact_keys(meta["source"], {"map_package", "parent_manifest_sha256",
                              "parent_checksums_sha256", "confidence_derivative",
                              "confidence_checksums_sha256"});
  const auto provenance = meta["source"];
  if (fs::canonical(provenance["map_package"].as<std::string>()) != fs::canonical(parent_package) ||
      fs::canonical(provenance["confidence_derivative"].as<std::string>()) !=
          fs::canonical(confidence_source) ||
      digest(provenance["parent_manifest_sha256"]) !=
          sha256_file(parent_package / "manifest.yaml") ||
      digest(provenance["parent_checksums_sha256"]) !=
          sha256_file(parent_package / "checksums.sha256") ||
      digest(provenance["confidence_checksums_sha256"]) != source.checksums_sha256) {
    throw std::invalid_argument("geometry source does not match verified PGO/V1 provenance");
  }
  const auto method = meta["method"];
  exact_keys(method, {"normal", "weighting", "translation", "rotation", "quality", "invalid"});
  if (method["normal"].as<std::string>() !=
          "PCA population covariance of raw map-frame patch points in normal_radius; "
          "min eigenvector; require lambda_mid > normal_covariance_m2" ||
      method["weighting"].as<std::string>() !=
          "one equal vote per valid-normal voxel; rotation is its mean over raw observations" ||
      method["translation"].as<std::string>() != "Ht=sum(n*n^T); dimensionless" ||
      method["rotation"].as<std::string>() !=
          "Hr=sum(mean(g*g^T)); g=(p_map-t_body_map) cross n; units m^2" ||
      method["quality"].as<std::string>() !=
          "Q=3*lambda_min/(trace+unit_specific_epsilon); "
          "condition=lambda_max/max(lambda_min,unit_specific_epsilon)" ||
      method["invalid"].as<std::string>() !=
          "insufficient support or zero trace => valid=0, numeric metrics=NaN") {
    throw std::invalid_argument("geometry estimator method not supported");
  }
  exact_keys(meta["parameters"], {"normal_radius_m", "normal_min_points",
                                  "observability_radius_m", "min_valid_normals", "epsilon"});
  const auto parameters = meta["parameters"];
  exact_keys(parameters["epsilon"], {"normal_covariance_m2", "translation", "rotation_m2"});
  GeometryEvidence geometry;
  geometry.voxel_size = meta["voxel_size"].as<float>();
  auto &p = geometry.parameters;
  p.normal_radius = parameters["normal_radius_m"].as<float>();
  p.normal_min_points = parameters["normal_min_points"].as<std::uint32_t>();
  p.observability_radius = parameters["observability_radius_m"].as<float>();
  p.min_valid_normals = parameters["min_valid_normals"].as<std::uint32_t>();
  p.normal_epsilon_m2 = parameters["epsilon"]["normal_covariance_m2"].as<double>();
  p.translation_epsilon = parameters["epsilon"]["translation"].as<double>();
  p.rotation_epsilon_m2 = parameters["epsilon"]["rotation_m2"].as<double>();
  validate_geometry_parameters(p);
  exact_keys(meta["counts"], {"source_keyframes", "usable_observations", "geometry_voxels",
                              "valid_normals", "valid_translation", "valid_rotation"});
  const auto counts = meta["counts"];
  geometry.source_keyframes = counts["source_keyframes"].as<std::uint32_t>();
  geometry.source_observations = counts["usable_observations"].as<std::uint64_t>();
  if (counts["geometry_voxels"].as<std::uint64_t>() != source.evidence.size()) {
    throw std::invalid_argument("geometry metadata voxel count differs from V1 source");
  }
  geometry = read_cloud(root / "geometry_voxels.pcd", meta, source, std::move(geometry));
  validate_evidence(geometry, source);
  std::uint64_t normals = 0, translation = 0, rotation = 0;
  for (const auto &v : geometry.voxels) {
    normals += v.normal_valid;
    translation += v.translation.valid;
    rotation += v.rotation.valid;
  }
  if (counts["valid_normals"].as<std::uint64_t>() != normals ||
      counts["valid_translation"].as<std::uint64_t>() != translation ||
      counts["valid_rotation"].as<std::uint64_t>() != rotation) {
    throw std::invalid_argument("geometry metadata validity counts differ from PCD");
  }
  // Fail closed if a source or checksum index was changed during loading.
  verify_review_source_integrity(confidence_source, source.checksums_sha256);
  if (sha256_file(parent_package / "manifest.yaml") !=
          digest(provenance["parent_manifest_sha256"]) ||
      sha256_file(parent_package / "checksums.sha256") !=
          digest(provenance["parent_checksums_sha256"])) {
    throw std::runtime_error("PGO source changed while geometry sidecar was loading");
  }
  return geometry;
}

}  // namespace agt_spatial_map_core
