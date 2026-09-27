#include "agt_spatial_map_core/spatial_export.hpp"

#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <openssl/evp.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>
#include <unistd.h>

namespace agt_spatial_map_core {
namespace {
namespace fs = std::filesystem;

void allowed_keys(const YAML::Node &node, const std::set<std::string> &allowed) {
  if (!node || !node.IsMap()) throw std::invalid_argument("expected a YAML mapping");
  std::set<std::string> seen;
  for (const auto &item : node) {
    const auto key = item.first.as<std::string>();
    if (!allowed.count(key) || !seen.insert(key).second) {
      throw std::invalid_argument("unknown or duplicate YAML key: " + key);
    }
  }
}

void put_file(const fs::path &path, const std::string &content) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out || !(out << content << '\n')) {
    throw std::runtime_error("cannot write derivative artifact: " + path.string());
  }
  out.close();
  if (!out) throw std::runtime_error("cannot close derivative artifact: " + path.string());
}

bool inside(const fs::path &candidate, const fs::path &base) {
  auto current = candidate.begin();
  for (auto b = base.begin(); b != base.end(); ++b, ++current) {
    if (current == candidate.end() || *current != *b) return false;
  }
  return true;
}

fs::path checked_output(const SpatialExportOptions &options) {
  const fs::path target = options.output_directory.lexically_normal();
  if (target.empty() || target.filename().empty() || target.filename() == "." ||
      target.filename() == "..") {
    throw std::invalid_argument("output-directory must name a separate directory");
  }
  const auto parent = fs::canonical(options.parent_package);
  if (!fs::is_directory(parent) || !fs::is_regular_file(parent / "manifest.yaml") ||
      !fs::is_regular_file(parent / "checksums.sha256")) {
    throw std::invalid_argument("verified mapping package manifest/checksums are missing");
  }
  const fs::path output_parent = target.has_parent_path() ? target.parent_path() : ".";
  if (!fs::is_directory(output_parent)) {
    throw std::invalid_argument("output parent directory must already exist");
  }
  const auto output = fs::canonical(output_parent) / target.filename();
  if (inside(output, parent) || inside(parent, output)) {
    throw std::invalid_argument("output may not be inside or replace the immutable mapping package");
  }
  if (fs::is_symlink(fs::symlink_status(output))) {
    throw std::invalid_argument("output directory may not be a symbolic link");
  }
  if (fs::exists(output) && (!fs::is_directory(output) || !fs::is_empty(output))) {
    throw std::invalid_argument("output directory is not empty; refusing to overwrite");
  }
  return output;
}

struct StagingGuard {
  fs::path staging;
  ~StagingGuard() {
    if (!staging.empty()) {
      std::error_code ignored;
      fs::remove_all(staging, ignored);
    }
  }
};

std::vector<const SpatialVoxelEvidence *> ordered_voxels(const SpatialEvidenceMap &voxels) {
  std::vector<const SpatialVoxelEvidence *> sorted;
  sorted.reserve(voxels.size());
  for (const auto &[key, v] : voxels) {
    if (!(key == v.key)) throw std::invalid_argument("evidence map key does not match voxel key");
    sorted.push_back(&v);
  }
  std::sort(sorted.begin(), sorted.end(), [](const auto *a, const auto *b) {
    return a->key < b->key;
  });
  return sorted;
}

bool selected(const SpatialVoxelEvidence &v, const ConfidenceParameters &p) {
  return v.override_mode != ManualOverrideMode::IGNORE &&
         v.override_mode != ManualOverrideMode::FORCE_LOW &&
         v.final_confidence >= p.stable_threshold;
}

void write_voxel_pcd(const fs::path &file,
                     const std::vector<const SpatialVoxelEvidence *> &sorted) {
  if (sorted.size() > std::numeric_limits<std::uint32_t>::max()) {
    throw std::overflow_error("too many confidence voxels for PCD format");
  }
  pcl::PCLPointCloud2 cloud;
  cloud.height = 1;
  cloud.width = static_cast<std::uint32_t>(sorted.size());
  cloud.is_dense = true;
  auto field = [&cloud](const char *name, std::uint8_t type, std::uint32_t bytes) {
    pcl::PCLPointField f;
    f.name = name;
    f.offset = cloud.point_step;
    f.datatype = type;
    f.count = 1;
    cloud.fields.push_back(f);
    cloud.point_step += bytes;
  };
  field("x", pcl::PCLPointField::FLOAT32, 4);
  field("y", pcl::PCLPointField::FLOAT32, 4);
  field("z", pcl::PCLPointField::FLOAT32, 4);
  field("voxel_x", pcl::PCLPointField::FLOAT64, 8);
  field("voxel_y", pcl::PCLPointField::FLOAT64, 8);
  field("voxel_z", pcl::PCLPointField::FLOAT64, 8);
  field("point_count", pcl::PCLPointField::UINT32, 4);
  field("observed_keyframes", pcl::PCLPointField::UINT32, 4);
  field("first_keyframe", pcl::PCLPointField::UINT32, 4);
  field("last_keyframe", pcl::PCLPointField::UINT32, 4);
  field("keyframe_span", pcl::PCLPointField::UINT32, 4);
  field("observation_score", pcl::PCLPointField::FLOAT32, 4);
  field("persistence_score", pcl::PCLPointField::FLOAT32, 4);
  field("geometry_score", pcl::PCLPointField::FLOAT32, 4);
  field("auto_confidence", pcl::PCLPointField::FLOAT32, 4);
  field("final_confidence", pcl::PCLPointField::FLOAT32, 4);
  field("override_mode", pcl::PCLPointField::UINT32, 4);
  field("has_manual_value", pcl::PCLPointField::UINT32, 4);
  field("manual_value", pcl::PCLPointField::FLOAT32, 4);
  if (sorted.size() > std::numeric_limits<std::uint32_t>::max() / cloud.point_step) {
    throw std::overflow_error("confidence PCD exceeds the uint32 row-size limit");
  }
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.data.resize(cloud.row_step);
  for (std::size_t i = 0; i < sorted.size(); ++i) {
    const auto &v = *sorted[i];
    constexpr std::int64_t kExactFloat64 = (std::int64_t{1} << 53);
    for (const auto value : {v.key.x, v.key.y, v.key.z}) {
      if (value < -kExactFloat64 || value > kExactFloat64) {
        throw std::out_of_range("voxel index not exactly representable in float64 PCD field");
      }
    }
    auto *data = cloud.data.data() + i * cloud.point_step;
    std::uint32_t offset = 0;
    auto write = [&data, &offset](const auto value) {
      std::memcpy(data + offset, &value, sizeof(value));
      offset += sizeof(value);
    };
    write(v.centroid.x()); write(v.centroid.y()); write(v.centroid.z());
    write(static_cast<double>(v.key.x));
    write(static_cast<double>(v.key.y));
    write(static_cast<double>(v.key.z));
    write(v.point_count); write(v.observed_keyframes);
    write(v.first_keyframe); write(v.last_keyframe); write(v.keyframe_span);
    write(v.observation_score); write(v.persistence_score); write(v.geometry_score);
    write(v.auto_confidence); write(v.final_confidence);
    write(static_cast<std::uint32_t>(v.override_mode));
    write(static_cast<std::uint32_t>(v.has_manual_value));
    write(v.manual_value);
    if (offset != cloud.point_step) throw std::logic_error("PCD point layout mismatch");
  }
  if (pcl::PCDWriter{}.writeBinary(file.string(), cloud) != 0) {
    throw std::runtime_error("cannot serialize confidence_voxels.pcd");
  }
}

std::uint64_t write_stable_pcd(const fs::path &file,
                               const std::vector<const SpatialVoxelEvidence *> &sorted,
                               const ConfidenceParameters &parameters) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (const auto *v : sorted) {
    if (!selected(*v, parameters)) continue;
    pcl::PointXYZI point;
    point.x = v->centroid.x(); point.y = v->centroid.y(); point.z = v->centroid.z();
    point.intensity = v->final_confidence;  // visualization, NOT sensor reflectance
    cloud.push_back(point);
  }
  // PCL rejects a zero-point PCD on read; never publish an unusable file
  // or invent points to make the package appear complete.
  if (cloud.empty()) {
    throw std::runtime_error("no stable voxels at this threshold; derivative not published");
  }
  if (pcl::io::savePCDFileBinary(file.string(), cloud) != 0) {
    throw std::runtime_error("cannot serialize stable_map.pcd");
  }
  return cloud.size();
}

YAML::Node read_override_file(const fs::path &file, const ConfidenceParameters &p) {
  YAML::Node root = YAML::LoadFile(file.string());
  allowed_keys(root, {"schema_version", "coordinate_system", "voxel_size", "overrides"});
  if (!root["schema_version"] || root["schema_version"].as<int>() != 1 ||
      !root["coordinate_system"] ||
      root["coordinate_system"].as<std::string>() != "voxel_index" ||
      !root["voxel_size"] || root["voxel_size"].as<float>() != p.voxel_size ||
      !root["overrides"] || !root["overrides"].IsSequence()) {
    throw std::invalid_argument("manual_overrides must use v1 voxel_index schema and matching voxel_size");
  }
  return root["overrides"];
}

std::unordered_set<VoxelKey, VoxelKeyHash> apply_overrides(
    SpatialEvidenceMap *voxels, const fs::path &file, const ConfidenceParameters &p) {
  std::unordered_set<VoxelKey, VoxelKeyHash> touched;
  if (file.empty()) return touched;
  const YAML::Node entries = read_override_file(file, p);
  for (const auto &entry : entries) {
    allowed_keys(entry, {"key", "mode", "value"});
    const auto coordinates = entry["key"];
    if (!coordinates || !coordinates.IsSequence() || coordinates.size() != 3 || !entry["mode"]) {
      throw std::invalid_argument("manual override needs key: [x, y, z] and mode");
    }
    const VoxelKey key{coordinates[0].as<std::int64_t>(), coordinates[1].as<std::int64_t>(),
                       coordinates[2].as<std::int64_t>()};
    if (!touched.insert(key).second) throw std::invalid_argument("duplicate manual voxel key");
    const auto found = voxels->find(key);
    if (found == voxels->end()) throw std::invalid_argument("manual override targets an unobserved voxel");
    auto &v = found->second;
    v.override_mode = parse_override_mode(entry["mode"].as<std::string>());
    v.has_manual_value = static_cast<bool>(entry["value"]);
    if (v.has_manual_value && v.override_mode != ManualOverrideMode::FORCE_LOW) {
      throw std::invalid_argument("explicit override value is only supported for FORCE_LOW");
    }
    if (v.has_manual_value) v.manual_value = entry["value"].as<float>();
    calculate_confidence(&v, p);
  }
  return touched;
}

void write_overrides(const fs::path &path,
                     const std::vector<const SpatialVoxelEvidence *> &sorted,
                     const std::unordered_set<VoxelKey, VoxelKeyHash> &touched,
                     const ConfidenceParameters &p) {
  YAML::Emitter yaml;
  yaml << YAML::BeginMap << YAML::Key << "schema_version" << YAML::Value << 1
       << YAML::Key << "coordinate_system" << YAML::Value << "voxel_index"
       << YAML::Key << "voxel_size" << YAML::Value << p.voxel_size
       << YAML::Key << "overrides" << YAML::Value << YAML::BeginSeq;
  for (const auto *v : sorted) {
    if (v->override_mode == ManualOverrideMode::AUTO && !touched.count(v->key)) continue;
    yaml << YAML::BeginMap << YAML::Key << "key" << YAML::Value << YAML::Flow
         << YAML::BeginSeq << v->key.x << v->key.y << v->key.z << YAML::EndSeq
         << YAML::Key << "mode" << YAML::Value << override_mode_name(v->override_mode);
    if (v->has_manual_value) yaml << YAML::Key << "value" << YAML::Value << v->manual_value;
    yaml << YAML::EndMap;
  }
  yaml << YAML::EndSeq << YAML::EndMap;
  put_file(path, yaml.c_str());
}

void write_metadata(const fs::path &path, const SpatialExportOptions &options,
                    const std::string &manifest_digest, const std::string &checksum_digest,
                    std::uint64_t voxel_count, std::uint64_t stable_count,
                    std::uint64_t manual_count) {
  const auto &p = options.parameters;
  const auto &stats = options.build_stats;
  YAML::Emitter yaml;
  yaml << YAML::BeginMap
       << YAML::Key << "schema_version" << YAML::Value << 1
       << YAML::Key << "artifact_type" << YAML::Value << "spatial_confidence_v1"
       << YAML::Key << "confidence_semantics" << YAML::Value << "single_session_observation_evidence"
       << YAML::Key << "frame_id" << YAML::Value << "map"
       << YAML::Key << "source" << YAML::Value << YAML::BeginMap
       << YAML::Key << "map_package" << YAML::Value << fs::canonical(options.parent_package).string()
       << YAML::Key << "parent_manifest_sha256" << YAML::Value << manifest_digest
       << YAML::Key << "parent_checksums_sha256" << YAML::Value << checksum_digest
       << YAML::Key << "optimized_pgo_verified_by" << YAML::Value
       << "agt_mapping_artifacts.validation.verify_artifact"
       << YAML::EndMap
       << YAML::Key << "parameters" << YAML::Value << YAML::BeginMap
       << YAML::Key << "voxel_size" << YAML::Value << p.voxel_size
       << YAML::Key << "observation_reference" << YAML::Value << p.observation_reference
       << YAML::Key << "keyframe_span_reference" << YAML::Value << p.keyframe_span_reference
       << YAML::Key << "persistence_alpha" << YAML::Value << p.persistence_alpha
       << YAML::Key << "stable_threshold" << YAML::Value << p.stable_threshold
       << YAML::Key << "force_low_value" << YAML::Value << p.force_low_value << YAML::EndMap
       << YAML::Key << "formula" << YAML::Value << YAML::BeginMap
       << YAML::Key << "observation_score" << YAML::Value << "1 - exp(-n / N0)"
       << YAML::Key << "span_score" << YAML::Value << "min(1, s / S0)"
       << YAML::Key << "persistence_score" << YAML::Value
       << "observation_score^alpha * span_score^(1-alpha)"
       << YAML::Key << "auto_confidence" << YAML::Value << "persistence_score * geometry_score"
       << YAML::EndMap
       << YAML::Key << "geometry" << YAML::Value << YAML::BeginMap
       << YAML::Key << "mode" << YAML::Value << "deferred"
       << YAML::Key << "score" << YAML::Value << 1.0 << YAML::EndMap
       << YAML::Key << "manual_modes" << YAML::Value << YAML::Flow << YAML::BeginSeq
       << "AUTO" << "FORCE_HIGH" << "FORCE_LOW" << "IGNORE" << YAML::EndSeq
       << YAML::Key << "stable_selection" << YAML::Value
       << "final_confidence >= stable_threshold AND override_mode NOT IN (FORCE_LOW, IGNORE)"
       << YAML::Key << "keyframe_index" << YAML::Value << "zero_based_nonempty_pose_record"
       << YAML::Key << "keyframe_span" << YAML::Value << "last_keyframe - first_keyframe"
       << YAML::Key << "confidence_voxels_pcd_fields" << YAML::Value << YAML::Flow << YAML::BeginSeq
       << "x" << "y" << "z" << "voxel_x" << "voxel_y" << "voxel_z" << "point_count"
       << "observed_keyframes" << "first_keyframe" << "last_keyframe" << "keyframe_span"
       << "observation_score" << "persistence_score" << "geometry_score"
       << "auto_confidence" << "final_confidence" << "override_mode"
       << "has_manual_value" << "manual_value" << YAML::EndSeq
       << YAML::Key << "voxel_index_pcd_encoding" << YAML::Value
       << "float64 exact integer for |index| <= 2^53; int64 in manual_overrides.yaml"
       << YAML::Key << "stable_map_pcd_fields" << YAML::Value << YAML::Flow
       << YAML::BeginSeq << "x" << "y" << "z" << "intensity" << YAML::EndSeq
       << YAML::Key << "stable_map_intensity" << YAML::Value
       << "final_confidence for visualization; not sensor reflectance"
       << YAML::Key << "counts" << YAML::Value << YAML::BeginMap
       << YAML::Key << "source_keyframes" << YAML::Value << stats.source_keyframes
       << YAML::Key << "input_points" << YAML::Value << stats.input_points
       << YAML::Key << "usable_points" << YAML::Value << stats.usable_points
       << YAML::Key << "nonfinite_points" << YAML::Value << stats.nonfinite_points
       << YAML::Key << "confidence_voxels" << YAML::Value << voxel_count
       << YAML::Key << "stable_voxels" << YAML::Value << stable_count
       << YAML::Key << "manual_overrides" << YAML::Value << manual_count
       << YAML::EndMap << YAML::EndMap;
  put_file(path, yaml.c_str());
}

}  // namespace

std::string sha256_file(const fs::path &file) {
  std::ifstream in(file, std::ios::binary);
  if (!in) throw std::runtime_error("cannot hash missing file: " + file.string());
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("cannot initialize SHA-256");
  }
  std::array<char, 1 << 16> data{};
  while (in.read(data.data(), data.size()) || in.gcount() > 0) {
    if (EVP_DigestUpdate(ctx.get(), data.data(), static_cast<std::size_t>(in.gcount())) != 1) {
      throw std::runtime_error("SHA-256 update failed");
    }
  }
  if (in.bad()) throw std::runtime_error("error reading file for SHA-256: " + file.string());
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int length = 0;
  if (EVP_DigestFinal_ex(ctx.get(), digest.data(), &length) != 1 || length != 32) {
    throw std::runtime_error("SHA-256 finalization failed");
  }
  std::string result;
  constexpr char digits[] = "0123456789abcdef";
  for (unsigned i = 0; i < length; ++i) {
    result.push_back(digits[digest[i] >> 4]);
    result.push_back(digits[digest[i] & 15]);
  }
  return result;
}

ConfidenceParameters load_confidence_config(const fs::path &config) {
  ConfidenceParameters p;
  if (config.empty()) return p;
  const auto node = YAML::LoadFile(config.string());
  allowed_keys(node, {"schema_version", "voxel_size", "observation_reference",
                      "keyframe_span_reference", "persistence_alpha", "stable_threshold",
                      "force_low_value", "geometry_mode"});
  if (!node["schema_version"] || node["schema_version"].as<int>() != 1 ||
      !node["geometry_mode"] || node["geometry_mode"].as<std::string>() != "deferred") {
    throw std::invalid_argument("confidence config requires schema_version: 1 and geometry_mode: deferred");
  }
  if (node["voxel_size"]) p.voxel_size = node["voxel_size"].as<float>();
  if (node["observation_reference"]) p.observation_reference = node["observation_reference"].as<float>();
  if (node["keyframe_span_reference"]) p.keyframe_span_reference = node["keyframe_span_reference"].as<float>();
  if (node["persistence_alpha"]) p.persistence_alpha = node["persistence_alpha"].as<float>();
  if (node["stable_threshold"]) p.stable_threshold = node["stable_threshold"].as<float>();
  if (node["force_low_value"]) p.force_low_value = node["force_low_value"].as<float>();
  validate_parameters(p);
  return p;
}

SpatialExportSummary export_spatial_artifacts(
    SpatialEvidenceMap *evidence, const SpatialExportOptions &options) {
  if (!evidence || evidence->empty()) throw std::invalid_argument("cannot export empty evidence");
  validate_parameters(options.parameters);
  const auto output = checked_output(options);
  const auto manifest_digest = sha256_file(options.parent_package / "manifest.yaml");
  const auto checksum_digest = sha256_file(options.parent_package / "checksums.sha256");
  const auto touched = apply_overrides(evidence, options.manual_overrides, options.parameters);
  const auto manual_count = touched.size();
  for (auto &[key, v] : *evidence) calculate_confidence(&v, options.parameters);
  const auto sorted = ordered_voxels(*evidence);

  const auto tick = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto stage = output.parent_path() /
                     ("." + output.filename().string() + ".staging-" +
                      std::to_string(getpid()) + "-" + std::to_string(tick));
  if (!fs::create_directory(stage)) throw std::runtime_error("staging directory already exists");
  StagingGuard guard{stage};
  write_voxel_pcd(stage / "confidence_voxels.pcd", sorted);
  const auto stable_count = write_stable_pcd(stage / "stable_map.pcd", sorted, options.parameters);
  write_overrides(stage / "manual_overrides.yaml", sorted, touched, options.parameters);
  write_metadata(stage / "confidence_metadata.yaml", options, manifest_digest, checksum_digest,
                 sorted.size(), stable_count, manual_count);
  constexpr std::array<const char *, 4> files = {
      "confidence_metadata.yaml", "confidence_voxels.pcd", "manual_overrides.yaml", "stable_map.pcd"};
  std::string checksums;
  for (const auto *name : files) {
    checksums += sha256_file(stage / name) + "  " + name + '\n';
  }
  put_file(stage / "checksums.sha256", checksums.substr(0, checksums.size() - 1));
  if (options.before_publish) options.before_publish();
  if (sha256_file(options.parent_package / "manifest.yaml") != manifest_digest ||
      sha256_file(options.parent_package / "checksums.sha256") != checksum_digest) {
    throw std::runtime_error("mapping parent changed during derivative build; refusing publication");
  }
  fs::rename(stage, output);  // Atomic same-filesystem directory rename; empty target is OK.
  guard.staging.clear();
  return {sorted.size(), stable_count, manual_count, output};
}

}  // namespace agt_spatial_map_core
