#include "spatial_review_source.hpp"

#include "agt_spatial_map_core/spatial_export.hpp"

#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
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
#include <unordered_set>
#include <vector>

namespace agt_spatial_map_core {
namespace {
namespace fs = std::filesystem;

constexpr std::array<const char *, 5> kFiles{{
    "confidence_metadata.yaml", "confidence_voxels.pcd", "manual_overrides.yaml",
    "stable_map.pcd", "checksums.sha256"}};

void allowed_keys(const YAML::Node &node, const std::set<std::string> &keys) {
  if (!node || !node.IsMap()) throw std::invalid_argument("expected a YAML mapping");
  std::set<std::string> seen;
  for (const auto &item : node) {
    const auto key = item.first.as<std::string>();
    if (!keys.count(key) || !seen.insert(key).second) {
      throw std::invalid_argument("unknown or duplicate review source YAML key: " + key);
    }
  }
}

void unit(float value, const char *name) {
  if (!std::isfinite(value) || value < 0.0F || value > 1.0F) {
    throw std::invalid_argument(std::string("invalid source confidence ") + name);
  }
}

bool near(float a, float b) { return std::fabs(a - b) <= 0.00001F; }

struct FieldSpec {
  const char *name;
  std::uint8_t type;
  std::uint32_t bytes;
};

constexpr std::array<FieldSpec, 19> kFields{{
    {"x", pcl::PCLPointField::FLOAT32, 4}, {"y", pcl::PCLPointField::FLOAT32, 4},
    {"z", pcl::PCLPointField::FLOAT32, 4},
    {"voxel_x", pcl::PCLPointField::FLOAT64, 8},
    {"voxel_y", pcl::PCLPointField::FLOAT64, 8},
    {"voxel_z", pcl::PCLPointField::FLOAT64, 8},
    {"point_count", pcl::PCLPointField::UINT32, 4},
    {"observed_keyframes", pcl::PCLPointField::UINT32, 4},
    {"first_keyframe", pcl::PCLPointField::UINT32, 4},
    {"last_keyframe", pcl::PCLPointField::UINT32, 4},
    {"keyframe_span", pcl::PCLPointField::UINT32, 4},
    {"observation_score", pcl::PCLPointField::FLOAT32, 4},
    {"persistence_score", pcl::PCLPointField::FLOAT32, 4},
    {"geometry_score", pcl::PCLPointField::FLOAT32, 4},
    {"auto_confidence", pcl::PCLPointField::FLOAT32, 4},
    {"final_confidence", pcl::PCLPointField::FLOAT32, 4},
    {"override_mode", pcl::PCLPointField::UINT32, 4},
    {"has_manual_value", pcl::PCLPointField::UINT32, 4},
    {"manual_value", pcl::PCLPointField::FLOAT32, 4},
}};

template <typename T>
T field(const std::uint8_t *data, std::uint32_t offset) {
  T value{};
  std::memcpy(&value, data + offset, sizeof(value));
  return value;
}

std::int64_t exact_index(double value) {
  constexpr auto limit = static_cast<double>(std::int64_t{1} << 53);
  if (!std::isfinite(value) || std::fabs(value) > limit || std::trunc(value) != value) {
    throw std::invalid_argument("source PCD voxel index is not an exact int64 value");
  }
  return static_cast<std::int64_t>(value);
}

SpatialEvidenceMap read_evidence(const fs::path &pcd, const YAML::Node &metadata,
                                 const ConfidenceParameters &p) {
  pcl::PCLPointCloud2 cloud;
  if (pcl::io::loadPCDFile(pcd.string(), cloud) != 0 || cloud.is_bigendian ||
      cloud.height != 1 || cloud.width == 0 || cloud.fields.size() != kFields.size() ||
      cloud.data.size() != static_cast<std::size_t>(cloud.width) * cloud.point_step ||
      cloud.row_step != cloud.width * static_cast<std::uint64_t>(cloud.point_step)) {
    throw std::invalid_argument("invalid source confidence_voxels.pcd layout");
  }
  const auto declared = metadata["confidence_voxels_pcd_fields"];
  if (!declared || !declared.IsSequence() || declared.size() != kFields.size()) {
    throw std::invalid_argument("invalid source confidence field declaration");
  }
  std::array<std::uint32_t, kFields.size()> offset{};
  std::vector<bool> occupied(cloud.point_step, false);
  for (std::size_t i = 0; i < kFields.size(); ++i) {
    const auto &want = kFields[i];
    const auto &actual = cloud.fields[i];
    if (actual.name != want.name || actual.datatype != want.type || actual.count != 1 ||
        !declared[i].IsScalar() || declared[i].as<std::string>() != want.name ||
        actual.offset > cloud.point_step || want.bytes > cloud.point_step - actual.offset) {
      throw std::invalid_argument("invalid source confidence PCD field: " + std::string(want.name));
    }
    offset[i] = actual.offset;
    for (std::uint32_t byte = actual.offset; byte < actual.offset + want.bytes; ++byte) {
      if (occupied[byte]) throw std::invalid_argument("overlapping source PCD fields");
      occupied[byte] = true;
    }
  }
  SpatialEvidenceMap evidence;
  evidence.reserve(cloud.width);
  for (std::size_t i = 0; i < cloud.width; ++i) {
    const auto *data = cloud.data.data() + i * cloud.point_step;
    SpatialVoxelEvidence v;
    v.centroid = Eigen::Vector3f(field<float>(data, offset[0]),
                                 field<float>(data, offset[1]),
                                 field<float>(data, offset[2]));
    v.key = {exact_index(field<double>(data, offset[3])),
             exact_index(field<double>(data, offset[4])),
             exact_index(field<double>(data, offset[5]))};
    v.point_count = field<std::uint32_t>(data, offset[6]);
    v.observed_keyframes = field<std::uint32_t>(data, offset[7]);
    v.first_keyframe = field<std::uint32_t>(data, offset[8]);
    v.last_keyframe = field<std::uint32_t>(data, offset[9]);
    v.keyframe_span = field<std::uint32_t>(data, offset[10]);
    v.observation_score = field<float>(data, offset[11]);
    v.persistence_score = field<float>(data, offset[12]);
    v.geometry_score = field<float>(data, offset[13]);
    v.auto_confidence = field<float>(data, offset[14]);
    v.final_confidence = field<float>(data, offset[15]);
    const auto mode = field<std::uint32_t>(data, offset[16]);
    const auto has_value = field<std::uint32_t>(data, offset[17]);
    v.manual_value = field<float>(data, offset[18]);
    if (mode > static_cast<std::uint32_t>(ManualOverrideMode::IGNORE) ||
        has_value > 1 || !v.centroid.allFinite() || v.point_count == 0 ||
        v.observed_keyframes == 0 || v.observed_keyframes > v.point_count ||
        v.last_keyframe < v.first_keyframe ||
        v.keyframe_span != v.last_keyframe - v.first_keyframe) {
      throw std::invalid_argument("invalid source confidence voxel evidence");
    }
    v.override_mode = static_cast<ManualOverrideMode>(mode);
    v.has_manual_value = has_value == 1;
    for (const auto value : {v.observation_score, v.persistence_score,
                             v.geometry_score, v.auto_confidence,
                             v.final_confidence, v.manual_value}) {
      unit(value, "PCD value");
    }
    if (v.geometry_score != 1.0F) {
      throw std::invalid_argument("V1 review source geometry_score must remain 1 (deferred)");
    }
    if (v.has_manual_value && v.override_mode != ManualOverrideMode::FORCE_LOW) {
      throw std::invalid_argument("source manual value exists outside FORCE_LOW");
    }
    const auto expected = manual_final_confidence(v.auto_confidence, v.override_mode,
        v.has_manual_value, v.manual_value, p.force_low_value);
    if (!near(v.final_confidence, expected)) {
      throw std::invalid_argument("source final confidence conflicts with source auto/override");
    }
    if (!evidence.emplace(v.key, v).second) {
      throw std::invalid_argument("duplicate voxel key in source confidence PCD");
    }
  }
  if (metadata["counts"]["confidence_voxels"].as<std::uint64_t>() != evidence.size()) {
    throw std::invalid_argument("source confidence voxel count mismatch");
  }
  return evidence;
}

void verify_source_overrides(const fs::path &path, const YAML::Node &metadata,
                             const ConfidenceParameters &p, const SpatialEvidenceMap &evidence) {
  const auto root = YAML::LoadFile(path.string());
  allowed_keys(root, {"schema_version", "coordinate_system", "voxel_size", "overrides"});
  if (!root["schema_version"] || root["schema_version"].as<int>() != 1 ||
      !root["coordinate_system"] || root["coordinate_system"].as<std::string>() != "voxel_index" ||
      !root["voxel_size"] || root["voxel_size"].as<float>() != p.voxel_size ||
      !root["overrides"] || !root["overrides"].IsSequence()) {
    throw std::invalid_argument("invalid source manual_overrides.yaml schema/voxel_size");
  }
  std::unordered_set<VoxelKey, VoxelKeyHash> seen;
  for (const auto &entry : root["overrides"]) {
    allowed_keys(entry, {"key", "mode", "value", "reason", "edited_at", "editor"});
    const auto coordinates = entry["key"];
    if (!coordinates || !coordinates.IsSequence() || coordinates.size() != 3 || !entry["mode"]) {
      throw std::invalid_argument("invalid source override key or mode");
    }
    const VoxelKey key{coordinates[0].as<std::int64_t>(), coordinates[1].as<std::int64_t>(),
                       coordinates[2].as<std::int64_t>()};
    if (!seen.insert(key).second) throw std::invalid_argument("duplicate source override key");
    const auto found = evidence.find(key);
    if (found == evidence.end()) throw std::invalid_argument("unknown source override key");
    const auto mode = parse_override_mode(entry["mode"].as<std::string>());
    const auto &v = found->second;
    if (mode != v.override_mode || static_cast<bool>(entry["value"]) != v.has_manual_value ||
        (v.has_manual_value && !near(entry["value"].as<float>(), v.manual_value))) {
      throw std::invalid_argument("source override YAML conflicts with PCD");
    }
    ManualOverrideAudit audit;
    for (const char *name : {"reason", "edited_at", "editor"}) {
      const auto field = entry[name];
      if (!field) continue;
      if (!field.IsScalar() || field.as<std::string>().empty()) {
        throw std::invalid_argument("empty source manual override audit field");
      }
      if (std::string(name) == "reason") audit.reason = field.as<std::string>();
      if (std::string(name) == "edited_at") audit.edited_at = field.as<std::string>();
      if (std::string(name) == "editor") audit.editor = field.as<std::string>();
    }
    validate_manual_override_audit(audit);
  }
  for (const auto &[key, v] : evidence) {
    if (v.override_mode != ManualOverrideMode::AUTO && !seen.count(key)) {
      throw std::invalid_argument("source PCD override absent from source manual YAML");
    }
  }
  if (metadata["counts"]["manual_overrides"].as<std::uint64_t>() != seen.size()) {
    throw std::invalid_argument("source override count mismatch");
  }
}

void verify_stable_source(const fs::path &path, const YAML::Node &metadata,
                          const ConfidenceParameters &p, const SpatialEvidenceMap &evidence) {
  const auto fields = metadata["stable_map_pcd_fields"];
  if (!fields || !fields.IsSequence() || fields.size() != 4 ||
      fields[0].as<std::string>() != "x" || fields[1].as<std::string>() != "y" ||
      fields[2].as<std::string>() != "z" || fields[3].as<std::string>() != "intensity") {
    throw std::invalid_argument("source stable map field declaration mismatch");
  }
  pcl::PointCloud<pcl::PointXYZI> source;
  if (pcl::io::loadPCDFile(path.string(), source) != 0 || source.empty()) {
    throw std::invalid_argument("invalid source stable_map.pcd");
  }
  std::vector<const SpatialVoxelEvidence *> ordered;
  ordered.reserve(evidence.size());
  for (const auto &[key, voxel] : evidence) ordered.push_back(&voxel);
  std::sort(ordered.begin(), ordered.end(), [](const auto *a, const auto *b) {
    return a->key < b->key;
  });
  std::size_t i = 0;
  for (const auto *voxel : ordered) {
    if (!stable_preview_selected(voxel->final_confidence, voxel->override_mode,
                                 p.stable_threshold)) continue;
    if (i >= source.size() || !source[i].getVector3fMap().allFinite() ||
        !near(source[i].x, voxel->centroid.x()) ||
        !near(source[i].y, voxel->centroid.y()) ||
        !near(source[i].z, voxel->centroid.z()) ||
        !near(source[i].intensity, voxel->final_confidence)) {
      throw std::invalid_argument("source stable map conflicts with confidence evidence");
    }
    ++i;
  }
  if (i != source.size() || i != metadata["counts"]["stable_voxels"].as<std::uint64_t>()) {
    throw std::invalid_argument("source stable map count mismatch");
  }
}

}  // namespace

void verify_review_source_integrity(const fs::path &derivative,
                                    const std::string &expected_checksums_sha256) {
  if (fs::is_symlink(fs::symlink_status(derivative)) || !fs::is_directory(derivative)) {
    throw std::invalid_argument("source confidence derivative must be a regular directory");
  }
  const auto root = fs::canonical(derivative);
  const std::set<std::string> names(kFiles.begin(), kFiles.end());
  for (const auto &item : fs::directory_iterator(root)) {
    if (item.is_symlink() || !item.is_regular_file() ||
        !names.count(item.path().filename().string())) {
      throw std::invalid_argument("unexpected or unsafe confidence source entry");
    }
  }
  for (const auto *name : kFiles) {
    const auto path = root / name;
    if (fs::is_symlink(fs::symlink_status(path)) || !fs::is_regular_file(path) ||
        fs::file_size(path) == 0) {
      throw std::invalid_argument("missing or unsafe confidence source file: " + std::string(name));
    }
  }
  const auto checksum_hash = sha256_file(root / "checksums.sha256");
  if (!expected_checksums_sha256.empty() && checksum_hash != expected_checksums_sha256) {
    throw std::runtime_error("confidence source checksums changed during review");
  }
  std::ifstream in(root / "checksums.sha256");
  std::set<std::string> seen;
  std::string line;
  const std::regex format(R"(^([0-9a-f]{64})  ([^/\\]+)$)");
  while (std::getline(in, line)) {
    std::smatch match;
    if (!std::regex_match(line, match, format) ||
        !names.count(match[2].str()) || match[2].str() == "checksums.sha256" ||
        !seen.insert(match[2].str()).second ||
        sha256_file(root / match[2].str()) != match[1].str()) {
      throw std::invalid_argument("confidence source checksum mismatch or unsafe coverage");
    }
  }
  if (in.bad() || seen.size() != kFiles.size() - 1) {
    throw std::invalid_argument("confidence source checksum coverage incomplete");
  }
}

VerifiedReviewSource load_verified_review_source(
    const fs::path &derivative, const fs::path &parent_package) {
  verify_review_source_integrity(derivative, {});
  VerifiedReviewSource result;
  result.derivative = fs::canonical(derivative);
  result.checksums_sha256 = sha256_file(result.derivative / "checksums.sha256");
  const auto metadata = YAML::LoadFile((result.derivative / "confidence_metadata.yaml").string());
  if (!metadata || !metadata.IsMap() || !metadata["schema_version"] ||
      metadata["schema_version"].as<int>() != 1 || !metadata["artifact_type"] ||
      metadata["artifact_type"].as<std::string>() != "spatial_confidence_v1" ||
      !metadata["confidence_semantics"] ||
      metadata["confidence_semantics"].as<std::string>() != "single_session_observation_evidence" ||
      !metadata["frame_id"] || metadata["frame_id"].as<std::string>() != "map") {
    throw std::invalid_argument("unsupported source confidence schema/semantics/frame");
  }
  const auto source = metadata["source"];
  const auto parameter = metadata["parameters"];
  const auto counts = metadata["counts"];
  if (!source || !source.IsMap() || !parameter || !parameter.IsMap() ||
      !counts || !counts.IsMap() || !metadata["geometry"] ||
      !metadata["geometry"]["score"] ||
      metadata["geometry"]["mode"].as<std::string>() != "deferred" ||
      metadata["geometry"]["score"].as<float>() != 1.0F) {
    throw std::invalid_argument("source confidence provenance/parameters/counts missing");
  }
  const auto parent = fs::canonical(parent_package);
  if (fs::canonical(source["map_package"].as<std::string>()) != parent ||
      source["parent_manifest_sha256"].as<std::string>() != sha256_file(parent / "manifest.yaml") ||
      source["parent_checksums_sha256"].as<std::string>() != sha256_file(parent / "checksums.sha256")) {
    throw std::invalid_argument("source confidence derivative does not match its PGO parent");
  }
  auto &p = result.parameters;
  p.voxel_size = parameter["voxel_size"].as<float>();
  p.observation_reference = parameter["observation_reference"].as<float>();
  p.keyframe_span_reference = parameter["keyframe_span_reference"].as<float>();
  p.persistence_alpha = parameter["persistence_alpha"].as<float>();
  p.stable_threshold = parameter["stable_threshold"].as<float>();
  p.force_low_value = parameter["force_low_value"].as<float>();
  validate_parameters(p);
  const auto keyframes = counts["source_keyframes"].as<std::uint64_t>();
  if (keyframes > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("source keyframe count exceeds v1 range");
  }
  result.stats.source_keyframes = static_cast<std::uint32_t>(keyframes);
  result.stats.input_points = counts["input_points"].as<std::uint64_t>();
  result.stats.usable_points = counts["usable_points"].as<std::uint64_t>();
  result.stats.nonfinite_points = counts["nonfinite_points"].as<std::uint64_t>();
  result.evidence = read_evidence(result.derivative / "confidence_voxels.pcd", metadata, p);
  result.stats.voxel_count = result.evidence.size();
  verify_source_overrides(result.derivative / "manual_overrides.yaml", metadata, p, result.evidence);
  verify_stable_source(result.derivative / "stable_map.pcd", metadata, p, result.evidence);
  return result;
}

}  // namespace agt_spatial_map_core
