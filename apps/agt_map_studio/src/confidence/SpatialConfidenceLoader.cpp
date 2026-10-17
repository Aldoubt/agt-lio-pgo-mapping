#include "confidence/SpatialConfidenceLoader.hpp"

#include <agt_spatial_map_core/spatial_export.hpp>

#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <regex>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace agt_map_studio {
namespace {
namespace fs = std::filesystem;
using agt_spatial_map_core::ManualOverrideMode;
using agt_spatial_map_core::VoxelKey;
using agt_spatial_map_core::VoxelKeyHash;

struct FieldSpec {
  const char *name;
  std::uint8_t type;
  std::uint32_t bytes;
};

constexpr std::array<FieldSpec, 19> kVoxelFields{{
    {"x", pcl::PCLPointField::FLOAT32, 4},
    {"y", pcl::PCLPointField::FLOAT32, 4},
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

const std::set<std::string> kFiles{
    "confidence_metadata.yaml", "confidence_voxels.pcd",
    "manual_overrides.yaml", "stable_map.pcd", "checksums.sha256"};

void checked_file(const fs::path &root, const std::string &name) {
  const auto file = root / name;
  if (fs::is_symlink(fs::symlink_status(file)) || !fs::is_regular_file(file) ||
      fs::file_size(file) == 0) {
    throw std::invalid_argument("Missing, empty or symlink confidence artifact file: " + name);
  }
}

void verify_checksums(const fs::path &root) {
  for (const auto &name : kFiles) checked_file(root, name);
  for (const auto &entry : fs::directory_iterator(root)) {
    if (!entry.is_regular_file() || entry.is_symlink() ||
        !kFiles.count(entry.path().filename().string())) {
      throw std::invalid_argument("Unexpected or unsafe confidence artifact entry: " +
                                  entry.path().filename().string());
    }
  }
  std::ifstream stream(root / "checksums.sha256");
  if (!stream) throw std::invalid_argument("Missing confidence artifact checksums.sha256");
  std::regex format(R"(^([0-9a-fA-F]{64}) [ *]([^/\\]+)$)");
  std::set<std::string> seen;
  std::string line;
  while (std::getline(stream, line)) {
    std::smatch match;
    if (!std::regex_match(line, match, format)) {
      throw std::invalid_argument("Malformed confidence artifact checksums.sha256");
    }
    const std::string name = match[2].str();
    if (!kFiles.count(name) || name == "checksums.sha256" || !seen.insert(name).second) {
      throw std::invalid_argument("Unsafe, duplicate or unknown checksum path: " + name);
    }
    if (agt_spatial_map_core::sha256_file(root / name) != match[1].str()) {
      throw std::invalid_argument("confidence artifact checksum validation failed: " + name);
    }
  }
  if (stream.bad() || seen.size() != kFiles.size() - 1U) {
    throw std::invalid_argument("confidence artifact checksum coverage incomplete");
  }
}

int schema_version(const YAML::Node &node, const char *what) {
  if (!node || !node.IsMap() || !node["schema_version"]) {
    throw std::invalid_argument(std::string("Missing ") + what + " schema_version");
  }
  const int version = node["schema_version"].as<int>();
  if (version != 1) {
    throw std::invalid_argument(std::string("Unsupported ") + what + " schema_version: " +
                                std::to_string(version));
  }
  return version;
}

void allowed_keys(const YAML::Node &node, const std::set<std::string> &keys) {
  if (!node || !node.IsMap()) throw std::invalid_argument("Expected YAML mapping");
  std::set<std::string> seen;
  for (const auto &item : node) {
    const auto key = item.first.as<std::string>();
    if (!keys.count(key) || !seen.insert(key).second) {
      throw std::invalid_argument("Unknown or duplicate manual override YAML key: " + key);
    }
  }
}

void unit_value(float value, const std::string &name) {
  if (!std::isfinite(value) || value < 0.0F || value > 1.0F) {
    throw std::invalid_argument("Invalid " + name + " (expected finite [0,1])");
  }
}

bool close_to(float a, float b) {
  return std::fabs(a - b) <= 0.00001F;
}

struct ValidatedFields {
  std::unordered_map<std::string, std::uint32_t> offsets;
  std::vector<std::string> names;
};

ValidatedFields validate_fields(const pcl::PCLPointCloud2 &cloud, const YAML::Node &declared) {
  if (!declared || !declared.IsSequence() || declared.size() != kVoxelFields.size()) {
    throw std::invalid_argument("confidence metadata declares incompatible PCD fields");
  }
  ValidatedFields fields;
  std::vector<bool> occupied(cloud.point_step, false);
  for (const auto &spec : kVoxelFields) {
    const pcl::PCLPointField *actual = nullptr;
    for (const auto &field : cloud.fields) {
      if (field.name == spec.name) {
        if (actual) throw std::invalid_argument("Duplicate PCD field: " + std::string(spec.name));
        actual = &field;
      }
    }
    if (!actual) throw std::invalid_argument("Missing PCD field: " + std::string(spec.name));
    if (actual->datatype != spec.type || actual->count != 1 ||
        actual->offset > cloud.point_step ||
        spec.bytes > cloud.point_step - actual->offset) {
      throw std::invalid_argument("Invalid PCD field type/stride: " + std::string(spec.name));
    }
    for (std::uint32_t byte = actual->offset; byte < actual->offset + spec.bytes; ++byte) {
      if (occupied[byte]) throw std::invalid_argument("Overlapping PCD field: " + std::string(spec.name));
      occupied[byte] = true;
    }
    fields.offsets.emplace(spec.name, actual->offset);
    fields.names.emplace_back(spec.name);
  }
  std::vector<std::string> actual_names;
  for (const auto &f : cloud.fields) {
    if (f.name != "_") actual_names.push_back(f.name);
  }
  if (actual_names != fields.names) {
    throw std::invalid_argument("confidence metadata declares incompatible PCD fields");
  }
  for (std::size_t i = 0; i < fields.names.size(); ++i) {
    if (!declared[i].IsScalar() || declared[i].as<std::string>() != fields.names[i]) {
      throw std::invalid_argument("confidence metadata declares incompatible PCD fields");
    }
  }
  return fields;
}

template <typename T>
T read_field(const std::uint8_t *data, const ValidatedFields &fields, const char *name) {
  T value{};
  std::memcpy(&value, data + fields.offsets.at(name), sizeof(value));
  return value;
}

std::int64_t int64_from_double(double value, const char *name) {
  constexpr auto limit = static_cast<double>(std::int64_t{1} << 53);
  if (!std::isfinite(value) || std::abs(value) > limit || std::trunc(value) != value) {
    throw std::invalid_argument(std::string("Invalid exact voxel index in field: ") + name);
  }
  return static_cast<std::int64_t>(value);
}

void validate_override_result(const ConfidenceVoxel &v, float force_low) {
  const auto mode = v.override_mode;
  if (v.has_manual_value && mode != ManualOverrideMode::FORCE_LOW) {
    throw std::invalid_argument("Only FORCE_LOW can carry a manual_value");
  }
  float expected = v.auto_confidence;  // never recompute automatic evidence
  switch (mode) {
    case ManualOverrideMode::AUTO: break;
    case ManualOverrideMode::FORCE_HIGH: expected = 1.0F; break;
    case ManualOverrideMode::FORCE_LOW:
      expected = v.has_manual_value ? v.manual_value : force_low;
      break;
    case ManualOverrideMode::IGNORE: expected = 0.0F; break;
    default: throw std::invalid_argument("Invalid PCD override_mode");
  }
  if (!close_to(v.final_confidence, expected)) {
    throw std::invalid_argument("confidence PCD final_confidence conflicts with manual override");
  }
}

std::vector<ConfidenceVoxel> read_voxels(const fs::path &file,
                                          const YAML::Node &declared,
                                          const ConfidenceArtifactInfo &info) {
  pcl::PCLPointCloud2 cloud;
  if (pcl::io::loadPCDFile(file.string(), cloud) != 0) {
    throw std::invalid_argument("Cannot load confidence_voxels.pcd");
  }
  if (cloud.is_bigendian) throw std::invalid_argument("Big-endian confidence PCD not supported");
  if (cloud.point_step == 0 || cloud.width == 0 || cloud.height != 1 ||
      cloud.width > static_cast<unsigned>(std::numeric_limits<int>::max()) ||
      cloud.row_step < cloud.width * static_cast<std::size_t>(cloud.point_step) ||
      cloud.data.size() < cloud.row_step) {
    throw std::invalid_argument("Invalid confidence PCD dimensions/stride/data");
  }
  const auto fields = validate_fields(cloud, declared);
  std::vector<ConfidenceVoxel> voxels;
  voxels.reserve(cloud.width);
  for (std::size_t i = 0; i < cloud.width; ++i) {
    const auto *data = cloud.data.data() + i * cloud.point_step;
    ConfidenceVoxel v;
    v.center.x() = read_field<float>(data, fields, "x");
    v.center.y() = read_field<float>(data, fields, "y");
    v.center.z() = read_field<float>(data, fields, "z");
    v.key.x = int64_from_double(read_field<double>(data, fields, "voxel_x"), "voxel_x");
    v.key.y = int64_from_double(read_field<double>(data, fields, "voxel_y"), "voxel_y");
    v.key.z = int64_from_double(read_field<double>(data, fields, "voxel_z"), "voxel_z");
    v.point_count = read_field<std::uint32_t>(data, fields, "point_count");
    v.observed_keyframes = read_field<std::uint32_t>(data, fields, "observed_keyframes");
    v.first_keyframe = read_field<std::uint32_t>(data, fields, "first_keyframe");
    v.last_keyframe = read_field<std::uint32_t>(data, fields, "last_keyframe");
    v.keyframe_span = read_field<std::uint32_t>(data, fields, "keyframe_span");
    v.observation_score = read_field<float>(data, fields, "observation_score");
    v.persistence_score = read_field<float>(data, fields, "persistence_score");
    v.geometry_score = read_field<float>(data, fields, "geometry_score");
    v.auto_confidence = read_field<float>(data, fields, "auto_confidence");
    v.final_confidence = read_field<float>(data, fields, "final_confidence");
    const auto mode = read_field<std::uint32_t>(data, fields, "override_mode");
    if (mode > static_cast<std::uint32_t>(ManualOverrideMode::IGNORE)) {
      throw std::invalid_argument("Invalid PCD override_mode: " + std::to_string(mode));
    }
    v.override_mode = static_cast<ManualOverrideMode>(mode);
    const auto has_value = read_field<std::uint32_t>(data, fields, "has_manual_value");
    if (has_value > 1) throw std::invalid_argument("Invalid PCD has_manual_value");
    v.has_manual_value = has_value == 1;
    v.manual_value = read_field<float>(data, fields, "manual_value");
    if (!v.center.allFinite() || v.point_count == 0 || v.observed_keyframes == 0 ||
        v.observed_keyframes > v.point_count || v.last_keyframe < v.first_keyframe ||
        v.keyframe_span != v.last_keyframe - v.first_keyframe) {
      throw std::invalid_argument("Invalid confidence voxel coordinates/evidence");
    }
    for (const auto &score : {v.observation_score, v.persistence_score,
                              v.geometry_score, v.auto_confidence,
                              v.final_confidence, v.manual_value}) {
      unit_value(score, "confidence PCD score");
    }
    if (v.geometry_score != 1.0F) {
      throw std::invalid_argument("V1 geometry_score must remain 1 (deferred)");
    }
    validate_override_result(v, info.force_low_value);
    voxels.emplace_back(std::move(v));
  }
  return voxels;
}

std::size_t read_overrides(const fs::path &file, const ConfidenceArtifactInfo &info,
                           std::vector<ConfidenceVoxel> *voxels) {
  const auto root = YAML::LoadFile(file.string());
  schema_version(root, "manual_overrides");
  allowed_keys(root, {"schema_version", "coordinate_system", "voxel_size", "overrides"});
  if (!root["coordinate_system"] || root["coordinate_system"].as<std::string>() != "voxel_index" ||
      !root["voxel_size"] || root["voxel_size"].as<float>() != info.voxel_size) {
    throw std::invalid_argument("manual_overrides voxel_size does not match confidence artifact");
  }
  const auto entries = root["overrides"];
  if (!entries || !entries.IsSequence()) throw std::invalid_argument("Invalid manual_overrides entries");
  std::unordered_map<VoxelKey, ConfidenceVoxel *, VoxelKeyHash> by_key;
  by_key.reserve(voxels->size());
  for (auto &v : *voxels) {
    if (!by_key.emplace(v.key, &v).second) {
      throw std::invalid_argument("Duplicate confidence voxel key");
    }
  }
  std::unordered_set<VoxelKey, VoxelKeyHash> seen;
  for (const auto &entry : entries) {
    allowed_keys(entry, {"key", "mode", "value", "reason", "edited_at", "editor"});
    // Optional audit keys were explicitly added to the core v1 parser first.
    const auto key_node = entry["key"];
    if (!key_node || !key_node.IsSequence() || key_node.size() != 3 || !entry["mode"]) {
      throw std::invalid_argument("Invalid manual override key/mode");
    }
    const VoxelKey key{key_node[0].as<std::int64_t>(), key_node[1].as<std::int64_t>(),
                       key_node[2].as<std::int64_t>()};
    if (!seen.insert(key).second) throw std::invalid_argument("Duplicate manual override voxel key");
    const auto found = by_key.find(key);
    if (found == by_key.end()) throw std::invalid_argument("Manual override targets an unknown voxel");
    ManualOverrideMode mode;
    try {
      mode = agt_spatial_map_core::parse_override_mode(entry["mode"].as<std::string>());
    } catch (const std::exception &) {
      throw std::invalid_argument("Malformed override mode: " + entry["mode"].as<std::string>());
    }
    auto &v = *found->second;
    if (v.override_mode != mode || v.has_manual_value != static_cast<bool>(entry["value"]) ||
        (v.has_manual_value && !close_to(v.manual_value, entry["value"].as<float>()))) {
      throw std::invalid_argument("manual_overrides conflicts with confidence PCD values");
    }
    const auto optional_audit = [&entry](const char *field) -> std::string {
      const auto node = entry[field];
      if (!node) return {};
      if (!node.IsScalar() || node.as<std::string>().empty()) {
        throw std::invalid_argument(std::string("invalid or empty manual override ") + field);
      }
      return node.as<std::string>();
    };
    agt_spatial_map_core::ManualOverrideAudit audit{
        optional_audit("reason"), optional_audit("edited_at"), optional_audit("editor")};
    agt_spatial_map_core::validate_manual_override_audit(audit);
    v.has_override_entry = true;
    v.audit = std::move(audit);
  }
  for (const auto &v : *voxels) {
    if (v.override_mode != ManualOverrideMode::AUTO && !seen.count(v.key)) {
      throw std::invalid_argument("confidence PCD has an override absent from manual_overrides.yaml");
    }
  }
  return seen.size();
}

void verify_stable_map(const fs::path &path, const YAML::Node &metadata,
                       const std::vector<ConfidenceVoxel> &voxels, float threshold) {
  pcl::PCLPointCloud2 cloud;
  if (pcl::io::loadPCDFile(path.string(), cloud) != 0) {
    throw std::invalid_argument("Cannot load stable_map.pcd");
  }
  constexpr std::array<const char *, 4> kFields{{"x", "y", "z", "intensity"}};
  const auto declared = metadata["stable_map_pcd_fields"];
  if (!declared || !declared.IsSequence() || declared.size() != kFields.size()) {
    throw std::invalid_argument("confidence metadata declares incompatible stable map fields");
  }
  if (cloud.height != 1 || cloud.width == 0 || cloud.is_bigendian ||
      cloud.data.size() != static_cast<std::size_t>(cloud.width) * cloud.point_step ||
      cloud.row_step != cloud.width * static_cast<std::uint64_t>(cloud.point_step)) {
    throw std::invalid_argument("invalid stable_map.pcd layout");
  }
  std::array<std::uint32_t, 4> offsets{};
  std::size_t actual_count = 0;
  for (const auto &field : cloud.fields) {
    if (field.name == "_") continue;  // optional PCL padding, never evidence
    if (actual_count >= kFields.size() || field.name != kFields[actual_count] ||
        field.datatype != pcl::PCLPointField::FLOAT32 || field.count != 1 ||
        field.offset > cloud.point_step || 4 > cloud.point_step - field.offset ||
        !declared[actual_count].IsScalar() ||
        declared[actual_count].as<std::string>() != kFields[actual_count]) {
      throw std::invalid_argument("stable_map.pcd field schema mismatch");
    }
    offsets[actual_count++] = field.offset;
  }
  if (actual_count != kFields.size()) {
    throw std::invalid_argument("stable_map.pcd missing required field");
  }
  const auto count = metadata["counts"]["stable_voxels"].as<std::uint64_t>();
  if (cloud.width != count) {
    throw std::invalid_argument("stable_map.pcd point count conflicts with confidence metadata");
  }
  std::vector<const ConfidenceVoxel *> sorted;
  sorted.reserve(voxels.size());
  for (const auto &voxel : voxels) sorted.push_back(&voxel);
  std::sort(sorted.begin(), sorted.end(), [](const auto *a, const auto *b) {
    return a->key < b->key;
  });
  std::size_t index = 0;
  for (const auto *voxel : sorted) {
    if (!agt_spatial_map_core::stable_preview_selected(
            voxel->final_confidence, voxel->override_mode, threshold)) continue;
    if (index >= cloud.width) {
      throw std::invalid_argument("stable_map.pcd has fewer points than its selected voxels");
    }
    const auto *data = cloud.data.data() + index * cloud.point_step;
    for (std::size_t f = 0; f < kFields.size(); ++f) {
      float value = 0.0F;
      std::memcpy(&value, data + offsets[f], sizeof(value));
      const float expected = f == 0 ? voxel->center.x()
                             : f == 1 ? voxel->center.y()
                             : f == 2 ? voxel->center.z()
                                      : voxel->final_confidence;
      if (!std::isfinite(value) || !close_to(value, expected)) {
        throw std::invalid_argument("stable_map.pcd content conflicts with confidence evidence");
      }
    }
    ++index;
  }
  if (index != cloud.width) {
    throw std::invalid_argument("stable_map.pcd has extra points not selected by confidence evidence");
  }
}

}  // namespace

bool SpatialConfidenceLoader::load(const fs::path &derivative_dir,
                                    const fs::path &expected_parent,
                                    SpatialConfidenceModel *model, std::string *error) {
  if (!model) {
    if (error) *error = "SpatialConfidenceModel must not be null";
    return false;
  }
  try {
    if (fs::is_symlink(fs::symlink_status(derivative_dir)) ||
        !fs::is_directory(derivative_dir)) {
      throw std::invalid_argument("Confidence artifact directory missing or symbolic link");
    }
    const auto root = fs::canonical(derivative_dir);
    verify_checksums(root);
    const auto metadata = YAML::LoadFile((root / "confidence_metadata.yaml").string());
    schema_version(metadata, "confidence");
    if (!metadata["artifact_type"] ||
        metadata["artifact_type"].as<std::string>() != "spatial_confidence_v1" ||
        !metadata["confidence_semantics"] ||
        metadata["confidence_semantics"].as<std::string>() != "single_session_observation_evidence" ||
        !metadata["frame_id"] || metadata["frame_id"].as<std::string>() != "map") {
      throw std::invalid_argument("Unsupported confidence metadata artifact/semantic/frame");
    }
    const auto source = metadata["source"];
    const auto parameters = metadata["parameters"];
    if (!source || !source.IsMap() || !parameters || !parameters.IsMap()) {
      throw std::invalid_argument("confidence metadata source/parameters missing");
    }
    const auto geometry = metadata["geometry"];
    if (!geometry || !geometry.IsMap() || !geometry["mode"] || !geometry["score"] ||
        geometry["mode"].as<std::string>() != "deferred" ||
        geometry["score"].as<float>() != 1.0F) {
      throw std::invalid_argument("V1 geometry must remain deferred with an unedited score of 1");
    }
    ConfidenceArtifactInfo info;
    info.derivative_dir = root;
    if (expected_parent.empty() || !fs::is_directory(expected_parent)) {
      throw std::invalid_argument("Open a validated mapping package before confidence artifacts");
    }
    info.source_package = fs::canonical(source["map_package"].as<std::string>());
    if (info.source_package != fs::canonical(expected_parent)) {
      throw std::invalid_argument("confidence metadata parent does not match opened mapping package");
    }
    info.parent_manifest_sha256 = source["parent_manifest_sha256"].as<std::string>();
    info.parent_checksums_sha256 = source["parent_checksums_sha256"].as<std::string>();
    if (info.parent_manifest_sha256 != agt_spatial_map_core::sha256_file(
            info.source_package / "manifest.yaml") ||
        info.parent_checksums_sha256 != agt_spatial_map_core::sha256_file(
            info.source_package / "checksums.sha256")) {
      throw std::invalid_argument("confidence artifact parent digest mismatch");
    }
    if (!parameters["voxel_size"]) throw std::invalid_argument("Invalid voxel_size: missing");
    info.voxel_size = parameters["voxel_size"].as<float>();
    if (!std::isfinite(info.voxel_size) || info.voxel_size <= 0.0F) {
      throw std::invalid_argument("Invalid voxel_size (expected positive finite)");
    }
    info.stable_threshold = parameters["stable_threshold"].as<float>();
    info.force_low_value = parameters["force_low_value"].as<float>();
    unit_value(info.stable_threshold, "stable_threshold");
    unit_value(info.force_low_value, "force_low_value");
    info.source_keyframes = metadata["counts"]["source_keyframes"].as<std::uint64_t>();
    info.stable_voxels = metadata["counts"]["stable_voxels"].as<std::uint64_t>();
    auto voxels = read_voxels(root / "confidence_voxels.pcd",
                              metadata["confidence_voxels_pcd_fields"], info);
    if (metadata["counts"]["confidence_voxels"].as<std::uint64_t>() != voxels.size()) {
      throw std::invalid_argument("confidence PCD voxel count conflicts with metadata");
    }
    const auto override_count = read_overrides(root / "manual_overrides.yaml", info, &voxels);
    if (metadata["counts"]["manual_overrides"].as<std::uint64_t>() != override_count) {
      throw std::invalid_argument("manual override count conflicts with confidence metadata");
    }
    verify_stable_map(root / "stable_map.pcd", metadata, voxels, info.stable_threshold);
    SpatialConfidenceModel candidate;
    candidate.assign(std::move(info), std::move(voxels));
    if (candidate.stable_preview_count() != candidate.info().stable_voxels) {
      throw std::invalid_argument("confidence stable selection conflicts with metadata count");
    }
    *model = std::move(candidate);
    return true;
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
}

}  // namespace agt_map_studio
