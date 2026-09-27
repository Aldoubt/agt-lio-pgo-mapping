#include "agt_spatial_map_core/spatial_export.hpp"

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace agt_spatial_map_core;

namespace {
class ExportTest : public ::testing::Test {
protected:
  fs::path root_;
  fs::path parent_;
  ConfidenceParameters p_;
  SpatialEvidenceMap voxels_;
  void SetUp() override {
    static unsigned serial = 0;
    root_ = fs::temp_directory_path() /
            ("agt_spatial_export_" + std::to_string(getpid()) + "_" +
             std::to_string(++serial));
    parent_ = root_ / "map_package";
    fs::create_directories(parent_);
    file(parent_ / "manifest.yaml", "schema_version: 1\n");
    file(parent_ / "checksums.sha256", "fixture parent checksum\n");
    voxel(VoxelKey{0, 0, 0}, 3, 3);    // stable AUTO
    voxel(VoxelKey{1, 0, 0}, 3, 3);    // stable before FORCE_LOW
    voxel(VoxelKey{-2, 0, 0}, 1, 0);   // unstable before FORCE_HIGH
    voxel(VoxelKey{-3, 0, 0}, 1, 0);   // ignored
  }
  void TearDown() override { fs::remove_all(root_); }
  static void file(const fs::path &path, const std::string &contents) {
    std::ofstream out(path, std::ios::binary);
    out << contents;
  }
  void voxel(const VoxelKey &key, unsigned observations, unsigned last) {
    SpatialVoxelEvidence v;
    v.key = key;
    v.centroid = Eigen::Vector3f(key.x * p_.voxel_size + .05F, .05F, .05F);
    v.point_count = observations + 1;
    v.observed_keyframes = observations;
    v.first_keyframe = 0;
    v.last_keyframe = last;
    v.keyframe_span = last;
    calculate_confidence(&v, p_);
    voxels_.emplace(key, v);
  }
  SpatialExportOptions options(const std::string &name = "derivative") const {
    SpatialExportOptions o;
    o.parent_package = parent_;
    o.output_directory = root_ / name;
    o.parameters = p_;
    o.build_stats = EvidenceBuildStats{7, 20, 19, 1, 4};
    return o;
  }
  static unsigned offset(const pcl::PCLPointCloud2 &cloud, const std::string &name) {
    for (const auto &f : cloud.fields) if (f.name == name) return f.offset;
    throw std::runtime_error("missing confidence PCD field: " + name);
  }
  template <typename T>
  static T get(const pcl::PCLPointCloud2 &cloud, unsigned index, const std::string &name) {
    T value{};
    std::memcpy(&value, cloud.data.data() + index * cloud.point_step + offset(cloud, name),
                sizeof(value));
    return value;
  }
};

TEST_F(ExportTest, ExportsFiveChecksumCoveredArtifactsWithSeparateAutomaticAndManualValues) {
  const fs::path overrides = root_ / "input_overrides.yaml";
  file(overrides, "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 0.20\n"
                  "overrides:\n"
                  "  - {key: [1, 0, 0], mode: FORCE_LOW, value: 0.02}\n"
                  "  - {key: [-2, 0, 0], mode: FORCE_HIGH}\n"
                  "  - {key: [-3, 0, 0], mode: IGNORE}\n");
  auto opts = options();
  opts.manual_overrides = overrides;
  const auto result = export_spatial_artifacts(&voxels_, opts);
  EXPECT_EQ(result.voxel_count, 4U);
  EXPECT_EQ(result.stable_voxel_count, 2U);
  EXPECT_EQ(result.manual_override_count, 3U);
  EXPECT_TRUE(fs::exists(result.output_directory / "checksums.sha256"));

  const auto metadata = YAML::LoadFile((result.output_directory / "confidence_metadata.yaml").string());
  EXPECT_EQ(metadata["schema_version"].as<int>(), 1);
  EXPECT_EQ(metadata["geometry"]["mode"].as<std::string>(), "deferred");
  EXPECT_EQ(metadata["confidence_semantics"].as<std::string>(),
            "single_session_observation_evidence");
  EXPECT_EQ(metadata["source"]["parent_manifest_sha256"].as<std::string>(),
            sha256_file(parent_ / "manifest.yaml"));
  EXPECT_EQ(metadata["counts"]["stable_voxels"].as<unsigned>(), 2U);

  pcl::PCLPointCloud2 cloud;
  ASSERT_EQ(pcl::io::loadPCDFile((result.output_directory / "confidence_voxels.pcd").string(),
                                 cloud), 0);
  ASSERT_EQ(cloud.width, 4U);
  EXPECT_NE(offset(cloud, "auto_confidence"), offset(cloud, "final_confidence"));
  // Sorted by signed voxel indices: -3 IGNORE, -2 FORCE_HIGH, 0 AUTO, 1 FORCE_LOW.
  EXPECT_EQ(get<double>(cloud, 0, "voxel_x"), -3.0);
  EXPECT_EQ(get<std::uint32_t>(cloud, 0, "override_mode"), 3U);
  EXPECT_EQ(get<float>(cloud, 0, "final_confidence"), 0.0F);
  EXPECT_EQ(get<float>(cloud, 1, "auto_confidence"), 0.0F);
  EXPECT_EQ(get<float>(cloud, 1, "final_confidence"), 1.0F);
  EXPECT_EQ(get<std::uint32_t>(cloud, 2, "override_mode"), 0U);
  EXPECT_GT(get<float>(cloud, 2, "auto_confidence"), 0.60F);
  EXPECT_NEAR(get<float>(cloud, 3, "final_confidence"), 0.02F, 1e-5F);
  EXPECT_GT(get<float>(cloud, 3, "auto_confidence"), 0.60F);
  EXPECT_EQ(get<std::uint32_t>(cloud, 3, "has_manual_value"), 1U);

  pcl::PointCloud<pcl::PointXYZI> stable;
  ASSERT_EQ(pcl::io::loadPCDFile((result.output_directory / "stable_map.pcd").string(),
                                 stable), 0);
  ASSERT_EQ(stable.size(), 2U);
  EXPECT_NEAR(stable[0].intensity, 1.0F, 1e-6F);  // FORCE_HIGH
  EXPECT_GT(stable[1].intensity, .60F);          // AUTO
  EXPECT_EQ(YAML::LoadFile((result.output_directory / "manual_overrides.yaml").string())
                ["overrides"].size(), 3U);

  std::ifstream checksum_stream(result.output_directory / "checksums.sha256");
  std::string digest, filename;
  int covered = 0;
  while (checksum_stream >> digest >> filename) {
    EXPECT_EQ(digest, sha256_file(result.output_directory / filename));
    ++covered;
  }
  EXPECT_EQ(covered, 4);
  EXPECT_EQ(sha256_file(parent_ / "manifest.yaml"),
            metadata["source"]["parent_manifest_sha256"].as<std::string>());
}

TEST_F(ExportTest, V1AuditExtensionPreservesReasonTimeAndEditorWithoutChangingGeometry) {
  const auto input = root_ / "reviewed_overrides.yaml";
  file(input, "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 0.2\n"
              "overrides:\n"
              "  - {key: [-2, 0, 0], mode: FORCE_HIGH, reason: MANUAL_ANCHOR, "
              "edited_at: '2026-09-27T12:34:56Z', editor: yangxuan}\n"
              "  - {key: [1, 0, 0], mode: FORCE_LOW, value: 0.1, "
              "reason: LOW_GEOMETRY, edited_at: '2026-09-27T12:35:06Z', editor: yangxuan}\n");
  auto opts = options("reviewed");
  opts.manual_overrides = input;
  const auto published = export_spatial_artifacts(&voxels_, opts);
  EXPECT_EQ(published.manual_override_count, 2U);
  EXPECT_EQ(published.stable_voxel_count, 2U);
  const auto entries = YAML::LoadFile((published.output_directory / "manual_overrides.yaml").string())
                           ["overrides"];
  ASSERT_EQ(entries.size(), 2U);
  EXPECT_EQ(entries[0]["reason"].as<std::string>(), "MANUAL_ANCHOR");
  EXPECT_EQ(entries[1]["reason"].as<std::string>(), "LOW_GEOMETRY");
  EXPECT_EQ(entries[1]["edited_at"].as<std::string>(), "2026-09-27T12:35:06Z");
  EXPECT_EQ(entries[1]["editor"].as<std::string>(), "yangxuan");
  EXPECT_FLOAT_EQ(voxels_.at(VoxelKey{1, 0, 0}).geometry_score, 1.0F);
  EXPECT_GT(voxels_.at(VoxelKey{1, 0, 0}).auto_confidence, .60F);
  EXPECT_FLOAT_EQ(voxels_.at(VoxelKey{1, 0, 0}).final_confidence, .1F);
}

TEST_F(ExportTest, RejectsMalformedOptionalAuditWithoutPublishing) {
  const auto input = root_ / "invalid_audit.yaml";
  for (const std::string bad : {
           "reason: LONG_TERM_STABILITY", "reason: ''", "edited_at: '2026-02-30T00:00:00Z'",
           "editor: 'with spaces'", "unexpected: true"}) {
    file(input, "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 0.2\n"
                "overrides: [{key: [-2, 0, 0], mode: FORCE_HIGH, " + bad + "}]\n");
    auto opts = options("invalid_audit");
    opts.manual_overrides = input;
    EXPECT_THROW(export_spatial_artifacts(&voxels_, opts), std::invalid_argument) << bad;
    EXPECT_FALSE(fs::exists(opts.output_directory));
  }
}

TEST_F(ExportTest, RejectsNonemptyOutputAndNeverReplacesParent) {
  const auto out = root_ / "existing";
  fs::create_directory(out);
  file(out / "keep.txt", "owner data");
  auto opts = options("existing");
  EXPECT_THROW(export_spatial_artifacts(&voxels_, opts), std::invalid_argument);
  EXPECT_EQ(fs::file_size(out / "keep.txt"), 10U);
  opts.output_directory = parent_ / "inside_original";
  EXPECT_THROW(export_spatial_artifacts(&voxels_, opts), std::invalid_argument);
  EXPECT_FALSE(fs::exists(parent_ / "inside_original"));
}

TEST_F(ExportTest, StagingFailureLeavesNoPublishedArtifactsEvenIfOutputWasEmpty) {
  const auto out = root_ / "empty";
  fs::create_directory(out);
  auto opts = options("empty");
  opts.before_publish = [] { throw std::runtime_error("injected pre-publication failure"); };
  EXPECT_THROW(export_spatial_artifacts(&voxels_, opts), std::runtime_error);
  EXPECT_TRUE(fs::is_empty(out));
  for (const auto &entry : fs::directory_iterator(root_)) {
    EXPECT_EQ(entry.path().filename().string().find(".empty.staging-"), std::string::npos);
  }
}

TEST_F(ExportTest, AbortsIfParentChecksumIndexChangesDuringBuild) {
  auto opts = options("not_published");
  opts.before_publish = [this] { file(parent_ / "checksums.sha256", "changed by another writer\n"); };
  EXPECT_THROW(export_spatial_artifacts(&voxels_, opts), std::runtime_error);
  EXPECT_FALSE(fs::exists(opts.output_directory));
}

TEST_F(ExportTest, RejectsMismatchedManualIndexGridAndUnknownVoxel) {
  const auto overrides = root_ / "invalid.yaml";
  file(overrides, "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 1\n"
                  "overrides: [{key: [0, 0, 0], mode: IGNORE}]\n");
  auto opts = options();
  opts.manual_overrides = overrides;
  EXPECT_THROW(export_spatial_artifacts(&voxels_, opts), std::invalid_argument);
  EXPECT_FALSE(fs::exists(opts.output_directory));
  file(overrides, "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 0.2\n"
                  "overrides: [{key: [999, 0, 0], mode: FORCE_HIGH}]\n");
  EXPECT_THROW(export_spatial_artifacts(&voxels_, opts), std::invalid_argument);
}

TEST_F(ExportTest, ZeroSelectedVoxelsCannotPublishAnUnreadableStableMap) {
  auto opts = options("empty_stable");
  opts.parameters.stable_threshold = 1.0F;
  EXPECT_THROW(export_spatial_artifacts(&voxels_, opts), std::runtime_error);
  EXPECT_FALSE(fs::exists(opts.output_directory));
  for (const auto &entry : fs::directory_iterator(root_)) {
    EXPECT_EQ(entry.path().filename().string().find(".empty_stable.staging-"),
              std::string::npos);
  }
}

TEST_F(ExportTest, ExistingEmptyDirectoryCanBeAtomicallyPublished) {
  const auto target = root_ / "existing_empty";
  fs::create_directory(target);
  const auto result = export_spatial_artifacts(&voxels_, options("existing_empty"));
  EXPECT_EQ(result.output_directory, target);
  EXPECT_TRUE(fs::exists(target / "checksums.sha256"));
}

TEST_F(ExportTest, VersionedConfigRejectsUnimplementedGeometryMode) {
  const auto config = root_ / "config.yaml";
  file(config, "schema_version: 1\ngeometry_mode: deferred\nvoxel_size: 0.25\n");
  EXPECT_FLOAT_EQ(load_confidence_config(config).voxel_size, 0.25F);
  file(config, "schema_version: 1\ngeometry_mode: pretend_multi_session\n");
  EXPECT_THROW(load_confidence_config(config), std::invalid_argument);
}
}  // namespace
