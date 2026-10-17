#include "agt_spatial_map_core/spatial_export.hpp"

#include <gtest/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace agt_spatial_map_core;

namespace {
class SpatialReviewTest : public ::testing::Test {
protected:
  fs::path root_, parent_, source_, intent_;
  void SetUp() override {
    static unsigned serial = 0;
    root_ = fs::temp_directory_path() /
            ("agt_spatial_review_" + std::to_string(getpid()) + "_" +
             std::to_string(++serial));
    parent_ = root_ / "immutable_map_package";
    source_ = root_ / "immutable_derivative";
    intent_ = root_ / "saved_intent.yaml";
    fs::create_directories(parent_);
    write(parent_ / "manifest.yaml", "schema_version: 1\n");
    write(parent_ / "checksums.sha256", "fixture checksum index\n");
    SpatialEvidenceMap voxels;
    auto add = [&voxels](VoxelKey key, unsigned observed, unsigned span) {
      SpatialVoxelEvidence v;
      v.key = key;
      v.centroid = Eigen::Vector3f(key.x * .2F + .05F, .05F, .05F);
      v.point_count = observed + 2;
      v.observed_keyframes = observed;
      v.first_keyframe = 0;
      v.last_keyframe = span;
      v.keyframe_span = span;
      calculate_confidence(&v, ConfidenceParameters{});
      voxels.emplace(key, v);
    };
    add({0, 0, 0}, 4, 4);  // stable AUTO
    add({1, 0, 0}, 4, 4);  // stable AUTO
    add({-2, 0, 0}, 1, 0);  // unstable AUTO
    add({-3, 0, 0}, 1, 0);  // unstable AUTO
    write(root_ / "initial.yaml",
          "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 0.2\n"
          "overrides: [{key: [0, 0, 0], mode: IGNORE, reason: PARKING_AREA}, "
          "{key: [-2, 0, 0], mode: FORCE_HIGH, reason: MANUAL_ANCHOR}]\n");
    SpatialExportOptions options;
    options.parent_package = parent_;
    options.output_directory = source_;
    options.manual_overrides = root_ / "initial.yaml";
    options.build_stats = EvidenceBuildStats{7, 20, 19, 1, 4};
    export_spatial_artifacts(&voxels, options);
    write(intent_,
          "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 0.2\n"
          "overrides:\n"
          "  - {key: [1, 0, 0], mode: FORCE_LOW, value: 0.9, reason: LOW_GEOMETRY, "
          "edited_at: '2026-09-27T12:34:56Z', editor: yangxuan}\n"
          "  - {key: [-3, 0, 0], mode: FORCE_HIGH, reason: MANUAL_ANCHOR}\n");
  }
  void TearDown() override { fs::remove_all(root_); }
  static void write(const fs::path &path, const std::string &content) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!(file << content)) throw std::runtime_error("cannot write review test fixture");
  }
  static std::size_t position(const pcl::PCLPointCloud2 &cloud, std::int64_t x) {
    for (std::size_t i = 0; i < cloud.width; ++i) {
      double value = 0;
      std::memcpy(&value, cloud.data.data() + i * cloud.point_step + 12, sizeof(value));
      if (value == x) return i;
    }
    throw std::runtime_error("missing voxel in review fixture");
  }
  template <typename T>
  static T field(const pcl::PCLPointCloud2 &cloud, std::int64_t x, const char *name) {
    for (const auto &f : cloud.fields) {
      if (f.name != name) continue;
      T value{};
      std::memcpy(&value, cloud.data.data() + position(cloud, x) * cloud.point_step + f.offset,
                  sizeof(value));
      return value;
    }
    throw std::runtime_error("missing review field");
  }
};

TEST_F(SpatialReviewTest, PreservesEveryAutomaticEvidenceFieldAndRestoresRemovedOldOverride) {
  const auto original_manifest = sha256_file(parent_ / "manifest.yaml");
  const auto original_checksums = sha256_file(parent_ / "checksums.sha256");
  const auto original_pcd = sha256_file(source_ / "confidence_voxels.pcd");
  const auto original_source_index = sha256_file(source_ / "checksums.sha256");
  const auto reviewed = root_ / "reviewed_new_directory";
  const auto result = review_spatial_artifacts(source_, parent_, intent_, reviewed);
  EXPECT_EQ(result.output_directory, reviewed);
  EXPECT_EQ(result.voxel_count, 4U);
  EXPECT_EQ(result.stable_voxel_count, 2U);
  EXPECT_EQ(result.manual_override_count, 2U);
  pcl::PCLPointCloud2 before, after;
  ASSERT_EQ(pcl::io::loadPCDFile((source_ / "confidence_voxels.pcd").string(), before), 0);
  ASSERT_EQ(pcl::io::loadPCDFile((reviewed / "confidence_voxels.pcd").string(), after), 0);
  for (std::int64_t key : {-3, -2, 0, 1}) {
    for (const char *name : {"x", "y", "z", "observation_score", "persistence_score",
                             "geometry_score", "auto_confidence"}) {
      EXPECT_EQ(field<float>(before, key, name), field<float>(after, key, name)) << name << ' ' << key;
    }
    for (const char *name : {"point_count", "observed_keyframes", "first_keyframe",
                             "last_keyframe", "keyframe_span"}) {
      EXPECT_EQ(field<std::uint32_t>(before, key, name),
                field<std::uint32_t>(after, key, name)) << name << ' ' << key;
    }
  }
  EXPECT_EQ(field<std::uint32_t>(after, 0, "override_mode"), 0U);   // old IGNORE removed
  EXPECT_EQ(field<std::uint32_t>(after, -2, "override_mode"), 0U);  // old FORCE_HIGH removed
  EXPECT_EQ(field<float>(after, 0, "final_confidence"),
            field<float>(after, 0, "auto_confidence"));
  EXPECT_FLOAT_EQ(field<float>(after, 1, "final_confidence"), .9F);
  EXPECT_EQ(field<std::uint32_t>(after, 1, "override_mode"), 2U);
  EXPECT_EQ(field<float>(after, -3, "final_confidence"), 1.0F);
  auto metadata = YAML::LoadFile((reviewed / "confidence_metadata.yaml").string());
  EXPECT_EQ(metadata["review"]["automatic_evidence"].as<std::string>(),
            "copied_from_verified_source_without_recomputation");
  EXPECT_EQ(metadata["review"]["source_checksums_sha256"].as<std::string>(), original_source_index);
  EXPECT_EQ(metadata["review"]["input_overrides_sha256"].as<std::string>(), sha256_file(intent_));
  std::ifstream reviewed_yaml(reviewed / "manual_overrides.yaml");
  const std::string reviewed_text((std::istreambuf_iterator<char>(reviewed_yaml)),
                                  std::istreambuf_iterator<char>());
  EXPECT_NE(reviewed_text.find("edited_at: \"2026-09-27T12:34:56Z\""), std::string::npos)
      << "The audited time must remain a YAML string, not an implicit timestamp";
  const auto entries = YAML::LoadFile((reviewed / "manual_overrides.yaml").string())["overrides"];
  ASSERT_EQ(entries.size(), 2U);
  EXPECT_EQ(entries[1]["reason"].as<std::string>(), "LOW_GEOMETRY");
  EXPECT_EQ(entries[1]["editor"].as<std::string>(), "yangxuan");
  EXPECT_EQ(original_manifest, sha256_file(parent_ / "manifest.yaml"));
  EXPECT_EQ(original_checksums, sha256_file(parent_ / "checksums.sha256"));
  EXPECT_EQ(original_pcd, sha256_file(source_ / "confidence_voxels.pcd"));
  EXPECT_EQ(original_source_index, sha256_file(source_ / "checksums.sha256"));
  std::ifstream index(reviewed / "checksums.sha256");
  std::string digest, filename;
  int covered = 0;
  while (index >> digest >> filename) {
    EXPECT_EQ(digest, sha256_file(reviewed / filename));
    ++covered;
  }
  EXPECT_EQ(covered, 4);
}

TEST_F(SpatialReviewTest, RejectsUnsafeSourceIntentAndTargetWithoutPublishing) {
  const auto output = root_ / "failed";
  write(intent_, "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 1\n"
                 "overrides: [{key: [0, 0, 0], mode: FORCE_HIGH}]\n");
  EXPECT_THROW(review_spatial_artifacts(source_, parent_, intent_, output), std::invalid_argument);
  EXPECT_FALSE(fs::exists(output));
  write(intent_, "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 0.2\n"
                 "overrides: [{key: [999, 0, 0], mode: FORCE_HIGH}]\n");
  EXPECT_THROW(review_spatial_artifacts(source_, parent_, intent_, output), std::invalid_argument);
  EXPECT_FALSE(fs::exists(output));
  write(intent_, "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 0.2\n"
                 "overrides: [{key: [0, 0, 0], mode: FORCE_HIGH, reason: FALSE_PROBABILITY}]\n");
  EXPECT_THROW(review_spatial_artifacts(source_, parent_, intent_, output), std::invalid_argument);
  EXPECT_FALSE(fs::exists(output));
  EXPECT_THROW(review_spatial_artifacts(source_, parent_, intent_, source_), std::invalid_argument);
  EXPECT_THROW(review_spatial_artifacts(source_, parent_, intent_, parent_ / "new"), std::invalid_argument);
  fs::create_directory(output);
  write(output / "keep.txt", "do not overwrite");
  EXPECT_THROW(review_spatial_artifacts(source_, parent_, intent_, output), std::invalid_argument);
  EXPECT_EQ(fs::file_size(output / "keep.txt"), 16U);
}

TEST_F(SpatialReviewTest, RejectsTamperingWithChecksumCoveredSourceOrChangedParent) {
  const auto output = root_ / "refuse";
  std::ifstream previous_file(source_ / "manual_overrides.yaml");
  const std::string previous((std::istreambuf_iterator<char>(previous_file)),
                             std::istreambuf_iterator<char>());
  write(source_ / "manual_overrides.yaml", "tampered\n");
  EXPECT_THROW(review_spatial_artifacts(source_, parent_, intent_, output), std::invalid_argument);
  EXPECT_FALSE(fs::exists(output));
  // Restore the exact source bytes, then independently change its PGO parent.
  write(source_ / "manual_overrides.yaml", previous);
  write(parent_ / "manifest.yaml", "modified parent\n");
  EXPECT_THROW(review_spatial_artifacts(source_, parent_, intent_, output), std::invalid_argument);
  EXPECT_FALSE(fs::exists(output));
}

TEST_F(SpatialReviewTest, StagedPublicationRejectsSourcePCDMutationEvenWithUnchangedIndex) {
  const auto output = root_ / "refuse_changed_evidence";
  const auto before = sha256_file(source_ / "confidence_voxels.pcd");
  EXPECT_THROW(review_spatial_artifacts(source_, parent_, intent_, output, [this] {
    std::ofstream tamper(source_ / "confidence_voxels.pcd", std::ios::binary | std::ios::app);
    tamper << "unexpected source change";
  }), std::invalid_argument);
  EXPECT_NE(before, sha256_file(source_ / "confidence_voxels.pcd"));
  EXPECT_FALSE(fs::exists(output));
  for (const auto &item : fs::directory_iterator(root_)) {
    EXPECT_EQ(item.path().filename().string().find(".refuse_changed_evidence.staging-"),
              std::string::npos);
  }
}

TEST_F(SpatialReviewTest, StagedPublicationAbortsOnChangedInputsAndCleansUp) {
  const auto output = root_ / "refuse_changed_intent";
  EXPECT_THROW(review_spatial_artifacts(source_, parent_, intent_, output,
      [this] { write(intent_, "changed while building\n"); }), std::runtime_error);
  EXPECT_FALSE(fs::exists(output));
  for (const auto &item : fs::directory_iterator(root_)) {
    EXPECT_EQ(item.path().filename().string().find(".refuse_changed_intent.staging-"),
              std::string::npos);
  }
}
}  // namespace
