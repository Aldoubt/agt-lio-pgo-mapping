#include "confidence/SpatialConfidenceLoader.hpp"

#include <agt_spatial_map_core/spatial_export.hpp>

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace agt_map_studio;
namespace core = agt_spatial_map_core;

namespace {
class SpatialConfidenceLoaderTest : public ::testing::Test {
protected:
  fs::path root_;
  fs::path parent_;
  fs::path derivative_;
  void SetUp() override {
    static unsigned serial = 0;
    root_ = fs::temp_directory_path() /
            ("agt_studio_conf_loader_" + std::to_string(getpid()) + "_" +
             std::to_string(++serial));
    parent_ = root_ / "map_package";
    derivative_ = root_ / "spatial_confidence_auto";
    fs::create_directories(parent_);
    write(parent_ / "manifest.yaml", "schema_version: 1\n");
    write(parent_ / "checksums.sha256", "fixture checksum index\n");
    core::SpatialEvidenceMap evidence;
    auto add = [&evidence](const core::VoxelKey &key, std::uint32_t observations,
                            std::uint32_t span) {
      core::SpatialVoxelEvidence v;
      v.key = key;
      v.centroid = Eigen::Vector3f(key.x * 0.20F + .05F,
                                   key.y * 0.20F + .05F, .05F);
      v.point_count = observations + 2;
      v.observed_keyframes = observations;
      v.first_keyframe = 0;
      v.last_keyframe = span;
      v.keyframe_span = span;
      core::calculate_confidence(&v, core::ConfidenceParameters{});
      evidence.emplace(key, v);
    };
    add({-3, -2, 0}, 4, 4);  // stable automatic evidence
    add({2, 0, 0}, 1, 0);    // unstable automatic evidence
    core::SpatialExportOptions options;
    options.parent_package = parent_;
    options.output_directory = derivative_;
    options.build_stats = core::EvidenceBuildStats{5, 11, 11, 0, 2};
    core::export_spatial_artifacts(&evidence, options);
  }
  void TearDown() override { fs::remove_all(root_); }
  static void write(const fs::path &file, const std::string &text) {
    std::ofstream out(file, std::ios::binary | std::ios::trunc);
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
  }
  static std::string read(const fs::path &file) {
    std::ifstream in(file, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
  }
  void reseal() {
    const std::vector<std::string> names{
        "confidence_metadata.yaml", "confidence_voxels.pcd", "manual_overrides.yaml", "stable_map.pcd"};
    std::string lines;
    for (const auto &name : names) lines += core::sha256_file(derivative_ / name) + "  " + name + '\n';
    write(derivative_ / "checksums.sha256", lines);
  }
  void change_metadata(const std::function<void(YAML::Node &)> &edit) {
    auto node = YAML::LoadFile((derivative_ / "confidence_metadata.yaml").string());
    edit(node);
    YAML::Emitter yaml;
    yaml << node;
    write(derivative_ / "confidence_metadata.yaml", std::string(yaml.c_str()) + '\n');
    reseal();
  }
};

TEST_F(SpatialConfidenceLoaderTest, LoadsTrueSchemaWithSignedFloat64KeysAndTypedEvidence) {
  SpatialConfidenceModel model;
  std::string error;
  ASSERT_TRUE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error)) << error;
  ASSERT_EQ(model.voxels().size(), 2U);
  const auto *v = model.find(core::VoxelKey{-3, -2, 0});
  ASSERT_NE(v, nullptr);
  EXPECT_EQ(v->point_count, 6U);
  EXPECT_EQ(v->observed_keyframes, 4U);
  EXPECT_EQ(v->first_keyframe, 0U);
  EXPECT_EQ(v->last_keyframe, 4U);
  EXPECT_EQ(v->keyframe_span, 4U);
  EXPECT_GT(v->auto_confidence, .6F);
  EXPECT_FLOAT_EQ(v->auto_confidence, v->final_confidence);
  EXPECT_FLOAT_EQ(model.info().voxel_size, 0.2F);
  EXPECT_FLOAT_EQ(model.info().stable_threshold, .6F);
  EXPECT_EQ(model.stable_preview_count(), 1U);
  EXPECT_EQ(model.xyz().size(), model.voxels().size() * 3U);
  EXPECT_NE(model.find(core::VoxelKey{2, 0, 0}), nullptr);
}

TEST_F(SpatialConfidenceLoaderTest, RejectsMissingAndTamperedArtifact) {
  SpatialConfidenceModel model;
  std::string error;
  fs::remove(derivative_ / "manual_overrides.yaml");
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error));
  EXPECT_NE(error.find("manual_overrides.yaml"), std::string::npos);
  write(derivative_ / "manual_overrides.yaml", "overrides: []\n");
  error.clear();
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error));
  EXPECT_NE(error.find("checksum validation failed"), std::string::npos);
}

TEST_F(SpatialConfidenceLoaderTest, RejectsMissingPcdFieldEvenWithValidChecksums) {
  const auto path = derivative_ / "confidence_voxels.pcd";
  auto payload = read(path);
  const auto index = payload.find("auto_confidence");
  ASSERT_NE(index, std::string::npos);
  payload[index + 14] = 'X';  // same-length field rename in PCD header
  write(path, payload);
  reseal();
  SpatialConfidenceModel model;
  std::string error;
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error));
  EXPECT_NE(error.find("Missing PCD field: auto_confidence"), std::string::npos) << error;
}

TEST_F(SpatialConfidenceLoaderTest, RejectsUnsupportedSchemaAndInvalidVoxelSize) {
  SpatialConfidenceModel model;
  std::string error;
  change_metadata([](YAML::Node &n) { n["schema_version"] = 2; });
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error));
  EXPECT_EQ(error, "Unsupported confidence schema_version: 2");
  change_metadata([](YAML::Node &n) { n["schema_version"] = 1; n["parameters"]["voxel_size"] = 0.0; });
  error.clear();
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error));
  EXPECT_NE(error.find("Invalid voxel_size"), std::string::npos);
}

TEST_F(SpatialConfidenceLoaderTest, RejectsMismatchedDeclaredFieldsAndManualVoxelSize) {
  SpatialConfidenceModel model;
  std::string error;
  change_metadata([](YAML::Node &n) {
    n["confidence_voxels_pcd_fields"][14] = "unexpected_score";
  });
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error));
  EXPECT_NE(error.find("incompatible PCD fields"), std::string::npos);
  change_metadata([](YAML::Node &n) { n["confidence_voxels_pcd_fields"][14] = "auto_confidence"; });
  write(derivative_ / "manual_overrides.yaml",
        "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 0.4\noverrides: []\n");
  reseal();
  error.clear();
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error));
  EXPECT_NE(error.find("manual_overrides voxel_size does not match"), std::string::npos) << error;
}

TEST_F(SpatialConfidenceLoaderTest, RejectsMalformedOverrideModeAndUnknownTarget) {
  SpatialConfidenceModel model;
  std::string error;
  write(derivative_ / "manual_overrides.yaml",
        "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 0.2\n"
        "overrides: [{key: [-3, -2, 0], mode: NOT_A_MODE}]\n");
  reseal();
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error));
  EXPECT_NE(error.find("Malformed override mode"), std::string::npos) << error;
  write(derivative_ / "manual_overrides.yaml",
        "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 0.2\n"
        "overrides: [{key: [999, 0, 0], mode: IGNORE}]\n");
  reseal();
  error.clear();
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error));
  EXPECT_NE(error.find("unknown voxel"), std::string::npos) << error;
}

TEST_F(SpatialConfidenceLoaderTest, RejectsDuplicateVoxelKeysInPcd) {
  const auto path = derivative_ / "confidence_voxels.pcd";
  pcl::PCLPointCloud2 cloud;
  ASSERT_EQ(pcl::io::loadPCDFile(path.string(), cloud), 0);
  ASSERT_EQ(cloud.width, 2U);
  ASSERT_GE(cloud.point_step, 36U);
  std::memcpy(cloud.data.data() + cloud.point_step + 12,
              cloud.data.data() + 12, 24);  // voxel_x/y/z are three float64 values
  ASSERT_EQ(pcl::PCDWriter{}.writeBinary(path.string(), cloud), 0);
  reseal();
  SpatialConfidenceModel model;
  std::string error;
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error));
  EXPECT_NE(error.find("Duplicate confidence voxel key"), std::string::npos) << error;
}

TEST_F(SpatialConfidenceLoaderTest, RefusesDifferentOrModifiedParentPackage) {
  SpatialConfidenceModel model;
  std::string error;
  const auto unrelated = root_ / "unrelated_package";
  fs::create_directory(unrelated);
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, unrelated, &model, &error));
  EXPECT_NE(error.find("parent does not match"), std::string::npos) << error;
  write(parent_ / "manifest.yaml", "changed parent\n");
  error.clear();
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error));
  EXPECT_NE(error.find("parent digest mismatch"), std::string::npos) << error;
}

TEST(SpatialConfidenceLoaderRealArtifactTest, LoadsOptionalRealPgoDerivative) {
  const char *derivative = std::getenv("AGT_SPATIAL_DERIVATIVE_TEST_DIR");
  const char *parent = std::getenv("AGT_SPATIAL_PARENT_TEST_DIR");
  if (!derivative || !parent || !*derivative || !*parent) {
    GTEST_SKIP() << "Set AGT_SPATIAL_DERIVATIVE_TEST_DIR and AGT_SPATIAL_PARENT_TEST_DIR";
  }
  const auto start = std::chrono::steady_clock::now();
  SpatialConfidenceModel model;
  std::string error;
  ASSERT_TRUE(SpatialConfidenceLoader::load(derivative, parent, &model, &error)) << error;
  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_EQ(model.voxels().size(), 333839U);
  EXPECT_EQ(model.stable_preview_count(), 123321U);
  std::cout << "REAL_CONFIDENCE_LOAD_SECONDS=" << elapsed << '\n';
}

TEST_F(SpatialConfidenceLoaderTest, FailureNeverOverwritesPreviouslyLoadedModel) {
  SpatialConfidenceModel model;
  std::string error;
  ASSERT_TRUE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error)) << error;
  const auto key = core::VoxelKey{-3, -2, 0};
  write(derivative_ / "manual_overrides.yaml", "bad\n");
  EXPECT_FALSE(SpatialConfidenceLoader::load(derivative_, parent_, &model, &error));
  EXPECT_NE(model.find(key), nullptr);
}
}  // namespace
