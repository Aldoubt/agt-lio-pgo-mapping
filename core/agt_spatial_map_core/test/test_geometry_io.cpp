#include "agt_spatial_map_core/geometry_evidence.hpp"
#include "agt_spatial_map_core/spatial_evidence_builder.hpp"
#include "agt_spatial_map_core/spatial_export.hpp"

#include <gtest/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace agt_spatial_map_core;

namespace {
class GeometryIoTest : public ::testing::Test {
protected:
  fs::path root_, parent_, source_, output_;
  SpatialEvidenceMap confidence_;
  EvidenceBuildStats stats_;
  GeometryEvidence geometry_;

  static void put(const fs::path &path, const std::string &text) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    if (!(file << text)) throw std::runtime_error("cannot write geometry fixture");
  }
  void SetUp() override {
    static unsigned serial = 0;
    root_ = fs::temp_directory_path() /
        ("agt_geometry_io_" + std::to_string(getpid()) + "_" + std::to_string(++serial));
    parent_ = root_ / "immutable_map_package";
    source_ = root_ / "immutable_confidence_v1";
    output_ = root_ / "new_geometry_sidecar";
    fs::create_directories(parent_ / "patches");
    put(parent_ / "manifest.yaml", "schema_version: 1\nfixture: optimized PGO\n");
    put(parent_ / "checksums.sha256", "fixture parent index\n");
    pcl::PointCloud<pcl::PointXYZI> patch;
    for (int i = -6; i <= 6; ++i) {
      for (int j = -6; j <= 6; ++j) {
        pcl::PointXYZI p;
        p.x = (i + .5F) * .2F; p.y = (j + .5F) * .2F;
        p.z = .05F; p.intensity = 1.0F;
        patch.push_back(p);
      }
    }
    std::ofstream poses(parent_ / "poses_timed.txt");
    for (int frame = 0; frame < 6; ++frame) {
      const auto name = "frame_" + std::to_string(frame) + ".pcd";
      if (pcl::io::savePCDFileBinary((parent_ / "patches" / name).string(), patch) != 0) {
        throw std::runtime_error("cannot write synthetic patch");
      }
      poses << name << " 12345 " << frame * .15 << " 0 1 1 0 0 0\n";
    }
    poses.close();
    confidence_ = SpatialEvidenceBuilder::build(parent_, ConfidenceParameters{}, &stats_);
    SpatialExportOptions options;
    options.parent_package = parent_;
    options.output_directory = source_;
    options.build_stats = stats_;
    export_spatial_artifacts(&confidence_, options);
    geometry_ = GeometryEvidenceEstimator::build(parent_, confidence_, .2F,
        GeometryParameters{}, stats_.source_keyframes, stats_.usable_points);
  }
  void TearDown() override { fs::remove_all(root_); }
  GeometryExportOptions options() const {
    GeometryExportOptions x;
    x.parent_package = parent_; x.confidence_source = source_;
    x.output_directory = output_;
    return x;
  }
  void index() const {
    put(output_ / "checksums.sha256",
        sha256_file(output_ / "geometry_metadata.yaml") + "  geometry_metadata.yaml\n" +
        sha256_file(output_ / "geometry_voxels.pcd") + "  geometry_voxels.pcd\n");
  }
};

TEST_F(GeometryIoTest, IndependentRoundTripChecksAllKeysMetricsAndImmutableSources) {
  const auto before_pgo = sha256_file(parent_ / "manifest.yaml");
  const auto before_v1 = sha256_file(source_ / "checksums.sha256");
  const auto before_stable = sha256_file(source_ / "stable_map.pcd");
  const auto before_confidence = sha256_file(source_ / "confidence_voxels.pcd");
  export_geometry_evidence(geometry_, options());
  ASSERT_TRUE(fs::exists(output_ / "geometry_voxels.pcd"));
  ASSERT_TRUE(fs::exists(output_ / "geometry_metadata.yaml"));
  ASSERT_TRUE(fs::exists(output_ / "checksums.sha256"));
  EXPECT_EQ(std::distance(fs::directory_iterator(output_), fs::directory_iterator{}), 3);
  const auto result = load_geometry_evidence(output_, parent_, source_);
  ASSERT_EQ(result.voxels.size(), confidence_.size());
  EXPECT_EQ(result.source_observations, stats_.usable_points);
  std::size_t valid = 0;
  for (const auto &v : result.voxels) {
    ++valid; // every PCD key is checked in the core loader against V1
    if (v.translation.valid) {
      EXPECT_GE(v.translation.isotropy, 0);
      EXPECT_LE(v.translation.isotropy, 1);
      EXPECT_TRUE(v.translation.eigenvalues.allFinite());
    }
  }
  EXPECT_GT(valid, 50U);
  auto yaml = YAML::LoadFile((output_ / "geometry_metadata.yaml").string());
  EXPECT_EQ(yaml["source"]["parent_manifest_sha256"].as<std::string>(), before_pgo);
  EXPECT_EQ(yaml["source"]["confidence_checksums_sha256"].as<std::string>(), before_v1);
  EXPECT_EQ(yaml["artifact_type"].as<std::string>(), "spatial_geometry_evidence_v1");
  EXPECT_EQ(yaml["frame_id"].as<std::string>(), "map");
  EXPECT_EQ(before_pgo, sha256_file(parent_ / "manifest.yaml"));
  EXPECT_EQ(before_v1, sha256_file(source_ / "checksums.sha256"));
  EXPECT_EQ(before_stable, sha256_file(source_ / "stable_map.pcd"));
  EXPECT_EQ(before_confidence, sha256_file(source_ / "confidence_voxels.pcd"));
  for (const auto &[key, v] : confidence_) EXPECT_EQ(v.geometry_score, 1.0F);
  EXPECT_THROW(export_geometry_evidence(geometry_, options()), std::invalid_argument);
}

TEST_F(GeometryIoTest, ExplicitInvalidNaNsSurviveBinaryPcdAndValidation) {
  GeometryParameters p;
  p.normal_min_points = 100000;
  auto invalid = GeometryEvidenceEstimator::build(parent_, confidence_, .2F, p,
      stats_.source_keyframes, stats_.usable_points);
  export_geometry_evidence(invalid, options());
  const auto result = load_geometry_evidence(output_, parent_, source_);
  for (const auto &v : result.voxels) {
    EXPECT_FALSE(v.normal_valid);
    EXPECT_FALSE(v.translation.valid);
    EXPECT_FALSE(v.rotation.valid);
    EXPECT_EQ(v.valid_normal_voxels, 0U);
    EXPECT_TRUE(std::isnan(v.normal.x()));
    EXPECT_TRUE(std::isnan(v.translation.isotropy));
    EXPECT_TRUE(std::isnan(v.rotation.condition));
  }
  pcl::PCLPointCloud2 cloud;
  ASSERT_EQ(pcl::io::loadPCDFile((output_ / "geometry_voxels.pcd").string(), cloud), 0);
  const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
      [](const auto &f) { return f.name == "translation_valid"; });
  ASSERT_NE(field, cloud.fields.end());
  const std::uint32_t lying_flag = 1;
  std::memcpy(cloud.data.data() + field->offset, &lying_flag, sizeof(lying_flag));
  ASSERT_EQ(pcl::PCDWriter{}.writeBinary((output_ / "geometry_voxels.pcd").string(), cloud), 0);
  index(); // recomputed checksum cannot disguise a semantic fabrication
  EXPECT_THROW(load_geometry_evidence(output_, parent_, source_), std::invalid_argument);
}

TEST_F(GeometryIoTest, RejectsSourceTamperingOverlapSymlinkAndNonemptyTargets) {
  auto opts = options();
  opts.output_directory = source_ / "nested";
  EXPECT_ANY_THROW(export_geometry_evidence(geometry_, opts));
  opts.output_directory = parent_ / "nested";
  EXPECT_ANY_THROW(export_geometry_evidence(geometry_, opts));
  fs::create_directory(output_);
  put(output_ / "KEEP", "untouched\n");
  EXPECT_THROW(export_geometry_evidence(geometry_, options()), std::invalid_argument);
  EXPECT_TRUE(fs::exists(output_ / "KEEP"));
  fs::remove_all(output_);
  fs::create_directory_symlink(source_, output_);
  EXPECT_THROW(export_geometry_evidence(geometry_, options()), std::invalid_argument);
  fs::remove(output_);
  put(source_ / "manual_overrides.yaml", "source modified without index update\n");
  EXPECT_ANY_THROW(export_geometry_evidence(geometry_, options()));
  EXPECT_FALSE(fs::exists(output_));
}

TEST_F(GeometryIoTest, PublicationStageRollbackAndRecheck) {
  auto opts = options();
  opts.before_publish = [this] { put(parent_ / "manifest.yaml", "modified parent\n"); };
  EXPECT_THROW(export_geometry_evidence(geometry_, opts), std::runtime_error);
  EXPECT_FALSE(fs::exists(output_));
  for (const auto &entry : fs::directory_iterator(root_)) {
    EXPECT_EQ(entry.path().filename().string().find(".new_geometry_sidecar.staging-"),
              std::string::npos);
  }
}

TEST_F(GeometryIoTest, SourceChangedDuringStagingRefusesAtomicPublication) {
  auto opts = options();
  const auto original_pgo = sha256_file(parent_ / "manifest.yaml");
  opts.before_publish = [this] {
    put(source_ / "manual_overrides.yaml", "edited after geometry was staged\n");
  };
  EXPECT_THROW(export_geometry_evidence(geometry_, opts), std::invalid_argument);
  EXPECT_FALSE(fs::exists(output_));
  EXPECT_EQ(sha256_file(parent_ / "manifest.yaml"), original_pgo);
  for (const auto &entry : fs::directory_iterator(root_)) {
    EXPECT_EQ(entry.path().filename().string().find(".new_geometry_sidecar.staging-"),
              std::string::npos);
  }
}

TEST_F(GeometryIoTest, LoaderRejectsRehashedBadProvenancePcdSchemaAndUnsafeEntries) {
  export_geometry_evidence(geometry_, options());
  auto meta = YAML::LoadFile((output_ / "geometry_metadata.yaml").string());
  meta["source"]["confidence_checksums_sha256"] = std::string(64, '0');
  put(output_ / "geometry_metadata.yaml", YAML::Dump(meta) + "\n");
  index();
  EXPECT_THROW(load_geometry_evidence(output_, parent_, source_), std::invalid_argument);
  fs::remove_all(output_);
  export_geometry_evidence(geometry_, options());
  pcl::PCLPointCloud2 cloud;
  ASSERT_EQ(pcl::io::loadPCDFile((output_ / "geometry_voxels.pcd").string(), cloud), 0);
  cloud.fields[6].datatype = pcl::PCLPointField::FLOAT32; // normal_valid must be UINT32
  ASSERT_EQ(pcl::PCDWriter{}.writeBinary((output_ / "geometry_voxels.pcd").string(), cloud), 0);
  index();
  EXPECT_THROW(load_geometry_evidence(output_, parent_, source_), std::invalid_argument);
  fs::remove_all(output_);
  export_geometry_evidence(geometry_, options());
  fs::create_symlink(source_ / "checksums.sha256", output_ / "unlisted_symlink");
  EXPECT_THROW(load_geometry_evidence(output_, parent_, source_), std::invalid_argument);
}

TEST_F(GeometryIoTest, ConfigRequiresCompleteVersionedUnitSpecificEpsilon) {
  const auto config = root_ / "geometry.yaml";
  put(config, "schema_version: 1\nnormal: {radius: 0.4, min_points: 8}\n"
              "observability: {radius: 0.8, min_valid_normals: 6}\n"
              "epsilon: {normal_covariance_m2: 1e-6, translation: 1e-6, rotation_m2: 1e-6}\n");
  EXPECT_FLOAT_EQ(load_geometry_config(config).normal_radius, .4F);
  put(config, "schema_version: 1\nnormal: {radius: 0.4, min_points: 8}\n"
              "observability: {radius: 0.8, min_valid_normals: 6}\n"
              "epsilon: {translation: 1e-6, rotation_m2: 1e-6}\n");
  EXPECT_ANY_THROW(load_geometry_config(config));
  put(config, "schema_version: 2\nnormal: {radius: 0.4, min_points: 8}\n"
              "observability: {radius: 0.8, min_valid_normals: 6}\n"
              "epsilon: {normal_covariance_m2: 1e-6, translation: 1e-6, rotation_m2: 1e-6}\n");
  EXPECT_ANY_THROW(load_geometry_config(config));
}
}  // namespace
