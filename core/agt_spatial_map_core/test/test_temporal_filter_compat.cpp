#include "agt_spatial_map_core/spatial_evidence_builder.hpp"
#include "agt_pcd2grid_exporter/TemporalPersistenceFilter.hpp"

#include <gtest/gtest.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <unistd.h>

namespace fs = std::filesystem;
using agt_spatial_map_core::SpatialEvidenceBuilder;
using agt_spatial_map_core::SpatialEvidenceMap;
using agt_spatial_map_core::VoxelKey;
using agt_spatial_map_core::VoxelKeyHash;
using agt_spatial_map_core::voxel_for;
using agt_pcd2grid_exporter::ProjectionParameters;
using agt_pcd2grid_exporter::TemporalFilterStats;
using agt_pcd2grid_exporter::TemporalPersistenceFilter;

namespace {
class TemporalCompatibilityTest : public ::testing::Test {
protected:
  fs::path parent_;
  void SetUp() override {
    static unsigned serial = 0;
    parent_ = fs::temp_directory_path() /
              ("agt_temporal_compat_" + std::to_string(getpid()) + "_" +
               std::to_string(++serial));
    fs::create_directories(parent_ / "patches");
  }
  void TearDown() override { fs::remove_all(parent_); }

  void fixture() {
    const Eigen::Vector3f a(-0.45F, 0.35F, 0.15F);
    const Eigen::Vector3f b(1.05F, -0.35F, 0.15F);
    const Eigen::Vector3f c(1.45F, 0.15F, 0.15F);
    const Eigen::Vector3f d(2.05F, 0.15F, 0.15F);
    const Eigen::Vector3f e(-0.41F, 1.15F, 0.15F);
    std::ofstream poses(parent_ / "poses_timed.txt");
    for (int frame = 0; frame != 7; ++frame) {
      const Eigen::Vector3f translation(frame * .21F, -frame * .03F, .06F);
      const Eigen::Quaternionf rotation(
          Eigen::AngleAxisf(frame * .08F, Eigen::Vector3f::UnitZ()));
      const std::string name = "frame_" + std::to_string(frame) + ".pcd";
      pcl::PointCloud<pcl::PointXYZI> cloud;
      auto append = [&](const Eigen::Vector3f &world, int copies = 1) {
        for (int i = 0; i < copies; ++i) {
          const auto body = rotation.conjugate() * (world - translation);
          pcl::PointXYZI point;
          point.x = body.x(); point.y = body.y(); point.z = body.z();
          point.intensity = 1.0F;
          cloud.push_back(point);
        }
      };
      if (frame == 0 || frame == 1 || frame == 2 || frame == 3 || frame == 6) {
        append(a, frame == 0 ? 30 : frame == 6 ? 10 : 1);
      }
      if (frame == 1 || frame == 2) append(b, frame == 1 ? 5 : 1);
      if (frame == 0 || frame == 4) append(c);
      if (frame == 1 || frame == 3 || frame == 6) append(d);
      if (frame == 5) append(e);
      ASSERT_FALSE(cloud.empty());
      ASSERT_EQ(pcl::io::savePCDFileBinary((parent_ / "patches" / name).string(), cloud), 0);
      poses << std::setprecision(12) << name << " 12345 "
            << translation.x() << ' ' << translation.y() << ' ' << translation.z() << ' '
            << rotation.w() << ' ' << rotation.x() << ' ' << rotation.y() << ' '
            << rotation.z() << '\n';
    }
  }

  using Counts = std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash>;

  static Counts select_from_new_evidence(const SpatialEvidenceMap &voxels,
                                          std::uint32_t minimum_frames,
                                          std::uint32_t minimum_span) {
    Counts selected;
    for (const auto &[key, v] : voxels) {
      if (v.observed_keyframes >= minimum_frames && v.keyframe_span >= minimum_span) {
        selected[key] = v.point_count;
      }
    }
    return selected;
  }

  Counts retained_by_unchanged_legacy_filter(std::uint32_t minimum_frames,
                                              std::uint32_t minimum_span,
                                              TemporalFilterStats *stats) {
    ProjectionParameters parameters;
    parameters.temporal_voxel_size = .2F;
    parameters.temporal_min_observations = minimum_frames;
    parameters.temporal_min_keyframe_span = minimum_span;
    pcl::PCLPointCloud2 cloud;
    std::string error;
    EXPECT_TRUE(TemporalPersistenceFilter::filter_package(
        parent_.string(), parameters, &cloud, stats, &error)) << error;
    pcl::PointCloud<pcl::PointXYZI> retained;
    pcl::fromPCLPointCloud2(cloud, retained);
    Counts selected;
    for (const auto &p : retained) {
      const auto key = voxel_for(Eigen::Vector3f(p.x, p.y, p.z),
                                 parameters.temporal_voxel_size);
      ++selected[key];
    }
    EXPECT_EQ(stats->retained_points, retained.size());
    return selected;
  }
};

TEST_F(TemporalCompatibilityTest, ExactLegacySelectionAcrossObservationAndSpanThresholds) {
  fixture();
  agt_spatial_map_core::EvidenceBuildStats build_stats;
  const auto evidence = SpatialEvidenceBuilder::build(
      parent_, agt_spatial_map_core::ConfidenceParameters{}, &build_stats);
  ASSERT_EQ(build_stats.source_keyframes, 7U);
  ASSERT_EQ(build_stats.input_points, 55U);
  ASSERT_EQ(build_stats.usable_points, 55U);
  const auto &a = evidence.at(VoxelKey{-3, 1, 0});
  EXPECT_EQ(a.point_count, 43U);
  EXPECT_EQ(a.observed_keyframes, 5U);  // not 43 independent observations
  EXPECT_EQ(a.first_keyframe, 0U);
  EXPECT_EQ(a.last_keyframe, 6U);
  EXPECT_EQ(a.keyframe_span, 6U);
  EXPECT_EQ(evidence.at(VoxelKey{5, -2, 0}).keyframe_span, 1U);
  for (const auto [min_frames, min_span] : {
           std::pair<std::uint32_t, std::uint32_t>{1, 0}, {2, 2}, {3, 4}, {4, 5}}) {
    SCOPED_TRACE("minimum frames=" + std::to_string(min_frames) +
                 ", span=" + std::to_string(min_span));
    TemporalFilterStats legacy_stats;
    const auto legacy = retained_by_unchanged_legacy_filter(min_frames, min_span, &legacy_stats);
    const auto expected = select_from_new_evidence(evidence, min_frames, min_span);
    EXPECT_EQ(legacy, expected);
    EXPECT_EQ(legacy_stats.keyframes, build_stats.source_keyframes);
    EXPECT_EQ(legacy_stats.input_points, build_stats.usable_points);
  }
  const ProjectionParameters legacy_defaults;
  EXPECT_TRUE(legacy_defaults.temporal_filter_enabled);
  EXPECT_EQ(legacy_defaults.temporal_voxel_size, .2F);
  EXPECT_EQ(legacy_defaults.temporal_min_observations, 2U);
  EXPECT_EQ(legacy_defaults.temporal_min_keyframe_span, 2U);
}

TEST_F(TemporalCompatibilityTest, BothPathsRejectMissingKeyframePatch) {
  std::ofstream(parent_ / "poses_timed.txt") << "missing.pcd 1 0 0 0 1 0 0 0\n";
  EXPECT_THROW(SpatialEvidenceBuilder::build(
      parent_, agt_spatial_map_core::ConfidenceParameters{}), std::runtime_error);
  pcl::PCLPointCloud2 cloud;
  TemporalFilterStats stats;
  std::string error;
  EXPECT_FALSE(TemporalPersistenceFilter::filter_package(
      parent_.string(), ProjectionParameters{}, &cloud, &stats, &error));
  EXPECT_NE(error.find("cannot load keyframe patch"), std::string::npos);
}
}  // namespace
