#include "agt_spatial_map_core/spatial_evidence_builder.hpp"

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace agt_spatial_map_core;

namespace {
class BuilderTest : public ::testing::Test {
protected:
  fs::path directory_;
  void SetUp() override {
    static unsigned serial = 0;
    directory_ = fs::temp_directory_path() /
                 ("agt_spatial_builder_" + std::to_string(getpid()) + "_" +
                  std::to_string(++serial));
    fs::create_directories(directory_ / "patches");
  }
  void TearDown() override { fs::remove_all(directory_); }

  void patch(const std::string &name, const std::vector<Eigen::Vector3f> &points) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    for (const auto &p : points) {
      pcl::PointXYZI point;
      point.x = p.x(); point.y = p.y(); point.z = p.z(); point.intensity = 1.0F;
      cloud.push_back(point);
    }
    ASSERT_EQ(pcl::io::savePCDFileBinary((directory_ / "patches" / name).string(), cloud), 0);
  }

  static void pose(std::ofstream &output, const std::string &name,
                   const Eigen::Vector3f &translation = Eigen::Vector3f::Zero(),
                   const Eigen::Quaternionf &q = Eigen::Quaternionf::Identity()) {
    output << std::setprecision(12) << name << " 12345 "
           << translation.x() << ' ' << translation.y() << ' ' << translation.z() << ' '
           << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << '\n';
  }
};

TEST_F(BuilderTest, CountsUniqueKeyframesAndSpanNotRawPointCount) {
  const Eigen::Vector3f world(1.05F, -0.35F, 0.15F);
  std::ofstream poses(directory_ / "poses_timed.txt");
  for (int k = 0; k != 7; ++k) {
    const Eigen::Vector3f translation(k * 0.37F, -k * 0.08F, 0.3F);
    const Eigen::Quaternionf rotation(Eigen::AngleAxisf(k * 0.07F, Eigen::Vector3f::UnitZ()));
    std::vector<Eigen::Vector3f> patch_points;
    if (k == 1 || k == 2 || k == 6) {
      const int copies = k == 1 ? 50 : 1;
      for (int j = 0; j != copies; ++j) {
        patch_points.push_back(rotation.conjugate() * (world - translation));
      }
    } else {
      patch_points.push_back(rotation.conjugate() *
                             (Eigen::Vector3f(10 + k, 2, 1) - translation));
    }
    const std::string name = "keyframe_" + std::to_string(k) + ".pcd";
    patch(name, patch_points);
    pose(poses, name, translation, rotation);
  }
  poses.close();
  EvidenceBuildStats stats;
  const auto voxels = SpatialEvidenceBuilder::build(directory_, ConfidenceParameters{}, &stats);
  const auto &v = voxels.at(VoxelKey{5, -2, 0});
  EXPECT_EQ(stats.source_keyframes, 7U);
  EXPECT_EQ(stats.input_points, 56U);
  EXPECT_EQ(stats.usable_points, 56U);
  EXPECT_EQ(stats.nonfinite_points, 0U);
  EXPECT_EQ(stats.voxel_count, voxels.size());
  EXPECT_EQ(v.point_count, 52U);
  EXPECT_EQ(v.observed_keyframes, 3U);
  EXPECT_EQ(v.first_keyframe, 1U);
  EXPECT_EQ(v.last_keyframe, 6U);
  EXPECT_EQ(v.keyframe_span, 5U);  // record index difference, not seconds
  EXPECT_NEAR(v.centroid.x(), world.x(), 1e-5);
  EXPECT_NEAR(v.centroid.y(), world.y(), 1e-5);
  EXPECT_NEAR(v.centroid.z(), world.z(), 1e-5);
  EXPECT_EQ(v.geometry_score, 1.0F);
  EXPECT_GT(v.auto_confidence, 0.60F);
  EXPECT_EQ(v.override_mode, ManualOverrideMode::AUTO);
  EXPECT_EQ(v.final_confidence, v.auto_confidence);
}

TEST_F(BuilderTest, RequiresMappingKeyframeInputs) {
  EXPECT_THROW(SpatialEvidenceBuilder::build(directory_, ConfidenceParameters{}), std::runtime_error);
  {
    std::ofstream poses(directory_ / "poses_timed.txt");
    pose(poses, "gone.pcd");
  }
  EXPECT_THROW(SpatialEvidenceBuilder::build(directory_, ConfidenceParameters{}), std::runtime_error);
}

TEST_F(BuilderTest, RejectsDuplicatePatchReferencesAndUnsafePaths) {
  patch("one.pcd", {Eigen::Vector3f(1, 1, 1)});
  {
    std::ofstream poses(directory_ / "poses_timed.txt");
    pose(poses, "one.pcd");
    pose(poses, "one.pcd");
  }
  EXPECT_THROW(SpatialEvidenceBuilder::build(directory_, ConfidenceParameters{}), std::runtime_error);
  {
    std::ofstream poses(directory_ / "poses_timed.txt");
    pose(poses, "../one.pcd");
  }
  EXPECT_THROW(SpatialEvidenceBuilder::build(directory_, ConfidenceParameters{}), std::runtime_error);
}

TEST_F(BuilderTest, RejectsInvalidPoseAndNonfiniteOnlyPatch) {
  patch("one.pcd", {Eigen::Vector3f(1, 1, 1)});
  {
    std::ofstream poses(directory_ / "poses_timed.txt");
    pose(poses, "one.pcd", Eigen::Vector3f::Zero(), Eigen::Quaternionf(0, 0, 0, 0));
  }
  EXPECT_THROW(SpatialEvidenceBuilder::build(directory_, ConfidenceParameters{}), std::runtime_error);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  patch("nan.pcd", {Eigen::Vector3f(nan, 0, 0)});
  {
    std::ofstream poses(directory_ / "poses_timed.txt");
    pose(poses, "nan.pcd");
  }
  EXPECT_THROW(SpatialEvidenceBuilder::build(directory_, ConfidenceParameters{}), std::runtime_error);
}
}  // namespace
