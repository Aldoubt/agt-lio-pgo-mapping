#include "agt_spatial_map_core/geometry_evidence.hpp"
#include "agt_spatial_map_core/spatial_evidence_builder.hpp"

#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <unordered_map>
#include <vector>
#include <unistd.h>

namespace fs = std::filesystem;
using namespace agt_spatial_map_core;

namespace {
constexpr float kVoxel = .2F;
using Obs = std::vector<GeometryObservation>;

SpatialEvidenceMap source_for(const Obs &observations, float size = kVoxel) {
  struct Sum { Eigen::Vector3d xyz = Eigen::Vector3d::Zero(); std::uint32_t n = 0; };
  std::unordered_map<VoxelKey, Sum, VoxelKeyHash> sums;
  for (const auto &o : observations) {
    auto &s = sums[voxel_for(o.position_map, size)];
    s.xyz += o.position_map.cast<double>(); ++s.n;
  }
  SpatialEvidenceMap result;
  for (const auto &[key, s] : sums) {
    SpatialVoxelEvidence v;
    v.key = key;
    v.centroid = (s.xyz / s.n).cast<float>();
    v.point_count = s.n;
    result.emplace(key, v);
  }
  return result;
}

Obs plane(int axis, const Eigen::Vector3f &origin = Eigen::Vector3f(0.5F, -0.3F, 1.3F)) {
  Obs obs;
  for (int a = -8; a <= 8; ++a) {
    for (int b = -8; b <= 8; ++b) {
      Eigen::Vector3f p(.05F, .05F, .05F);
      p[(axis + 1) % 3] = (a + .5F) * kVoxel;
      p[(axis + 2) % 3] = (b + .5F) * kVoxel;
      obs.push_back({p, origin});
    }
  }
  return obs;
}

GeometryVoxelEvidence selected(const GeometryEvidence &result, VoxelKey key) {
  for (const auto &v : result.voxels) if (v.key == key) return v;
  throw std::runtime_error("missing synthetic voxel");
}

TEST(GeometryEstimator, GroundAndWallHaveComplementaryWeakDirections) {
  const auto ground = plane(2);
  const auto g = GeometryEvidenceEstimator::estimate(source_for(ground), ground, kVoxel,
                                                      GeometryParameters{});
  const auto v = selected(g, voxel_for(Eigen::Vector3f(.05F, .05F, .05F), kVoxel));
  ASSERT_TRUE(v.normal_valid);
  ASSERT_TRUE(v.translation.valid);
  ASSERT_TRUE(v.rotation.valid);
  EXPECT_GT(std::abs(v.normal.z()), .97F);
  EXPECT_GT(v.planarity, .8F);
  EXPECT_LT(v.linearity, .2F);
  EXPECT_NEAR(v.translation.eigenvalues.x(), 0.0, .0001);
  EXPECT_NEAR(v.translation.isotropy, 0.0, .0001);
  EXPECT_LT(std::abs(v.translation.weak_direction.z()), .15F);
  EXPECT_LT(v.rotation.isotropy, .001F);
  EXPECT_GT(std::abs(v.rotation.weak_direction.z()), .95F); // yaw weak on ground
  EXPECT_GT(v.normal_support_points, 8U);
  EXPECT_GT(v.valid_normal_voxels, 6U);

  const auto wall = plane(0);
  const auto w = GeometryEvidenceEstimator::estimate(source_for(wall), wall, kVoxel,
                                                      GeometryParameters{});
  const auto u = selected(w, voxel_for(Eigen::Vector3f(.05F, .05F, .05F), kVoxel));
  ASSERT_TRUE(u.normal_valid);
  ASSERT_TRUE(u.rotation.valid);
  EXPECT_GT(std::abs(u.normal.x()), .95F);
  EXPECT_LT(std::abs(u.translation.weak_direction.x()), .15F);
  EXPECT_GT(std::abs(u.rotation.weak_direction.x()), .95F);
}

TEST(GeometryEstimator, CornerHasMoreDirectionalDiversityThanSinglePlane) {
  const auto ground = plane(2);
  const auto base = GeometryEvidenceEstimator::estimate(source_for(ground), ground, kVoxel,
                                                         GeometryParameters{});
  Obs corner = ground;
  for (auto axis : {0, 1}) {
    auto wall = plane(axis);
    corner.insert(corner.end(), wall.begin(), wall.end());
  }
  const auto result = GeometryEvidenceEstimator::estimate(source_for(corner), corner, kVoxel,
                                                           GeometryParameters{});
  const auto key = voxel_for(Eigen::Vector3f(.05F, .05F, .05F), kVoxel);
  const auto g = selected(result, key), b = selected(base, key);
  ASSERT_TRUE(g.translation.valid);
  ASSERT_TRUE(g.rotation.valid);
  EXPECT_GT(g.valid_normal_voxels, b.valid_normal_voxels);
  EXPECT_GT(g.translation.isotropy, b.translation.isotropy + .08F);
  EXPECT_GT(g.rotation.isotropy, b.rotation.isotropy + .02F);
  EXPECT_LE(g.translation.isotropy, 1.0F);
  EXPECT_LE(g.rotation.isotropy, 1.0F);
}

TEST(GeometryEstimator, CylinderPoleHasWeakVerticalTranslation) {
  Obs pole;
  constexpr float radius = .42F;
  for (int k = 0; k < 24; ++k) {
    const float angle = static_cast<float>(k * 2.0 * 3.141592653589793 / 24);
    for (int h = -8; h <= 8; ++h) {
      pole.push_back({Eigen::Vector3f(radius * std::cos(angle),
                                      radius * std::sin(angle), h * .12F + .05F),
                      Eigen::Vector3f(0, 0, 1.5F)});
    }
  }
  const auto result = GeometryEvidenceEstimator::estimate(source_for(pole), pole, kVoxel,
                                                           GeometryParameters{});
  const auto p = selected(result, voxel_for(pole.front().position_map, kVoxel));
  EXPECT_TRUE(p.normal_valid);
  ASSERT_TRUE(p.translation.valid);
  EXPECT_GT(std::abs(p.translation.weak_direction.z()), .75F);
  EXPECT_LT(p.translation.isotropy, .15F); // pole geometry does not resolve z translation
}

TEST(GeometryEstimator, SparseAndCollinearNeighborhoodNeverFabricateQuality) {
  Obs sparse;
  for (int i = 0; i != 4; ++i) {
    sparse.push_back({Eigen::Vector3f(i * 2.0F + .05F, .05F, .05F),
                      Eigen::Vector3f::Zero()});
  }
  const auto result = GeometryEvidenceEstimator::estimate(source_for(sparse), sparse,
      kVoxel, GeometryParameters{});
  for (const auto &v : result.voxels) {
    EXPECT_FALSE(v.normal_valid);
    EXPECT_FALSE(v.translation.valid);
    EXPECT_FALSE(v.rotation.valid);
    EXPECT_EQ(v.valid_normal_voxels, 0U);
    EXPECT_TRUE(std::isnan(v.normal.x()));
    EXPECT_TRUE(std::isnan(v.translation.isotropy));
    EXPECT_TRUE(std::isnan(v.rotation.condition));
  }
  Obs line;
  for (int i = 0; i < 20; ++i) {
    line.push_back({Eigen::Vector3f((i - 10) * .015F + .05F, .05F, .05F),
                    Eigen::Vector3f::Zero()});
  }
  const auto degenerate = GeometryEvidenceEstimator::estimate(source_for(line), line,
      kVoxel, GeometryParameters{});
  for (const auto &v : degenerate.voxels) {
    EXPECT_FALSE(v.normal_valid) << "line PCA has no unique surface normal";
  }
}

TEST(GeometryEstimator, HtNormalVotesAreNotWeightedByRawPointDensity) {
  Obs mixed = plane(2);
  auto wall = plane(0);
  for (auto &o : wall) o.position_map.x() = .85F;
  mixed.insert(mixed.end(), wall.begin(), wall.end());
  GeometryParameters params;
  params.normal_radius = .32F; // keep far wall normals disjoint from ground
  params.observability_radius = 1.0F;
  const auto baseline = GeometryEvidenceEstimator::estimate(source_for(mixed), mixed,
      kVoxel, params);
  const auto key = voxel_for(Eigen::Vector3f(.5F, .1F, .05F), kVoxel);
  const auto first = selected(baseline, key);
  ASSERT_TRUE(first.translation.valid);
  // Repeated raw observations of ONE wall voxel must not add 30 votes to Ht.
  // The center, surface normal and target voxel key are exactly unchanged.
  const GeometryObservation extra{
      Eigen::Vector3f(.85F, .1F, .7F), Eigen::Vector3f(.5F, -.3F, 1.3F)};
  for (int repeat = 0; repeat < 30; ++repeat) mixed.push_back(extra);
  const auto dense = GeometryEvidenceEstimator::estimate(source_for(mixed), mixed,
      kVoxel, params);
  const auto second = selected(dense, key);
  ASSERT_TRUE(second.translation.valid);
  EXPECT_EQ(first.valid_normal_voxels, second.valid_normal_voxels);
  EXPECT_EQ(first.supporting_observations + 30U, second.supporting_observations);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(first.translation.eigenvalues[i], second.translation.eigenvalues[i], .001);
  }
  EXPECT_NEAR(first.translation.isotropy, second.translation.isotropy, .001);
}

TEST(GeometryEstimator, UsesEachObservationBodyOriginRatherThanWorldOrigin) {
  const auto ground = plane(2);
  const auto standard = GeometryEvidenceEstimator::estimate(source_for(ground), ground,
      kVoxel, GeometryParameters{});
  auto colocated = ground;
  for (auto &v : colocated) v.body_origin_map = v.position_map;
  const auto zero = GeometryEvidenceEstimator::estimate(source_for(ground), colocated,
      kVoxel, GeometryParameters{});
  const auto key = voxel_for(Eigen::Vector3f(.05F, .05F, .05F), kVoxel);
  const auto a = selected(standard, key), b = selected(zero, key);
  EXPECT_EQ(a.translation.valid, b.translation.valid);
  EXPECT_NEAR(a.translation.eigenvalues.z(), b.translation.eigenvalues.z(), .0001);
  EXPECT_TRUE(a.rotation.valid);
  EXPECT_FALSE(b.rotation.valid); // every g=(p-t_body)xn = 0
  EXPECT_TRUE(std::isnan(b.rotation.isotropy));
}

TEST(GeometryEstimator, RotationTranslationAndPointOrderAreCovariant) {
  Obs corner = plane(2);
  for (auto axis : {0, 1}) {
    const auto wall = plane(axis);
    corner.insert(corner.end(), wall.begin(), wall.end());
  }
  GeometryParameters params;
  // Avoid support lying on the radius boundary; a rasterized VoxelKey grid
  // is equivariant under grid-aligned translations and axis rotations.
  params.normal_radius = .38F;
  params.observability_radius = .78F;
  const auto confidence = source_for(corner);
  const auto original = GeometryEvidenceEstimator::estimate(confidence, corner,
      kVoxel, params);
  auto shuffled = corner;
  std::mt19937 rng(20260927);
  std::shuffle(shuffled.begin(), shuffled.end(), rng);
  const auto shuffled_result = GeometryEvidenceEstimator::estimate(confidence, shuffled,
      kVoxel, params);
  const Eigen::Matrix3f R = Eigen::AngleAxisf(static_cast<float>(3.141592653589793 / 2),
                                               Eigen::Vector3f::UnitZ()).toRotationMatrix();
  const Eigen::Vector3f t(3.2F, -2.8F, 1.2F);
  auto transformed = corner;
  for (auto &o : transformed) {
    o.position_map = R * o.position_map + t;
    o.body_origin_map = R * o.body_origin_map + t;
  }
  const auto rotated = GeometryEvidenceEstimator::estimate(source_for(transformed), transformed,
      kVoxel, params);
  const auto p = Eigen::Vector3f(.05F, .05F, .05F);
  const auto a = selected(original, voxel_for(p, kVoxel));
  const auto b = selected(shuffled_result, voxel_for(p, kVoxel));
  const auto c = selected(rotated, voxel_for(R * p + t, kVoxel));
  ASSERT_TRUE(a.translation.valid && a.rotation.valid);
  ASSERT_TRUE(b.translation.valid && b.rotation.valid);
  ASSERT_TRUE(c.translation.valid && c.rotation.valid);
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(a.translation.eigenvalues[i], b.translation.eigenvalues[i], .005);
    EXPECT_NEAR(a.rotation.eigenvalues[i], b.rotation.eigenvalues[i], .005);
    EXPECT_NEAR(a.translation.eigenvalues[i], c.translation.eigenvalues[i], .5);
    EXPECT_NEAR(a.rotation.eigenvalues[i], c.rotation.eigenvalues[i], .5);
  }
  EXPECT_NEAR(a.translation.isotropy, b.translation.isotropy, .0005);
  EXPECT_NEAR(a.rotation.isotropy, b.rotation.isotropy, .0005);
  EXPECT_NEAR(a.translation.isotropy, c.translation.isotropy, .025);
  EXPECT_NEAR(a.rotation.isotropy, c.rotation.isotropy, .025);
}

TEST(GeometryEstimator, RejectsMissingOrIncorrectImmutableV1Observations) {
  const auto ground = plane(2);
  auto source = source_for(ground);
  auto missing = ground;
  missing.pop_back();
  EXPECT_THROW(GeometryEvidenceEstimator::estimate(source, missing, kVoxel,
      GeometryParameters{}), std::invalid_argument);
  source.begin()->second.centroid.x() += .002F;
  EXPECT_THROW(GeometryEvidenceEstimator::estimate(source, ground, kVoxel,
      GeometryParameters{}), std::invalid_argument);
  GeometryParameters p;
  p.normal_epsilon_m2 = 0;
  EXPECT_THROW(GeometryEvidenceEstimator::estimate(source, ground, kVoxel, p),
               std::invalid_argument);
}

TEST(GeometryEstimator, PatchBuilderUsesNormalizedTMapBodyAndV1VoxelFloor) {
  const fs::path root = fs::temp_directory_path() /
      ("agt_geometry_pose_" + std::to_string(getpid()));
  fs::create_directories(root / "patches");
  const Eigen::Quaternionf q(Eigen::AngleAxisf(.45F, Eigen::Vector3f::UnitZ()));
  const Eigen::Vector3f t(1.1F, -.7F, .6F);
  auto world = plane(2);
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (const auto &o : world) {
    const auto body = q.conjugate() * (o.position_map - t);
    pcl::PointXYZI p;
    p.x = body.x(); p.y = body.y(); p.z = body.z(); p.intensity = 1.0F;
    cloud.push_back(p);
  }
  ASSERT_EQ(pcl::io::savePCDFileBinary((root / "patches" / "one.pcd").string(), cloud), 0);
  {
    std::ofstream poses(root / "poses_timed.txt");
    poses << std::setprecision(12) << "one.pcd 123 " << t.x() << ' ' << t.y() << ' ' << t.z()
          << ' ' << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << '\n';
  }
  EvidenceBuildStats stats;
  auto source = SpatialEvidenceBuilder::build(root, ConfidenceParameters{}, &stats);
  const auto result = GeometryEvidenceEstimator::build(root, source, kVoxel,
      GeometryParameters{}, stats.source_keyframes, stats.usable_points);
  EXPECT_EQ(result.source_keyframes, 1U);
  EXPECT_EQ(result.source_observations, cloud.size());
  EXPECT_EQ(result.voxels.size(), source.size());
  EXPECT_GT(std::count_if(result.voxels.begin(), result.voxels.end(),
                          [](const auto &v) { return v.normal_valid; }), 5);
  EXPECT_THROW(GeometryEvidenceEstimator::build(root, source, kVoxel,
      GeometryParameters{}, 2, stats.usable_points), std::invalid_argument);
  fs::remove_all(root);
}

}  // namespace
