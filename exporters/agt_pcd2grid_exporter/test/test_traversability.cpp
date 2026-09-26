#include "agt_pcd2grid_exporter/OccupancyGridWriter.hpp"
#include "agt_pcd2grid_exporter/PCDProjector.hpp"
#include "agt_pcd2grid_exporter/ParameterLoader.hpp"
#include "agt_pcd2grid_exporter/TraversabilityGridBuilder.hpp"

#include <gtest/gtest.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace agt_pcd2grid_exporter {
namespace {

struct Pose {
  float x;
  float y;
  float z;
  float yaw;
};

using Points = std::vector<Eigen::Vector3f>;

// Writes a keyframe package: world points are observed by every keyframe
// (converted into each body frame); body_points are appended per keyframe.
std::filesystem::path write_package(const std::string &name, const std::vector<Pose> &poses,
                                    const Points &world_points, const Points &body_points = {},
                                    const std::vector<Points> &per_keyframe_world = {}) {
  const auto root = std::filesystem::temp_directory_path() / name;
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root / "patches");
  std::ofstream poses_file(root / "poses_timed.txt");
  for (std::size_t k = 0; k < poses.size(); ++k) {
    Eigen::Isometry3f map_from_body = Eigen::Isometry3f::Identity();
    map_from_body.linear() =
        Eigen::AngleAxisf(poses[k].yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    map_from_body.translation() = Eigen::Vector3f(poses[k].x, poses[k].y, poses[k].z);
    const Eigen::Isometry3f body_from_map = map_from_body.inverse();
    pcl::PointCloud<pcl::PointXYZI> patch;
    const auto add = [&patch](const Eigen::Vector3f &body) {
      pcl::PointXYZI point;
      point.x = body.x();
      point.y = body.y();
      point.z = body.z();
      point.intensity = 1.0F;
      patch.push_back(point);
    };
    for (const auto &world : world_points) add(body_from_map * world);
    if (k < per_keyframe_world.size()) {
      for (const auto &world : per_keyframe_world[k]) add(body_from_map * world);
    }
    for (const auto &body : body_points) add(body);
    EXPECT_EQ(pcl::io::savePCDFileBinary((root / "patches" / (std::to_string(k) + ".pcd")).string(),
                                         patch),
              0);
    const Eigen::Quaternionf q(map_from_body.linear());
    poses_file << k << ".pcd " << 100.0 + static_cast<double>(k) << ' ' << poses[k].x << ' '
               << poses[k].y << ' ' << poses[k].z << ' ' << q.w() << ' ' << q.x() << ' ' << q.y()
               << ' ' << q.z() << '\n';
  }
  return root;
}

ProjectionParameters test_parameters() {
  ProjectionParameters parameters;
  parameters.projection_mode = ProjectionMode::Traversability;
  parameters.resolution = 0.1F;
  parameters.temporal_filter_enabled = false;
  parameters.occupied_threshold = 2U;
  parameters.min_component_cells = 1U;
  parameters.closing_radius_cells = 0U;
  parameters.robot.base_from_body_xyz = {{0.0F, 0.0F, 1.0F}};
  parameters.robot.base_from_body_rpy = {{0.0F, 0.0F, 0.0F}};
  parameters.robot.footprint = {{0.3F, 0.2F}, {0.3F, -0.2F}, {-0.3F, -0.2F}, {-0.3F, 0.2F}};
  parameters.traversability.hole_fill_max_area = 0.0F;
  parameters.traversability.obstacle_min_observation_range = 0.0F;
  return parameters;
}

int value_at(const OccupancyGrid &grid, float x, float y) {
  const auto gx = static_cast<long long>(std::floor((x - grid.origin_x) / grid.resolution));
  const auto gy = static_cast<long long>(std::floor((y - grid.origin_y) / grid.resolution));
  if (gx < 0 || gy < 0 || gx >= grid.width || gy >= grid.height) return -2;
  return grid.occupancy[static_cast<std::size_t>(gy) * grid.width + static_cast<std::size_t>(gx)];
}

const std::vector<Pose> kThreePoses = {{0.0F, 0.0F, 1.0F, 0.0F},
                                       {0.5F, 0.0F, 1.0F, 0.0F},
                                       {1.0F, 0.0F, 1.0F, 0.0F}};

TEST(TraversabilityGridBuilderTest, RaysCarveFreeSpaceBetweenSparseGroundReturns) {
  Points world;
  for (int i = 0; i < 7; ++i) world.emplace_back(0.85F + 0.8F * i, 0.05F, 0.0F);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 5; ++j) world.emplace_back(6.05F + 0.2F * i, -0.35F + 0.2F * j, 0.0F);
  }
  // overhead structure (above obstacle_height.max) must not become an obstacle
  world.emplace_back(3.05F, 0.05F, 2.5F);
  world.emplace_back(3.05F, 0.05F, 2.6F);
  // isolated obstacle far to the side (also anchors the grid bounds)
  world.emplace_back(6.05F, 3.55F, 0.5F);
  world.emplace_back(6.05F, 3.55F, 0.65F);
  const auto root = write_package("agt_trav_carve", kThreePoses, world);
  OccupancyGrid grid;
  ProjectionStats stats;
  TraversabilityStats traversability;
  std::string error;
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), test_parameters(), &grid, &stats,
                                               &traversability, &error))
      << error;
  EXPECT_EQ(value_at(grid, 2.85F, 0.05F), 0) << "no ground return here, only carving rays";
  EXPECT_EQ(value_at(grid, 3.05F, 0.05F), 0) << "overhead points are not obstacles";
  EXPECT_EQ(value_at(grid, 3.05F, 3.05F), -1) << "never observed";
  EXPECT_EQ(value_at(grid, 6.05F, 3.55F), 100);
  EXPECT_GT(traversability.carved_cells, 0U);
  EXPECT_GT(traversability.rays_cast, 0U);
  std::filesystem::remove_all(root);
}

TEST(TraversabilityGridBuilderTest, WallIsOccupiedAndSpaceBehindItStaysUnknown) {
  Points world;
  for (int i = 0; i < 8; ++i) {
    for (int j = 0; j < 3; ++j) world.emplace_back(0.85F + 0.4F * i, -0.35F + 0.4F * j, 0.0F);
  }
  for (int j = 0; j < 20; ++j) {
    for (int k = 0; k < 4; ++k) world.emplace_back(4.05F, -0.95F + 0.1F * j, 0.3F + 0.3F * k);
  }
  world.emplace_back(4.55F, 3.05F, 0.5F);
  world.emplace_back(4.55F, 3.05F, 0.7F);
  const auto root = write_package("agt_trav_wall", kThreePoses, world);
  OccupancyGrid grid;
  ProjectionStats stats;
  TraversabilityStats traversability;
  std::string error;
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), test_parameters(), &grid, &stats,
                                               &traversability, &error))
      << error;
  EXPECT_EQ(value_at(grid, 4.05F, 0.05F), 100);
  EXPECT_EQ(value_at(grid, 2.05F, 0.05F), 0);
  EXPECT_EQ(value_at(grid, 4.55F, 0.05F), -1);
  std::filesystem::remove_all(root);
}

TEST(TraversabilityGridBuilderTest, SelfReturnsAreDroppedAndSweptTransientsCleared) {
  std::vector<Pose> poses;
  for (int k = 0; k <= 5; ++k) poses.push_back({0.5F * static_cast<float>(k), 0.0F, 1.0F, 0.0F});
  Points world;
  for (int i = 0; i < 9; ++i) {
    for (int j = 0; j < 3; ++j) world.emplace_back(0.85F + 0.4F * i, -0.35F + 0.4F * j, 0.0F);
  }
  // camera mast behind the sensor, rigidly attached to the robot
  const Points mast = {{-0.2F, 0.0F, 0.3F}, {-0.2F, 0.0F, 0.5F}, {-0.2F, 0.0F, 0.7F}};
  // a person standing on the future path, seen only before the robot passes
  const Points person = {{1.85F, 0.05F, 0.4F}, {1.85F, 0.05F, 0.6F}};
  const std::vector<Points> per_keyframe = {person, person, person};
  const auto root = write_package("agt_trav_self", poses, world, mast, per_keyframe);
  OccupancyGrid grid;
  ProjectionStats stats;
  TraversabilityStats traversability;
  std::string error;
  ProjectionParameters parameters = test_parameters();
  parameters.traversability.self_filter_min_height = 0.1F;  // keep synthetic floor points
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), parameters, &grid, &stats,
                                               &traversability, &error))
      << error;
  EXPECT_EQ(traversability.self_filtered_points, 18U);
  EXPECT_EQ(value_at(grid, 1.85F, 0.05F), 0);
  EXPECT_GE(traversability.sweep_cleared_occupied_cells, 1U);
  for (int i = 0; i < 25; ++i) {
    const float x = 0.05F + 0.1F * i;
    EXPECT_NE(value_at(grid, x, 0.05F), 100) << "occupied cell on the driven path at x=" << x;
  }
  ProjectionParameters no_sweep = parameters;
  no_sweep.traversability.sweep_enabled = false;
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), no_sweep, &grid, &stats,
                                               &traversability, &error))
      << error;
  EXPECT_EQ(value_at(grid, 1.85F, 0.05F), 100) << "hits win over carving rays without sweep";
  std::filesystem::remove_all(root);
}

TEST(TraversabilityGridBuilderTest, GroundIsNotGrownAcrossADropOff) {
  Points world;
  for (int i = 0; i < 11; ++i) {
    for (int j = 0; j < 5; ++j) world.emplace_back(0.55F + 0.25F * i, -0.45F + 0.25F * j, 0.0F);
  }
  for (int i = 0; i < 11; ++i) {
    for (int j = 0; j < 5; ++j) world.emplace_back(3.55F + 0.25F * i, -0.45F + 0.25F * j, -0.6F);
  }
  const auto root = write_package("agt_trav_drop", kThreePoses, world);
  OccupancyGrid grid;
  ProjectionStats stats;
  TraversabilityStats traversability;
  std::string error;
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), test_parameters(), &grid, &stats,
                                               &traversability, &error))
      << error;
  EXPECT_EQ(value_at(grid, 2.05F, 0.05F), 0);
  EXPECT_EQ(value_at(grid, 3.35F, 0.05F), -1) << "no flat extrapolation at the drop edge";
  EXPECT_EQ(value_at(grid, 5.05F, 0.05F), -1) << "lower level is not reachable ground";
  EXPECT_GT(traversability.ground_rejected_cells, 0U);
  std::filesystem::remove_all(root);
}

TEST(TraversabilityGridBuilderTest, PorousVegetationIsNotCarvedFree) {
  Points world;
  for (int i = 0; i < 12; ++i) {
    for (int j = 0; j < 5; ++j) world.emplace_back(0.85F + 0.4F * i, -0.35F + 0.2F * j, 0.0F);
  }
  // a bush: every 0.1 m cell holds exactly one return, seen by one keyframe only,
  // while rays towards the floor behind it slip through the gaps
  std::vector<Points> per_keyframe(3);
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      per_keyframe[static_cast<std::size_t>((i + j) % 3)].emplace_back(
          2.55F + 0.1F * i, -0.15F + 0.1F * j, 0.5F);
    }
  }
  const auto root = write_package("agt_trav_bush", kThreePoses, world, {}, per_keyframe);
  OccupancyGrid grid;
  ProjectionStats stats;
  TraversabilityStats traversability;
  std::string error;
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), test_parameters(), &grid, &stats,
                                               &traversability, &error))
      << error;
  for (int i = 0; i < 5; ++i) {
    for (int j = 0; j < 5; ++j) {
      EXPECT_EQ(value_at(grid, 2.55F + 0.1F * i, -0.15F + 0.1F * j), 100)
          << "bush cell " << i << "," << j;
    }
  }
  EXPECT_GT(traversability.support_occupied_cells, 0U);
  ProjectionParameters legacy = test_parameters();
  legacy.traversability.obstacle_support_radius_cells = 0U;
  legacy.traversability.free_requires_no_hits = false;
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), legacy, &grid, &stats,
                                               &traversability, &error))
      << error;
  EXPECT_EQ(value_at(grid, 2.75F, 0.05F), 0) << "failure mode without support/veto";
  std::filesystem::remove_all(root);
}

TEST(TraversabilityGridBuilderTest, WriterRecordsTraversabilityMetadata) {
  Points world;
  for (int i = 0; i < 8; ++i) world.emplace_back(0.85F + 0.4F * i, 0.05F, 0.0F);
  world.emplace_back(3.55F, 1.05F, 0.5F);
  world.emplace_back(3.55F, 1.05F, 0.7F);
  const auto root = write_package("agt_trav_writer", kThreePoses, world);
  ProjectionParameters parameters = test_parameters();
  OccupancyGrid grid;
  ProjectionStats stats;
  TraversabilityStats traversability;
  std::string error;
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), parameters, &grid, &stats,
                                               &traversability, &error))
      << error;
  const auto output = std::filesystem::temp_directory_path() / "agt_trav_writer_out";
  std::filesystem::remove_all(output);
  ASSERT_TRUE(OccupancyGridWriter::write_navigation_map(grid, parameters, stats, output.string(),
                                                        root.string(), &error, &traversability))
      << error;
  std::ifstream metadata(output / "metadata.yaml");
  const std::string text((std::istreambuf_iterator<char>(metadata)), {});
  EXPECT_NE(text.find("traversability:"), std::string::npos);
  EXPECT_NE(text.find("sweep_cleared_occupied_cells"), std::string::npos);
  ProjectionParameters loaded;
  ASSERT_TRUE(ParameterLoader::load((output / "projection.yaml").string(), &loaded, &error))
      << error;
  EXPECT_EQ(loaded.projection_mode, ProjectionMode::Traversability);
  EXPECT_FLOAT_EQ(loaded.robot.base_from_body_xyz[2], 1.0F);
  ASSERT_EQ(loaded.robot.footprint.size(), 4U);
  EXPECT_FLOAT_EQ(loaded.robot.footprint[0].first, 0.3F);
  EXPECT_EQ(loaded.traversability.free_min_passes, parameters.traversability.free_min_passes);
  EXPECT_FALSE(loaded.temporal_filter_enabled);
  std::filesystem::remove_all(output);
  std::filesystem::remove_all(root);
}

TEST(TraversabilityGridBuilderTest, FollowerSeenOnlyUpCloseIsNotAnObstacle) {
  std::vector<Pose> poses;
  for (int k = 0; k <= 6; ++k) poses.push_back({0.5F * static_cast<float>(k), 0.0F, 1.0F, 0.0F});
  Points world;
  for (int i = 0; i < 14; ++i) {
    for (int j = 0; j < 5; ++j) world.emplace_back(-1.95F + 0.4F * i, -0.95F + 0.4F * j, 0.0F);
  }
  // a static post seen from every keyframe (from far and near)
  world.emplace_back(2.05F, 1.05F, 0.5F);
  world.emplace_back(2.05F, 1.05F, 0.8F);
  // an operator walking 1 m behind and 0.3 m left of the robot
  std::vector<Points> follower(poses.size());
  for (std::size_t k = 0; k < poses.size(); ++k) {
    follower[k] = {{poses[k].x - 0.95F, 0.35F, 1.2F}, {poses[k].x - 0.95F, 0.35F, 1.3F}};
  }
  const auto root = write_package("agt_trav_follower", poses, world, {}, follower);
  ProjectionParameters parameters = test_parameters();
  parameters.traversability.obstacle_min_observation_range = 2.0F;
  OccupancyGrid grid;
  ProjectionStats stats;
  TraversabilityStats traversability;
  std::string error;
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), parameters, &grid, &stats,
                                               &traversability, &error))
      << error;
  EXPECT_EQ(value_at(grid, 2.05F, 1.05F), 100) << "static post must stay";
  EXPECT_NE(value_at(grid, -0.95F, 0.35F), 100) << "follower trail must not become an obstacle";
  EXPECT_GT(traversability.near_field_only_points, 0U);
  parameters.traversability.obstacle_min_observation_range = 0.0F;
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), parameters, &grid, &stats,
                                               &traversability, &error))
      << error;
  EXPECT_EQ(value_at(grid, -0.95F, 0.35F), 100) << "without the rule the trail is baked in";
  std::filesystem::remove_all(root);
}

TEST(TraversabilityGridBuilderTest, RejectsDivergedPoses) {
  const Points world = {{2.05F, 0.05F, 0.0F}, {2.45F, 0.05F, 0.0F}};
  const auto root = write_package("agt_trav_diverged", kThreePoses, world);
  // replace the last pose by a 90 deg roll, i.e. a diverged LIO state
  {
    std::ifstream in(root / "poses_timed.txt");
    std::vector<std::string> lines;
    for (std::string line; std::getline(in, line);) lines.push_back(line);
    lines.back() = "2.pcd 102 1.0 0.0 1.0 0.70710678 0.70710678 0 0";
    std::ofstream out(root / "poses_timed.txt");
    for (const auto &line : lines) out << line << '\n';
  }
  OccupancyGrid grid;
  ProjectionStats stats;
  TraversabilityStats traversability;
  std::string error;
  EXPECT_FALSE(TraversabilityGridBuilder::build(root.string(), test_parameters(), &grid, &stats,
                                                &traversability, &error));
  EXPECT_NE(error.find("diverged"), std::string::npos) << error;
  std::filesystem::remove_all(root);
}

TEST(TraversabilityParameterTest, PointCloudProjectorRejectsTraversabilityMode) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.push_back(pcl::PointXYZ(1.0F, 0.0F, 0.0F));
  pcl::PCLPointCloud2 blob;
  pcl::toPCLPointCloud2(cloud, blob);
  OccupancyGrid grid;
  ProjectionStats stats;
  std::string error;
  EXPECT_FALSE(PCDProjector::project(blob, test_parameters(), &grid, &stats, &error));
  EXPECT_NE(error.find("--package"), std::string::npos) << error;
}

TEST(TraversabilityParameterTest, RejectsInvalidFootprint) {
  const auto path = std::filesystem::temp_directory_path() / "agt_trav_bad.yaml";
  {
    std::ofstream stream(path);
    stream << "projection_mode: traversability\nrobot:\n  footprint: [[0.5, 0.4], [0.5, -0.4]]\n";
  }
  ProjectionParameters parameters;
  std::string error;
  EXPECT_FALSE(ParameterLoader::load(path.string(), &parameters, &error));
  EXPECT_NE(error.find("footprint"), std::string::npos);
  std::filesystem::remove(path);
}

}  // namespace
}  // namespace agt_pcd2grid_exporter
