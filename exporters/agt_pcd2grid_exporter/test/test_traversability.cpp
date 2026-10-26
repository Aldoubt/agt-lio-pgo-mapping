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
#include <sys/wait.h>
#include <unistd.h>

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
  std::ofstream plain_poses(root / "poses.txt");
  pcl::PointCloud<pcl::PointXYZI> map;
  for (std::size_t k = 0; k < poses.size(); ++k) {
    Eigen::Isometry3f map_from_body = Eigen::Isometry3f::Identity();
    map_from_body.linear() =
        Eigen::AngleAxisf(poses[k].yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
    map_from_body.translation() = Eigen::Vector3f(poses[k].x, poses[k].y, poses[k].z);
    const Eigen::Isometry3f body_from_map = map_from_body.inverse();
    pcl::PointCloud<pcl::PointXYZI> patch;
    const auto add = [&patch, &map, &map_from_body](const Eigen::Vector3f &body) {
      pcl::PointXYZI point;
      point.x = body.x();
      point.y = body.y();
      point.z = body.z();
      point.intensity = 1.0F;
      patch.push_back(point);
      const auto world = map_from_body * body;
      point.x = world.x(); point.y = world.y(); point.z = world.z();
      map.push_back(point);
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
    plain_poses << k << ".pcd " << poses[k].x << ' ' << poses[k].y << ' ' << poses[k].z
                << ' ' << q.w() << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << '\n';
  }
  if (!map.empty()) EXPECT_EQ(pcl::io::savePCDFileBinary((root / "map.pcd").string(), map), 0);
  std::ofstream(root / "calibration.yaml") << "status: synthetic_test_fixture\n";
  std::ofstream(root / "metadata.yaml") << "fixture: true\n";
  return root;
}

// Test data uses the actual existing source integrity writer; it is not a
// replacement production validator and does not claim a measured robot profile.
void complete_source_integrity(const std::filesystem::path &root) {
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    execlp("python3", "python3", "-c",
           "from pathlib import Path; import sys; "
           "from agt_mapping_artifacts.map_package_exporter import _write_integrity_files; "
           "_write_integrity_files(Path(sys.argv[1]))", root.c_str(), nullptr);
    _exit(127);
  }
  int status;
  ASSERT_EQ(waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  ASSERT_EQ(WEXITSTATUS(status), 0);
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
  complete_source_integrity(root);
  ProjectionParameters parameters = test_parameters();
  parameters.traversability.obstacle_min_observation_range = 2.0F;
  OccupancyGrid grid;
  ProjectionStats stats;
  TraversabilityStats traversability;
  std::string error;
  const auto clouds = root.parent_path() / "agt_trav_follower_cleaned";
  std::filesystem::remove_all(clouds);
  OccupancyGrid baseline;
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), parameters, &baseline, &stats,
                                               &traversability, &error)) << error;
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), parameters, &grid, &stats,
                                               &traversability, &error, "", clouds.string()))
      << error;
  EXPECT_EQ(grid.occupancy, baseline.occupancy);
  EXPECT_EQ(grid.hit_count, baseline.hit_count);
  EXPECT_EQ(grid.width, baseline.width);
  EXPECT_EQ(grid.height, baseline.height);
  EXPECT_EQ(grid.origin_x, baseline.origin_x);
  EXPECT_EQ(grid.origin_y, baseline.origin_y);
  EXPECT_TRUE(std::filesystem::exists(clouds / "manifest.yaml"));
  EXPECT_TRUE(std::filesystem::exists(clouds / "map.yaml"));
  EXPECT_EQ(value_at(grid, 2.05F, 1.05F), 100) << "static post must stay";
  EXPECT_NE(value_at(grid, -0.95F, 0.35F), 100) << "follower trail must not become an obstacle";
  EXPECT_GT(traversability.near_field_only_points, 0U);
  pcl::PointCloud<pcl::PointXYZ> kept, removed;
  ASSERT_EQ(pcl::io::loadPCDFile((clouds / "static_map.pcd").string(), kept), 0);
  ASSERT_EQ(pcl::io::loadPCDFile((clouds / "removed_candidates.pcd").string(), removed), 0);
  EXPECT_EQ(removed.size(), 2U * poses.size());
  EXPECT_EQ(kept.size() + removed.size(), stats.input_points - traversability.self_filtered_points);
  bool have_post = false, have_ground = false;
  for (const auto &p : kept) {
    have_post |= std::abs(p.x - 2.05F) < 0.01F && p.z > 0.4F;
    have_ground |= std::abs(p.z) < 0.01F;
    EXPECT_FALSE(std::abs(p.y - 0.35F) < 0.01F && p.z > 1.1F);
  }
  EXPECT_TRUE(have_post);
  EXPECT_TRUE(have_ground);
  parameters.traversability.obstacle_min_observation_range = 0.0F;
  const auto all_kept = root.parent_path() / "agt_trav_follower_all_kept";
  std::filesystem::remove_all(all_kept);
  ASSERT_TRUE(TraversabilityGridBuilder::build(root.string(), parameters, &grid, &stats,
                                               &traversability, &error, "", all_kept.string()))
      << error;
  EXPECT_EQ(value_at(grid, -0.95F, 0.35F), 100) << "without the rule the trail is baked in";
  std::ifstream cleanup_report(all_kept / "cloud_cleanup.yaml");
  const std::string report((std::istreambuf_iterator<char>(cleanup_report)),
                          std::istreambuf_iterator<char>());
  EXPECT_NE(report.find("removed_candidate_points: 0\n"), std::string::npos);
  pcl::PCLPointCloud2 empty_candidates;
  EXPECT_EQ(pcl::io::loadPCDFile((all_kept / "removed_candidates.pcd").string(), empty_candidates), 0);
  EXPECT_EQ(empty_candidates.width * empty_candidates.height, 0U);
  EXPECT_FALSE(TraversabilityGridBuilder::build(root.string(), parameters, &grid, &stats,
                                                &traversability, &error, "", clouds.string()));
  EXPECT_NE(error.find("already exists"), std::string::npos);
  EXPECT_FALSE(TraversabilityGridBuilder::build(root.string(), parameters, &grid, &stats,
                                                &traversability, &error, "", (root / "bad").string()));
  EXPECT_FALSE(std::filesystem::exists(root / "bad"));
  std::filesystem::remove_all(root);
  std::filesystem::remove_all(clouds);
  std::filesystem::remove_all(all_kept);
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

TEST(TraversabilityGridBuilderTest, FailedCleanupTransactionLeavesNoPublishedOrStagingDirectory) {
  const Points world = {{2.05F, 0.05F, 0.0F}, {2.45F, 0.05F, 0.0F}};
  const auto root = write_package("agt_trav_cleanup_failure", kThreePoses, world);
  complete_source_integrity(root);
  const auto output = root.parent_path() / "agt_trav_cleanup_failure_output";
  std::filesystem::remove_all(output);
  OccupancyGrid grid;
  ProjectionStats stats;
  TraversabilityStats traversability;
  std::string error;
  // Deterministic I/O failure after the map has been fully computed/staged:
  // map.pgm is a file, so it cannot contain a cloud output child.
  EXPECT_FALSE(TraversabilityGridBuilder::build(root.string(), test_parameters(), &grid, &stats,
      &traversability, &error, "", output.string(), "map.pgm/clouds"));
  EXPECT_FALSE(std::filesystem::exists(output));
  for (const auto &entry : std::filesystem::directory_iterator(output.parent_path())) {
    EXPECT_NE(entry.path().filename().string().find(".agt_trav_cleanup_failure_output.staging-"), 0U);
  }
  // A valid geometric source without its authority manifest must also fail closed.
  std::filesystem::remove(root / "manifest.yaml");
  EXPECT_FALSE(TraversabilityGridBuilder::build(root.string(), test_parameters(), &grid, &stats,
      &traversability, &error, "", output.string()));
  EXPECT_FALSE(std::filesystem::exists(output));
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
