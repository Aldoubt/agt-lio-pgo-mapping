#include "agt_pcd2grid_exporter/OccupancyGridWriter.hpp"
#include "agt_pcd2grid_exporter/PCDProjector.hpp"
#include "agt_pcd2grid_exporter/TemporalPersistenceFilter.hpp"

#include <gtest/gtest.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <tuple>
#include <vector>

namespace agt_pcd2grid_exporter {
namespace {

pcl::PCLPointCloud2 make_cloud(
    const std::vector<std::tuple<float, float, float>> &points) {
  pcl::PCLPointCloud2 cloud;
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.height = 1;
  cloud.point_step = 12;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.fields = {
      pcl::PCLPointField{"x", 0, pcl::PCLPointField::FLOAT32, 1},
      pcl::PCLPointField{"y", 4, pcl::PCLPointField::FLOAT32, 1},
      pcl::PCLPointField{"z", 8, pcl::PCLPointField::FLOAT32, 1},
  };
  cloud.data.resize(cloud.row_step);
  for (std::size_t i = 0; i < points.size(); ++i) {
    const float values[] = {std::get<0>(points[i]), std::get<1>(points[i]),
                            std::get<2>(points[i])};
    std::memcpy(cloud.data.data() + i * cloud.point_step, values, sizeof(values));
  }
  return cloud;
}

ProjectionParameters fixed_parameters() {
  ProjectionParameters parameters;
  parameters.projection_mode = ProjectionMode::FixedHeight;
  parameters.closing_radius_cells = 0U;
  parameters.min_component_cells = 1U;
  return parameters;
}

TEST(PCDProjectorTest, EmptyCloudIsRejected) {
  pcl::PCLPointCloud2 cloud;
  OccupancyGrid grid;
  ProjectionStats stats;
  std::string error;
  EXPECT_FALSE(PCDProjector::project(cloud, ProjectionParameters(), &grid, &stats,
                                     &error));
  EXPECT_NE(error.find("no point data"), std::string::npos);
}

TEST(PCDProjectorTest, SinglePointProjectsToOneCell) {
  ProjectionParameters parameters = fixed_parameters();
  parameters.resolution = 1.0F;
  parameters.occupied_threshold = 1U;
  OccupancyGrid grid;
  ProjectionStats stats;
  std::string error;
  ASSERT_TRUE(PCDProjector::project(
      make_cloud({{2.2F, -1.2F, 0.0F}}), parameters, &grid, &stats, &error)) << error;
  EXPECT_EQ(grid.width, 1U);
  EXPECT_EQ(grid.height, 1U);
  EXPECT_EQ(grid.origin_x, 2.0F);
  EXPECT_EQ(grid.origin_y, -2.0F);
  EXPECT_EQ(grid.hits(0, 0), 1U);
  EXPECT_EQ(grid.value(0, parameters), 100);
}

TEST(PCDProjectorTest, MultiplePointsShareHitCount) {
  ProjectionParameters parameters = fixed_parameters();
  parameters.resolution = 1.0F;
  parameters.occupied_threshold = 3U;
  OccupancyGrid grid;
  ProjectionStats stats;
  std::string error;
  ASSERT_TRUE(PCDProjector::project(
      make_cloud({{0.1F, 0.1F, 0.0F}, {0.2F, 0.2F, 0.1F}, {0.3F, 0.3F, 0.2F}}),
      parameters, &grid, &stats, &error)) << error;
  EXPECT_EQ(grid.hits(0, 0), 3U);
  EXPECT_EQ(stats.occupied_cells, 1U);
  EXPECT_EQ(grid.value(0, parameters), 100);
}

TEST(PCDProjectorTest, ZFilterExcludesPoints) {
  ProjectionParameters parameters = fixed_parameters();
  parameters.resolution = 1.0F;
  parameters.z_min = -0.1F;
  parameters.z_max = 0.1F;
  OccupancyGrid grid;
  ProjectionStats stats;
  std::string error;
  ASSERT_TRUE(PCDProjector::project(
      make_cloud({{0.0F, 0.0F, 0.0F}, {1.0F, 1.0F, 1.0F}}), parameters,
      &grid, &stats, &error)) << error;
  EXPECT_EQ(stats.input_points, 2U);
  EXPECT_EQ(stats.accepted_points, 1U);
  EXPECT_EQ(stats.z_filtered_points, 1U);
}

TEST(OccupancyGridWriterTest, WritesNav2Package) {
  ProjectionParameters parameters = fixed_parameters();
  parameters.resolution = 1.0F;
  parameters.occupied_threshold = 1U;
  OccupancyGrid grid;
  ProjectionStats stats;
  std::string error;
  ASSERT_TRUE(PCDProjector::project(
      make_cloud({{0.0F, 0.0F, 0.0F}}), parameters, &grid, &stats, &error));
  const auto directory = std::filesystem::temp_directory_path() / "agt_pcd2grid_test";
  std::filesystem::remove_all(directory);
  ASSERT_TRUE(OccupancyGridWriter::write_navigation_map(
      grid, parameters, stats, directory.string(), "input.pcd", &error)) << error;
  EXPECT_TRUE(std::filesystem::exists(directory / "map.pgm"));
  EXPECT_TRUE(std::filesystem::exists(directory / "map.yaml"));
  EXPECT_TRUE(std::filesystem::exists(directory / "projection.yaml"));
  EXPECT_TRUE(std::filesystem::exists(directory / "metadata.yaml"));
  std::ifstream stream(directory / "map.yaml");
  const std::string yaml((std::istreambuf_iterator<char>(stream)), {});
  EXPECT_NE(yaml.find("occupied_thresh"), std::string::npos);
  EXPECT_NE(yaml.find("origin"), std::string::npos);
  std::filesystem::remove_all(directory);
}

TEST(PCDProjectorTest, LocalGroundSeparatesFreeAndObstacleEvidence) {
  ProjectionParameters parameters;
  parameters.resolution = 1.0F;
  parameters.ground_cell_size = 3.0F;
  parameters.ground_neighbor_radius = 0U;
  parameters.ground_percentile = 0.0F;
  parameters.ground_free_tolerance = 0.05F;
  parameters.obstacle_min_height = 0.2F;
  parameters.obstacle_max_height = 2.0F;
  parameters.occupied_threshold = 1U;
  parameters.closing_radius_cells = 0U;
  parameters.min_component_cells = 1U;
  OccupancyGrid grid;
  ProjectionStats stats;
  std::string error;
  ASSERT_TRUE(PCDProjector::project(
      make_cloud({{0.1F, 0.1F, 0.0F}, {1.1F, 0.1F, 0.0F},
                  {0.2F, 0.2F, 0.5F}}),
      parameters, &grid, &stats, &error)) << error;
  ASSERT_EQ(grid.width, 2U);
  EXPECT_EQ(grid.value(0U, parameters), 100);
  EXPECT_EQ(grid.value(1U, parameters), 0);
  EXPECT_EQ(stats.ground_points, 2U);
  EXPECT_EQ(stats.obstacle_points, 1U);
}

TEST(PCDProjectorTest, SmallIsolatedObstacleComponentsAreRemoved) {
  ProjectionParameters parameters = fixed_parameters();
  parameters.resolution = 1.0F;
  parameters.occupied_threshold = 1U;
  parameters.min_component_cells = 2U;
  OccupancyGrid grid;
  ProjectionStats stats;
  std::string error;
  ASSERT_TRUE(PCDProjector::project(
      make_cloud({{0.1F, 0.1F, 0.0F}, {1.1F, 0.1F, 0.0F},
                  {5.1F, 5.1F, 0.0F}}),
      parameters, &grid, &stats, &error)) << error;
  EXPECT_EQ(grid.value(0U, parameters), 100);
  EXPECT_EQ(grid.value(static_cast<std::size_t>(5U) * grid.width + 5U, parameters), -1);
  EXPECT_EQ(stats.removed_small_component_cells, 1U);
}

TEST(TemporalPersistenceFilterTest, RemovesSingleKeyframeVoxels) {
  const auto root = std::filesystem::temp_directory_path() / "agt_temporal_filter_test";
  std::filesystem::remove_all(root);
  std::filesystem::create_directories(root / "patches");
  for (int index = 0; index < 2; ++index) {
    pcl::PointCloud<pcl::PointXYZI> patch;
    pcl::PointXYZI stable;
    stable.x = 1.0F;
    stable.y = 1.0F;
    stable.z = 0.5F;
    patch.push_back(stable);
    if (index == 0) {
      pcl::PointXYZI transient;
      transient.x = 5.0F;
      transient.y = 0.0F;
      transient.z = 0.5F;
      patch.push_back(transient);
    }
    ASSERT_EQ(pcl::io::savePCDFileBinary(
                  (root / "patches" / (std::to_string(index) + ".pcd")).string(), patch),
              0);
  }
  {
    std::ofstream poses(root / "poses_timed.txt");
    poses << "0.pcd 1.0 0 0 0 1 0 0 0\n"
          << "1.pcd 2.0 0 0 0 1 0 0 0\n";
  }
  ProjectionParameters parameters;
  parameters.temporal_voxel_size = 0.2F;
  parameters.temporal_min_observations = 2U;
  parameters.temporal_min_keyframe_span = 1U;
  pcl::PCLPointCloud2 filtered;
  TemporalFilterStats stats;
  std::string error;
  ASSERT_TRUE(TemporalPersistenceFilter::filter_package(
      root.string(), parameters, &filtered, &stats, &error)) << error;
  pcl::PointCloud<pcl::PointXYZI> points;
  pcl::fromPCLPointCloud2(filtered, points);
  EXPECT_EQ(points.size(), 2U);
  EXPECT_EQ(stats.input_points, 3U);
  EXPECT_EQ(stats.removed_points, 1U);
  std::filesystem::remove_all(root);
}

}  // namespace
}  // namespace agt_pcd2grid_exporter
