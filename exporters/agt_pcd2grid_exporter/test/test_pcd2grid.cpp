#include "agt_pcd2grid_exporter/OccupancyGridWriter.hpp"
#include "agt_pcd2grid_exporter/PCDProjector.hpp"

#include <gtest/gtest.h>

#include <pcl/PCLPointCloud2.h>

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
  ProjectionParameters parameters;
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
  ProjectionParameters parameters;
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
  ProjectionParameters parameters;
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
  ProjectionParameters parameters;
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

}  // namespace
}  // namespace agt_pcd2grid_exporter
