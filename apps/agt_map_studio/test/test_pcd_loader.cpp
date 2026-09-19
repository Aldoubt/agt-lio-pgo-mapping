#include "io/PCDLoader.hpp"

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>

namespace {

class PCDLoaderTest : public ::testing::Test {
protected:
  void SetUp() override {
    path_ = "/tmp/agt_map_studio_loader_test.pcd";
    std::ofstream stream(path_);
    stream << "# .PCD v0.7 - Point Cloud Data file format\n"
           << "VERSION 0.7\n"
           << "FIELDS x y z intensity ring\n"
           << "SIZE 4 4 4 4 2\n"
           << "TYPE F F F F U\n"
           << "COUNT 1 1 1 1 1\n"
           << "WIDTH 3\n"
           << "HEIGHT 1\n"
           << "VIEWPOINT 0 0 0 1 0 0 0\n"
           << "POINTS 3\n"
           << "DATA ascii\n"
           << "0 1 2 10 1\n"
           << "3 4 5 20 2\n"
           << "6 7 8 30 3\n";
  }

  void TearDown() override { std::remove(path_.c_str()); }

  std::string path_;
};

TEST_F(PCDLoaderTest, LoadsXyzAndPreservesSourceFields) {
  agt_map_studio::LoadedPointCloud cloud;
  std::string error;
  ASSERT_TRUE(agt_map_studio::PCDLoader::load(path_, &cloud, &error)) << error;
  ASSERT_NE(cloud.source, nullptr);
  EXPECT_EQ(cloud.point_count(), 3U);
  EXPECT_EQ(cloud.valid_point_count, 3U);
  EXPECT_TRUE(cloud.has_intensity);
  ASSERT_EQ(cloud.xyz.size(), 9U);
  ASSERT_EQ(cloud.intensity.size(), 3U);
  EXPECT_FLOAT_EQ(cloud.xyz[0], 0.0F);
  EXPECT_FLOAT_EQ(cloud.xyz[8], 8.0F);
  EXPECT_FLOAT_EQ(cloud.intensity[1], 20.0F);
  ASSERT_EQ(cloud.source->fields.size(), 5U);
  EXPECT_EQ(cloud.source->fields[4].name, "ring");
}

TEST(PCDLoaderStandaloneTest, RejectsMissingCoordinates) {
  const std::string path = "/tmp/agt_map_studio_invalid_test.pcd";
  std::ofstream stream(path);
  stream << "VERSION 0.7\n"
         << "FIELDS intensity\n"
         << "SIZE 4\nTYPE F\nCOUNT 1\nWIDTH 1\nHEIGHT 1\n"
         << "POINTS 1\nDATA ascii\n1\n";
  stream.close();
  agt_map_studio::LoadedPointCloud cloud;
  std::string error;
  EXPECT_FALSE(agt_map_studio::PCDLoader::load(path, &cloud, &error));
  EXPECT_NE(error.find("x, y and z"), std::string::npos);
  std::remove(path.c_str());
}

TEST(PCDLoaderArtifactTest, LoadsConfiguredMapArtifact) {
  const char *configured_path = std::getenv("AGT_MAP_STUDIO_TEST_PCD");
  if (!configured_path || configured_path[0] == '\0') {
    GTEST_SKIP() << "AGT_MAP_STUDIO_TEST_PCD is not set";
  }
  agt_map_studio::LoadedPointCloud cloud;
  std::string error;
  ASSERT_TRUE(agt_map_studio::PCDLoader::load(configured_path, &cloud, &error))
      << error;
  EXPECT_GT(cloud.point_count(), 100000U);
  EXPECT_NE(cloud.source, nullptr);
  EXPECT_TRUE(cloud.has_intensity);
}

}  // namespace
