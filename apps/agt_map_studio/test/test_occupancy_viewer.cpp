#include "occupancy/GridMap.hpp"
#include "occupancy/MapYamlLoader.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>

namespace agt_map_studio {
namespace {

TEST(GridMapTest, EmptyMapAndCoordinateRoundTrip) {
  GridMap map;
  EXPECT_TRUE(map.empty());
  std::string error;
  ASSERT_TRUE(map.set_geometry(10, 20, 0.5F, 10.0, -3.0, &error)) << error;
  const GridWorldPoint world = map.pixel_to_world(3, 4);
  EXPECT_DOUBLE_EQ(world.x, 11.5);
  EXPECT_DOUBLE_EQ(world.y, -1.0);
  int pixel_x = -1;
  int pixel_y = -1;
  ASSERT_TRUE(map.world_to_pixel(world.x + 0.2, world.y + 0.2, &pixel_x, &pixel_y));
  EXPECT_EQ(pixel_x, 3);
  EXPECT_EQ(pixel_y, 4);
  EXPECT_FALSE(map.world_to_pixel(100.0, 100.0, &pixel_x, &pixel_y));
}

TEST(MapYamlLoaderTest, LoadsP2AndFlipsIntoLowerLeftGrid) {
  const auto directory = std::filesystem::temp_directory_path() / "agt_map_p2_test";
  std::filesystem::create_directories(directory);
  {
    std::ofstream pgm(directory / "map.pgm");
    pgm << "P2\n# comment\n2 2\n255\n0 205\n254 0\n";
    std::ofstream yaml(directory / "map.yaml");
    yaml << "image: map.pgm\nresolution: 0.5\norigin: [10.0, -3.0, 0.0]\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.196\nnegate: 0\nmode: trinary\n";
  }
  GridMap map;
  MapYamlMetadata metadata;
  std::string error;
  ASSERT_TRUE(MapYamlLoader::load((directory / "map.yaml").string(), &map,
                                  &metadata, &error)) << error;
  EXPECT_EQ(map.width(), 2U);
  EXPECT_EQ(map.height(), 2U);
  EXPECT_EQ(map.at(0, 0), GridMap::kFree);
  EXPECT_EQ(map.at(1, 0), GridMap::kOccupied);
  EXPECT_EQ(map.at(0, 1), GridMap::kOccupied);
  EXPECT_EQ(map.at(1, 1), GridMap::kUnknown);
  std::filesystem::remove_all(directory);
}

TEST(MapYamlLoaderTest, LoadsP5Binary) {
  const auto directory = std::filesystem::temp_directory_path() / "agt_map_p5_test";
  std::filesystem::create_directories(directory);
  {
    std::ofstream pgm(directory / "map.pgm", std::ios::binary);
    pgm << "P5\n2 1\n255\n";
    const unsigned char pixels[] = {0U, 254U};
    pgm.write(reinterpret_cast<const char *>(pixels), sizeof(pixels));
    std::ofstream yaml(directory / "map.yaml");
    yaml << "image: map.pgm\nresolution: 1.0\norigin: [0.0, 0.0, 0.0]\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.196\nnegate: 0\n";
  }
  GridMap map;
  MapYamlMetadata metadata;
  std::string error;
  ASSERT_TRUE(MapYamlLoader::load((directory / "map.yaml").string(), &map,
                                  &metadata, &error)) << error;
  EXPECT_EQ(map.at(0, 0), GridMap::kOccupied);
  EXPECT_EQ(map.at(1, 0), GridMap::kFree);
  std::filesystem::remove_all(directory);
}

}  // namespace
}  // namespace agt_map_studio
