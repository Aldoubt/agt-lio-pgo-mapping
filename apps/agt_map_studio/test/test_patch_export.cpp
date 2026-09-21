#include "occupancy/RefinementModel.hpp"
#include "occupancy/commands/DrawObstacleCommand.hpp"
#include "occupancy/commands/EraseRectangleCommand.hpp"
#include "occupancy/commands/FillPolygonCommand.hpp"
#include "occupancy/commands/ForbiddenPolygonCommand.hpp"

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>

namespace agt_map_studio {
namespace {

GridMap make_base_map() {
  GridMap map;
  std::string error;
  EXPECT_TRUE(map.set_geometry(6, 6, 1.0F, 0.0, 0.0, &error)) << error;
  std::fill(map.cells().begin(), map.cells().end(), GridMap::kFree);
  map.cells()[1U * map.width() + 1U] = GridMap::kOccupied;
  map.cells()[1U * map.width() + 2U] = GridMap::kOccupied;
  return map;
}

std::filesystem::path temp_file(const char *name) {
  return std::filesystem::temp_directory_path() / name;
}

}  // namespace

TEST(FillPolygonCommandTest, FillsCellCentersInsidePolygon) {
  RefinementModel model;
  model.set_base_map(make_base_map(), MapYamlMetadata());
  // Triangle covering cells whose centres are (0.5,0.5), (1.5,0.5), (0.5,1.5)
  auto command = FillPolygonCommand::create(
      model, {{0.0, 0.0}, {3.0, 0.0}, {0.0, 3.0}}, GridMap::kOccupied);
  ASSERT_TRUE(command);
  std::string error;
  ASSERT_TRUE(model.execute(std::move(command), &error)) << error;
  EXPECT_EQ(model.effective_at(0, 0), GridMap::kOccupied);
  EXPECT_EQ(model.effective_at(1, 0), GridMap::kOccupied);
  EXPECT_EQ(model.effective_at(0, 1), GridMap::kOccupied);
  EXPECT_EQ(model.effective_at(2, 2), GridMap::kFree);
  ASSERT_TRUE(model.undo());
  EXPECT_EQ(model.effective_at(0, 0), GridMap::kFree);
}

TEST(FillPolygonCommandTest, NoOpPolygonReturnsNull) {
  RefinementModel model;
  model.set_base_map(make_base_map(), MapYamlMetadata());
  EXPECT_FALSE(FillPolygonCommand::create(model, {{3.0, 3.0}, {5.0, 3.0}, {5.0, 5.0}}, GridMap::kFree));
  EXPECT_FALSE(FillPolygonCommand::create(model, {{0.0, 0.0}, {1.0, 0.0}}, GridMap::kFree));
}

TEST(NavigationPatchExportTest, WritesPatchNavMapSchema) {
  RefinementModel model;
  model.set_base_map(make_base_map(), MapYamlMetadata());
  std::string error;
  ASSERT_TRUE(model.execute(EraseRectangleCommand::create(model, {0.5, 0.5}, {2.5, 1.5}), &error)) << error;
  ASSERT_TRUE(model.execute(DrawObstacleCommand::create(model, {0.0, 4.0}, {3.0, 4.0}, 0.5), &error)) << error;
  ASSERT_TRUE(model.execute(FillPolygonCommand::create(
      model, {{3.0, 3.0}, {5.0, 3.0}, {5.0, 5.0}}, GridMap::kUnknown), &error)) << error;
  ASSERT_TRUE(model.execute(ForbiddenPolygonCommand::create(
      model, {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}}), &error)) << error;
  // An undone operation must not be exported.
  ASSERT_TRUE(model.execute(FillPolygonCommand::create(
      model, {{0.0, 3.0}, {1.0, 3.0}, {1.0, 4.0}, {0.0, 4.0}}, GridMap::kOccupied), &error)) << error;
  ASSERT_TRUE(model.undo());
  EXPECT_EQ(model.patch_edit_count(), 3U);

  const auto patch_path = temp_file("agt_studio_patch_test.yaml");
  ASSERT_TRUE(model.write_navigation_patch(patch_path.string(), &error)) << error;
  const YAML::Node root = YAML::LoadFile(patch_path.string());
  ASSERT_TRUE(root["edits"].IsSequence());
  ASSERT_EQ(root["edits"].size(), 3U);
  EXPECT_EQ(root["edits"][0]["mode"].as<std::string>(), "free");
  EXPECT_EQ(root["edits"][0]["polygon_m"].size(), 4U);
  EXPECT_EQ(root["edits"][1]["mode"].as<std::string>(), "occupied");
  EXPECT_EQ(root["edits"][2]["mode"].as<std::string>(), "unknown");
  for (const auto &edit : root["edits"]) {
    ASSERT_GE(edit["polygon_m"].size(), 3U);
    for (const auto &vertex : edit["polygon_m"]) ASSERT_EQ(vertex.size(), 2U);
  }

  const auto zones_path = temp_file("agt_studio_keepout_test.yaml");
  ASSERT_TRUE(model.write_keepout_zones(zones_path.string(), &error)) << error;
  const YAML::Node zones = YAML::LoadFile(zones_path.string());
  ASSERT_EQ(zones["zones"].size(), 1U);
  EXPECT_EQ(zones["zones"][0]["polygon_m"].size(), 3U);
  std::filesystem::remove(patch_path);
  std::filesystem::remove(zones_path);
}

TEST(NavigationPatchExportTest, FingerprintTracksActiveOperations) {
  RefinementModel model;
  model.set_base_map(make_base_map(), MapYamlMetadata());
  EXPECT_TRUE(model.active_fingerprint().empty());
  std::string error;
  ASSERT_TRUE(model.execute(EraseRectangleCommand::create(model, {0.5, 0.5}, {2.5, 1.5}), &error)) << error;
  const std::string first = model.active_fingerprint();
  EXPECT_FALSE(first.empty());
  ASSERT_TRUE(model.undo());
  EXPECT_TRUE(model.active_fingerprint().empty());
  ASSERT_TRUE(model.redo());
  EXPECT_EQ(model.active_fingerprint(), first);
}

}  // namespace agt_map_studio
