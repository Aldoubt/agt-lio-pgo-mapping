#include "occupancy/RefinementModel.hpp"
#include "occupancy/MapYamlLoader.hpp"
#include "occupancy/commands/DrawObstacleCommand.hpp"
#include "occupancy/commands/EraseRectangleCommand.hpp"
#include "occupancy/commands/ForbiddenPolygonCommand.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <algorithm>

namespace agt_map_studio {
namespace {

GridMap make_base_map() {
  GridMap map;
  std::string error;
  EXPECT_TRUE(map.set_geometry(5, 5, 1.0F, 0.0, 0.0, &error)) << error;
  std::fill(map.cells().begin(), map.cells().end(), GridMap::kFree);
  map.cells()[1U * map.width() + 1U] = GridMap::kOccupied;
  map.cells()[1U * map.width() + 2U] = GridMap::kOccupied;
  return map;
}

TEST(RefinementModelTest, EraseUndoRedoUsesSparseCellChanges) {
  RefinementModel model;
  model.set_base_map(make_base_map(), MapYamlMetadata());
  auto command = EraseRectangleCommand::create(model, {0.5, 0.5}, {2.5, 1.5});
  ASSERT_TRUE(command);
  ASSERT_EQ(command->operation().changes.size(), 2U);
  std::string error;
  ASSERT_TRUE(model.execute(std::move(command), &error)) << error;
  EXPECT_EQ(model.effective_at(1, 1), GridMap::kFree);
  EXPECT_EQ(model.effective_at(2, 1), GridMap::kFree);
  ASSERT_TRUE(model.undo());
  EXPECT_EQ(model.effective_at(1, 1), GridMap::kOccupied);
  ASSERT_TRUE(model.redo());
  EXPECT_EQ(model.effective_at(2, 1), GridMap::kFree);
}

TEST(RefinementModelTest, ObstacleLineChangesFreeCellsOnly) {
  RefinementModel model;
  model.set_base_map(make_base_map(), MapYamlMetadata());
  auto command = DrawObstacleCommand::create(model, {0.0, 0.0}, {3.0, 0.0}, 1.0);
  ASSERT_TRUE(command);
  std::string error;
  ASSERT_TRUE(model.execute(std::move(command), &error)) << error;
  EXPECT_EQ(model.effective_at(0, 0), GridMap::kOccupied);
  EXPECT_EQ(model.effective_at(1, 0), GridMap::kOccupied);
}

TEST(RefinementModelTest, ForbiddenPolygonDoesNotChangeOccupancy) {
  RefinementModel model;
  model.set_base_map(make_base_map(), MapYamlMetadata());
  const auto before = model.effective_at(1, 1);
  auto command = ForbiddenPolygonCommand::create(
      model, {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}});
  ASSERT_TRUE(command);
  std::string error;
  ASSERT_TRUE(model.execute(std::move(command), &error)) << error;
  EXPECT_EQ(model.effective_at(1, 1), before);
  ASSERT_EQ(model.forbidden_zones().size(), 1U);
  ASSERT_TRUE(model.undo());
  EXPECT_TRUE(model.forbidden_zones().empty());
  ASSERT_TRUE(model.redo());
  EXPECT_EQ(model.forbidden_zones().size(), 1U);
}

TEST(RefinementModelTest, RefinementRoundTripAndExport) {
  RefinementModel model;
  auto base = make_base_map();
  std::string error;
  base.set_source_paths("/tmp/base_map.yaml", "/tmp/map.pgm");
  model.set_base_map(base, MapYamlMetadata());
  auto command = EraseRectangleCommand::create(model, {0.5, 0.5}, {2.5, 1.5});
  ASSERT_TRUE(command);
  ASSERT_TRUE(model.execute(std::move(command), &error));
  auto forbidden = ForbiddenPolygonCommand::create(
      model, {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}});
  ASSERT_TRUE(forbidden);
  ASSERT_TRUE(model.execute(std::move(forbidden), &error));

  const auto directory = std::filesystem::temp_directory_path() / "agt_refinement_test";
  std::filesystem::remove_all(directory);
  ASSERT_TRUE(model.save_refinement_yaml((directory / "map_refinement.yaml").string(),
                                         &error)) << error;
  RefinementModel restored;
  restored.set_base_map(make_base_map(), MapYamlMetadata());
  ASSERT_TRUE(restored.load_refinement_yaml(
      (directory / "map_refinement.yaml").string(), &error)) << error;
  EXPECT_EQ(restored.effective_at(1, 1), GridMap::kFree);
  EXPECT_EQ(restored.forbidden_zones().size(), 1U);
  ASSERT_TRUE(model.export_navigation_map((directory / "navigation_map").string(),
                                          &error)) << error;
  EXPECT_TRUE(std::filesystem::exists(directory / "navigation_map/map.pgm"));
  EXPECT_TRUE(std::filesystem::exists(directory / "navigation_map/map.yaml"));
  EXPECT_TRUE(std::filesystem::exists(directory / "navigation_map/map_refinement.yaml"));
  EXPECT_TRUE(std::filesystem::exists(directory / "navigation_map/metadata.yaml"));
  GridMap exported_map;
  MapYamlMetadata exported_metadata;
  ASSERT_TRUE(MapYamlLoader::load(
      (directory / "navigation_map/map.yaml").string(), &exported_map,
      &exported_metadata, &error)) << error;
  EXPECT_EQ(exported_map.width(), 5U);
  EXPECT_EQ(exported_map.at(1, 1), GridMap::kFree);
  std::filesystem::remove_all(directory);
}

}  // namespace
}  // namespace agt_map_studio
