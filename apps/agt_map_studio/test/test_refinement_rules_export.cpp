#include "selection/SelectionManager.h"

#include <QTemporaryDir>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace agt_map_studio {

TEST(RefinementRulesExportTest, WritesRefinementCoreRulesForActiveDeletes) {
  SelectionManager manager;
  manager.reset(10);

  SelectionGeometry box;
  box.rule_type = "remove_box";
  box.box.min = Eigen::Vector3f(0.0F, 0.0F, 0.0F);
  box.box.max = Eigen::Vector3f(1.0F, 2.0F, 3.0F);
  box.box.valid = true;
  manager.select_points({0, 1}, box);
  ASSERT_TRUE(manager.delete_selected());

  SelectionGeometry polygon;
  polygon.rule_type = "remove_polygon";
  polygon.polygon_xy = {{0.0, 0.0}, {4.0, 0.0}, {4.0, 4.0}};
  polygon.has_z_range = true;
  polygon.z_min = -0.5;
  polygon.z_max = 2.5;
  manager.select_points({2, 3}, polygon);
  ASSERT_TRUE(manager.delete_selected());

  SelectionGeometry band;
  band.rule_type = "remove_height_band";
  band.z_min = 2.0;
  band.z_max = 3.0;
  manager.select_points({4}, band);
  ASSERT_TRUE(manager.delete_selected());

  SelectionGeometry sphere;
  sphere.rule_type = "remove_sphere";
  sphere.center = Eigen::Vector3d(1.0, 1.0, 1.0);
  sphere.radius = 0.75;
  manager.select_points({5}, sphere);
  ASSERT_TRUE(manager.delete_selected());
  ASSERT_TRUE(manager.undo());  // sphere is undone and must be skipped

  const QString first_fingerprint = manager.active_fingerprint();
  EXPECT_FALSE(first_fingerprint.isEmpty());

  QTemporaryDir temporary;
  ASSERT_TRUE(temporary.isValid());
  const QString path = temporary.filePath("refinement.yaml");
  QString error;
  ASSERT_TRUE(manager.write_refinement_rules(path, "source.pcd", &error)) << error.toStdString();

  const YAML::Node root = YAML::LoadFile(path.toStdString());
  EXPECT_EQ(root["version"].as<int>(), 1);
  ASSERT_TRUE(root["operations"].IsSequence());
  ASSERT_EQ(root["operations"].size(), 3U);
  EXPECT_EQ(root["operations"][0]["type"].as<std::string>(), "remove_box");
  EXPECT_DOUBLE_EQ(root["operations"][0]["max"]["z"].as<double>(), 3.0);
  EXPECT_EQ(root["operations"][1]["type"].as<std::string>(), "remove_polygon");
  EXPECT_EQ(root["operations"][1]["points"].size(), 3U);
  EXPECT_DOUBLE_EQ(root["operations"][1]["z_range"][1].as<double>(), 2.5);
  EXPECT_EQ(root["operations"][2]["type"].as<std::string>(), "remove_height_band");
  EXPECT_DOUBLE_EQ(root["operations"][2]["min_z"].as<double>(), 2.0);

  ASSERT_TRUE(manager.redo());
  EXPECT_NE(manager.active_fingerprint(), first_fingerprint);
}

TEST(RefinementRulesExportTest, InvertSelectionFlipsVisibleOnly) {
  SelectionManager manager;
  manager.reset(4);
  manager.select_points({0}, AxisAlignedBoundingBox());
  ASSERT_TRUE(manager.delete_selected());
  manager.select_points({1}, AxisAlignedBoundingBox());
  manager.invert_selection();
  EXPECT_EQ(manager.selected_count(), 2U);
  EXPECT_EQ(manager.deleted_count(), 1U);
  EXPECT_EQ(manager.statuses()[1], PointStatus::VISIBLE);
}

}  // namespace agt_map_studio
