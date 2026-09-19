#include "selection/SelectionManager.h"

#include <QTemporaryDir>

#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>

#include <cstring>

namespace agt_map_studio {

TEST(SelectionManagerTest, DeleteUndoRedoPreservesPointStates) {
  SelectionManager manager;
  manager.reset(4);
  AxisAlignedBoundingBox box;
  box.min = Eigen::Vector3f(0.0F, 1.0F, 2.0F);
  box.max = Eigen::Vector3f(3.0F, 4.0F, 5.0F);
  box.valid = true;
  manager.select_points({1, 3}, box);
  EXPECT_EQ(manager.selected_count(), 2U);
  ASSERT_TRUE(manager.delete_selected());
  EXPECT_EQ(manager.deleted_count(), 2U);
  EXPECT_EQ(manager.visible_count(), 2U);
  ASSERT_TRUE(manager.undo());
  EXPECT_EQ(manager.deleted_count(), 0U);
  EXPECT_EQ(manager.visible_count(), 4U);
  ASSERT_TRUE(manager.redo());
  EXPECT_EQ(manager.deleted_count(), 2U);
  EXPECT_EQ(manager.history().size(), 1U);
  EXPECT_FALSE(manager.history().front().undone);
}

TEST(SelectionManagerTest, EmptyDeleteDoesNothing) {
  SelectionManager manager;
  manager.reset(3);
  EXPECT_FALSE(manager.delete_selected());
  EXPECT_FALSE(manager.undo());
  EXPECT_FALSE(manager.redo());
  EXPECT_EQ(manager.visible_count(), 3U);
}

TEST(SelectionManagerTest, ExportKeepsOriginalPclFields) {
  pcl::PCLPointCloud2 source;
  source.width = 4;
  source.height = 1;
  source.point_step = 16;
  source.row_step = source.width * source.point_step;
  source.fields = {
      pcl::PCLPointField{"x", 0, pcl::PCLPointField::FLOAT32, 1},
      pcl::PCLPointField{"y", 4, pcl::PCLPointField::FLOAT32, 1},
      pcl::PCLPointField{"z", 8, pcl::PCLPointField::FLOAT32, 1},
      pcl::PCLPointField{"intensity", 12, pcl::PCLPointField::FLOAT32, 1},
  };
  source.data.resize(source.row_step);
  for (std::size_t i = 0; i < source.width; ++i) {
    const float values[] = {static_cast<float>(i), 1.0F, 2.0F, 10.0F + i};
    std::memcpy(source.data.data() + i * source.point_step, values,
                sizeof(values));
  }

  QTemporaryDir temporary;
  ASSERT_TRUE(temporary.isValid());
  const QString input = temporary.filePath("input.pcd");
  ASSERT_EQ(pcl::io::savePCDFile(input.toStdString(), source,
                                 Eigen::Vector4f::Zero(),
                                 Eigen::Quaternionf::Identity(), true), 0);
  LoadedPointCloud loaded;
  std::string error;
  ASSERT_TRUE(PCDLoader::load(input.toStdString(), &loaded, &error)) << error;

  SelectionManager manager;
  manager.reset(loaded.point_count());
  manager.select_points({1}, AxisAlignedBoundingBox());
  ASSERT_TRUE(manager.delete_selected());
  QString export_error;
  ASSERT_TRUE(manager.export_clean_map(loaded, temporary.filePath("clean_map"),
                                       input, &export_error))
      << export_error.toStdString();

  LoadedPointCloud clean;
  ASSERT_TRUE(PCDLoader::load(temporary.filePath("clean_map/map.pcd").toStdString(),
                              &clean, &error))
      << error;
  EXPECT_EQ(clean.point_count(), 3U);
  ASSERT_TRUE(clean.source);
  ASSERT_EQ(clean.source->fields.size(), 4U);
  EXPECT_EQ(clean.source->fields[3].name, "intensity");
}

}  // namespace agt_map_studio
