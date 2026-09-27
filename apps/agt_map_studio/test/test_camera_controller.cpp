#include "viewer/CameraController.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace agt_map_studio {
namespace {
TEST(CameraControllerTest, TopViewProjectsMapXYWithoutSingularUpVector) {
  CameraController camera;
  camera.set_viewport(QSize(1200, 900));
  camera.reset(QVector3D(-10.0F, -5.0F, -1.0F),
               QVector3D(10.0F, 5.0F, 3.0F));
  camera.set_top();
  const QMatrix4x4 mvp = camera.projection_matrix() * camera.view_matrix();
  const QVector4D center = mvp * QVector4D(0.0F, 0.0F, 1.0F, 1.0F);
  ASSERT_TRUE(std::isfinite(center.x()));
  ASSERT_TRUE(std::isfinite(center.y()));
  ASSERT_TRUE(std::isfinite(center.w()));
  ASSERT_GT(center.w(), 0.0F);
  EXPECT_NEAR(center.x() / center.w(), 0.0F, 1e-4F);
  EXPECT_NEAR(center.y() / center.w(), 0.0F, 1e-4F);
  const QVector4D right = mvp * QVector4D(1.0F, 0.0F, 1.0F, 1.0F);
  const QVector4D north = mvp * QVector4D(0.0F, 1.0F, 1.0F, 1.0F);
  EXPECT_GT(right.x() / right.w(), 0.0F);
  EXPECT_GT(north.y() / north.w(), 0.0F);
}
}  // namespace
}  // namespace agt_map_studio
