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

TEST(CameraControllerTest, ZoomMovesCameraCloserAndOutWithoutChangingTarget) {
  CameraController camera;
  camera.reset(QVector3D(-10.0F, -5.0F, -1.0F),
               QVector3D(10.0F, 5.0F, 3.0F));
  const QVector3D target = camera.target();
  const float initial_distance = camera.distance();

  camera.zoom(1200.0F);
  EXPECT_LT(camera.distance(), initial_distance);
  EXPECT_EQ(camera.target(), target);

  const float zoomed_distance = camera.distance();
  camera.zoom(-1200.0F);
  EXPECT_GT(camera.distance(), zoomed_distance);
  EXPECT_EQ(camera.target(), target);
}

TEST(CameraControllerTest, WorldPanMovesPositionAndTargetTogether) {
  CameraController camera;
  const QVector3D position = camera.position();
  const QVector3D target = camera.target();
  const QVector3D delta(2.0F, -3.0F, 0.5F);

  camera.pan_world(delta);

  EXPECT_EQ(camera.position(), position + delta);
  EXPECT_EQ(camera.target(), target + delta);
  EXPECT_NEAR(camera.distance(), (position - target).length(), 1e-5F);
}

TEST(CameraControllerTest, FocusHeightMovesCameraOntoVisibleBandWithoutChangingViewDirection) {
  CameraController camera;
  camera.reset(QVector3D(-10.0F, -5.0F, -3.0F),
               QVector3D(10.0F, 5.0F, 61.0F));
  const QVector3D initial_offset = camera.position() - camera.target();
  const float initial_distance = camera.distance();

  camera.focus_height(-1.5F);

  EXPECT_NEAR(camera.target().z(), -1.5F, 1e-5F);
  EXPECT_NEAR(camera.distance(), initial_distance, 1e-5F);
  EXPECT_NEAR((camera.position() - camera.target() - initial_offset).length(),
              0.0F, 1e-5F);
}
}  // namespace
}  // namespace agt_map_studio
