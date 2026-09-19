#pragma once

#include <QMatrix4x4>
#include <QPoint>
#include <QSize>
#include <QVector3D>

#include <set>

namespace agt_map_studio {

class CameraController {
public:
  CameraController();

  void set_speed(float speed);
  void set_fast_speed(float speed);
  void set_viewport(const QSize &size);
  void update(float seconds);
  void set_key(int key, bool pressed);
  void orbit(float dx, float dy);
  void pan(float dx, float dy);
  void zoom(float wheel_delta);
  void reset(const QVector3D &min_bound, const QVector3D &max_bound);
  void set_isometric();
  void set_front();
  void set_top();

  QMatrix4x4 view_matrix() const;
  QMatrix4x4 projection_matrix() const;
  QVector3D position() const { return position_; }
  QVector3D target() const { return target_; }
  float distance() const;
  float speed() const { return speed_; }
  float fast_speed() const { return fast_speed_; }

private:
  QVector3D forward() const;
  QVector3D right() const;

  QVector3D position_;
  QVector3D target_;
  QVector3D world_up_;
  QSize viewport_;
  float speed_;
  float fast_speed_;
  float field_of_view_degrees_;
  float near_clip_;
  float far_clip_;
  std::set<int> pressed_keys_;
};

}  // namespace agt_map_studio
