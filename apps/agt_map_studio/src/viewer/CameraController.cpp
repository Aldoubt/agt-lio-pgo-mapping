#include "viewer/CameraController.hpp"

#include <QKeyEvent>
#include <QtMath>

#include <algorithm>

namespace agt_map_studio {

CameraController::CameraController()
    : position_(0.0F, -5.0F, 3.0F),
      target_(0.0F, 0.0F, 0.0F),
      world_up_(0.0F, 0.0F, 1.0F),
      viewport_(1280, 720),
      speed_(0.5F),
      fast_speed_(3.0F),
      field_of_view_degrees_(55.0F),
      near_clip_(0.01F),
      far_clip_(10000.0F) {}

void CameraController::set_speed(float speed) {
  speed_ = std::max(0.001F, speed);
}

void CameraController::set_fast_speed(float speed) {
  fast_speed_ = std::max(speed_, speed);
}

void CameraController::set_viewport(const QSize &size) {
  viewport_ = size;
}

QVector3D CameraController::forward() const {
  const QVector3D value = target_ - position_;
  return value.lengthSquared() > 1e-8F ? value.normalized()
                                       : QVector3D(0.0F, 1.0F, 0.0F);
}

QVector3D CameraController::right() const {
  const QVector3D value = QVector3D::crossProduct(forward(), world_up_);
  return value.lengthSquared() > 1e-8F ? value.normalized()
                                       : QVector3D(1.0F, 0.0F, 0.0F);
}

float CameraController::distance() const {
  return std::max(0.01F, (position_ - target_).length());
}

void CameraController::set_key(int key, bool pressed) {
  if (pressed) {
    pressed_keys_.insert(key);
  } else {
    pressed_keys_.erase(key);
  }
}

void CameraController::update(float seconds) {
  if (pressed_keys_.empty()) return;
  const bool fast = pressed_keys_.count(Qt::Key_Shift) != 0;
  const float amount = (fast ? fast_speed_ : speed_) * seconds;
  QVector3D delta;
  const QVector3D camera_forward = forward();
  const QVector3D camera_right = right();
  if (pressed_keys_.count(Qt::Key_W)) delta += camera_forward * amount;
  if (pressed_keys_.count(Qt::Key_S) || pressed_keys_.count(Qt::Key_Down))
    delta -= camera_forward * amount;
  if (pressed_keys_.count(Qt::Key_A)) delta -= camera_right * amount;
  if (pressed_keys_.count(Qt::Key_D)) delta += camera_right * amount;
  if (pressed_keys_.count(Qt::Key_Q)) delta -= world_up_ * amount;
  if (pressed_keys_.count(Qt::Key_E)) delta += world_up_ * amount;
  position_ += delta;
  target_ += delta;
}

void CameraController::orbit(float dx, float dy) {
  QVector3D offset = position_ - target_;
  const float radius = std::max(0.01F, offset.length());
  QMatrix4x4 rotation;
  rotation.rotate(-dx * 0.35F, world_up_);
  rotation.rotate(-dy * 0.35F, right());
  offset = rotation * offset;
  if (offset.lengthSquared() > 1e-8F) {
    position_ = target_ + offset.normalized() * radius;
  }
}

void CameraController::pan(float dx, float dy) {
  const float scale = distance() * 0.0025F;
  const QVector3D up = QVector3D::crossProduct(right(), forward()).normalized();
  const QVector3D delta = (-right() * dx + up * dy) * scale;
  position_ += delta;
  target_ += delta;
}

void CameraController::zoom(float wheel_delta) {
  const float scale = qExp(-wheel_delta * 0.001F);
  const float new_distance = std::clamp(distance() * scale, 0.02F, 1.0e7F);
  position_ = target_ + (position_ - target_).normalized() * new_distance;
}

void CameraController::reset(const QVector3D &min_bound,
                              const QVector3D &max_bound) {
  target_ = (min_bound + max_bound) * 0.5F;
  const float diagonal = std::max(1.0F, (max_bound - min_bound).length());
  far_clip_ = std::max(10000.0F, diagonal * 100.0F);
  const QVector3D direction = QVector3D(1.0F, -1.0F, 0.75F).normalized();
  position_ = target_ - direction * diagonal * 1.25F;
}

void CameraController::set_isometric() {
  const float radius = std::max(1.0F, distance());
  const QVector3D direction = QVector3D(1.0F, -1.0F, 0.75F).normalized();
  position_ = target_ - direction * radius;
}

void CameraController::set_front() {
  const float radius = std::max(1.0F, distance());
  position_ = target_ + QVector3D(0.0F, -radius, 0.0F);
}

void CameraController::set_top() {
  const float radius = std::max(1.0F, distance());
  position_ = target_ + QVector3D(0.0F, 0.0F, radius);
}

QMatrix4x4 CameraController::view_matrix() const {
  QMatrix4x4 matrix;
  matrix.lookAt(position_, target_, world_up_);
  return matrix;
}

QMatrix4x4 CameraController::projection_matrix() const {
  QMatrix4x4 matrix;
  const float aspect = viewport_.height() > 0
                           ? static_cast<float>(viewport_.width()) /
                                 static_cast<float>(viewport_.height())
                           : 1.0F;
  matrix.perspective(field_of_view_degrees_, aspect, near_clip_, far_clip_);
  return matrix;
}

}  // namespace agt_map_studio
