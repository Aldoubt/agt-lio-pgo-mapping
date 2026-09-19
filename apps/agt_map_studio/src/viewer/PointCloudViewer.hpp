#pragma once

#include "io/PCDLoader.hpp"
#include "selection/SelectionBox.h"
#include "selection/SelectionManager.h"
#include "viewer/CameraController.hpp"

#include <QElapsedTimer>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLWidget>
#include <QPoint>
#include <QTimer>
#include <QVector4D>

#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>

#include <memory>

namespace agt_map_studio {

enum class InteractionMode { Navigate, Select, Delete };

class PointCloudViewer : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT

public:
  explicit PointCloudViewer(QWidget *parent = nullptr);
  ~PointCloudViewer() override;

  void set_cloud(LoadedPointCloud cloud, const QString &filename);
  void reset_camera();
  bool save_view(const QString &path, QString *error) const;
  void set_camera_speeds(float speed, float fast_speed);
  void set_show_axis(bool enabled);
  void set_dark_background(bool enabled);
  void set_height_coloring(bool enabled);
  void adjust_point_size(float delta);
  void set_point_size(float size);
  void set_selection_manager(SelectionManager *manager);
  void set_mode(InteractionMode mode);
  InteractionMode mode() const { return mode_; }
  void select_screen_rect(const SelectionBox &box);
  void mark_edit_state_dirty();
  const LoadedPointCloud &cloud() const { return cloud_; }
  void isometric_view();
  void front_view();
  void top_view();

  QString stats_text() const;
  QString filename() const { return filename_; }
  std::size_t point_count() const { return cloud_.valid_point_count; }
  float fps() const { return fps_; }
  bool has_cloud() const { return cloud_.source != nullptr; }

signals:
  void stats_changed(const QString &text);

protected:
  void initializeGL() override;
  void resizeGL(int width, int height) override;
  void paintGL() override;
  void keyPressEvent(QKeyEvent *event) override;
  void keyReleaseEvent(QKeyEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;

private slots:
  void tick();

private:
  void upload_cloud();
  void upload_statuses();
  void draw_axes(const QMatrix4x4 &mvp);
  QString mode_text() const;

  LoadedPointCloud cloud_;
  QString filename_;
  CameraController camera_;
  QOpenGLBuffer cloud_buffer_;
  QOpenGLBuffer status_buffer_;
  QOpenGLBuffer axis_buffer_;
  std::unique_ptr<QOpenGLShaderProgram> shader_;
  QTimer timer_;
  QElapsedTimer fps_timer_;
  int frame_count_ = 0;
  float fps_ = 0.0F;
  float point_size_ = 2.0F;
  bool show_axis_ = true;
  bool dark_background_ = false;
  bool height_coloring_ = true;
  bool gl_ready_ = false;
  bool left_drag_ = false;
  bool right_drag_ = false;
  bool selecting_ = false;
  bool status_buffer_dirty_ = true;
  SelectionBox selection_box_;
  SelectionManager *selection_manager_ = nullptr;
  InteractionMode mode_ = InteractionMode::Navigate;
  QPoint last_mouse_position_;
};

}  // namespace agt_map_studio
