#pragma once

#include "confidence/SpatialConfidenceModel.hpp"
#include "geometry/GeometryEvidenceModel.hpp"
#include "confidence/SpatialConfidenceEditor.hpp"
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
#include <QPolygon>
#include <QTimer>
#include <QVector3D>
#include <QVector4D>

#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>

#include <memory>
#include <optional>
#include <array>
#include <vector>

namespace agt_map_studio {

enum class InteractionMode { Navigate, Select, Delete };
enum class PointColorMode {
  Height, Solid, AutoConfidence, FinalConfidence,
  ObservationScore, PersistenceScore,
  GeometryNormalShape, GeometryTranslationQ, GeometryRotationQ,
  GeometryTranslationWeak, GeometryRotationWeak,
};

inline bool is_geometry_color_mode(PointColorMode mode) {
  return mode == PointColorMode::GeometryNormalShape ||
         mode == PointColorMode::GeometryTranslationQ ||
         mode == PointColorMode::GeometryRotationQ ||
         mode == PointColorMode::GeometryTranslationWeak ||
         mode == PointColorMode::GeometryRotationWeak;
}

// How a selection is drawn in Select/Delete mode.
enum class SelectionTool { ScreenRect, PolygonPrism, Sphere };
enum class AuxiliaryLayer { Structure = 0, Blocks = 1, Query = 2, Candidate = 3 };

class PointCloudViewer : public QOpenGLWidget, protected QOpenGLFunctions {
  Q_OBJECT

public:
  explicit PointCloudViewer(QWidget *parent = nullptr);
  ~PointCloudViewer() override;

  void set_cloud(LoadedPointCloud cloud, const QString &filename);
  void set_auxiliary_cloud(AuxiliaryLayer layer, const LoadedPointCloud &cloud,
                           bool visible = true);
  void set_auxiliary_visible(AuxiliaryLayer layer, bool visible);
  void clear_auxiliary_cloud(AuxiliaryLayer layer);
  void set_query_pick_mode(bool enabled) { query_pick_mode_ = enabled; }
  void reset_camera();
  bool save_view(const QString &path, QString *error) const;
  void set_camera_speeds(float speed, float fast_speed);
  void set_show_axis(bool enabled);
  void set_dark_background(bool enabled);
  void set_height_coloring(bool enabled);
  void set_color_mode(PointColorMode mode);
  PointColorMode color_mode() const { return color_mode_; }
  void set_confidence_model(const SpatialConfidenceModel *model);
  void set_geometry_model(const GeometryEvidenceModel *model);
  void set_confidence_editor(const SpatialConfidenceEditor *editor);
  void set_confidence_selection_manager(SelectionManager *manager);
  void set_stable_only(bool enabled);
  bool stable_only() const { return stable_only_; }
  bool has_confidence() const { return confidence_model_ && !confidence_model_->empty(); }
  bool showing_confidence() const;
  void refresh_confidence_preview();  // override or selection changed; GPU dirty, not per-frame
  void adjust_point_size(float delta);
  void set_point_size(float size);
  void set_selection_manager(SelectionManager *manager);
  void set_mode(InteractionMode mode);
  InteractionMode mode() const { return mode_; }
  void set_selection_tool(SelectionTool tool);
  SelectionTool selection_tool() const { return tool_; }
  // Z window applied to PolygonPrism / ScreenRect selections when enabled.
  void set_z_window(bool enabled, double z_min, double z_max);
  void set_sphere_radius(double radius_m) { sphere_radius_ = radius_m; }
  double sphere_radius() const { return sphere_radius_; }

  void select_screen_rect(const SelectionBox &box);
  void select_screen_polygon(const QPolygon &polygon);
  void select_height_band(double z_min, double z_max);
  void select_sphere_at(const QPoint &screen, double radius_m);
  void cancel_pending_polygon();
  // Recompute the remove_box geometry from the currently selected points.
  void rebuild_selection_box_from_points();
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
  void delete_requested_outside_delete_mode();
  void confidence_voxel_selected(std::size_t index);
  void map_point_selected(double x, double y, double z);

protected:
  void initializeGL() override;
  void resizeGL(int width, int height) override;
  void paintGL() override;
  void keyPressEvent(QKeyEvent *event) override;
  void keyReleaseEvent(QKeyEvent *event) override;
  void mousePressEvent(QMouseEvent *event) override;
  void mouseMoveEvent(QMouseEvent *event) override;
  void mouseReleaseEvent(QMouseEvent *event) override;
  void mouseDoubleClickEvent(QMouseEvent *event) override;
  void wheelEvent(QWheelEvent *event) override;

private slots:
  void tick();

private:
  void upload_cloud();
  void upload_statuses();
  void upload_confidence_cloud();
  void upload_confidence_statuses();
  void upload_confidence_colors();
  void upload_auxiliary_clouds();
  bool confidence_mode() const;
  const std::vector<float> &active_xyz() const;
  SelectionManager *active_selection_manager() const;
  void signal_confidence_selection();
  void emit_stats();
  void pick_confidence_voxel(const QPoint &screen);
  void draw_axes(const QMatrix4x4 &mvp);
  QString mode_text() const;
  QString tool_text() const;
  std::optional<QPoint> project(std::size_t index, const QMatrix4x4 &mvp) const;
  std::optional<Eigen::Vector3f> unproject_to_ground(const QPoint &screen, float z) const;
  bool passes_z_window(float z) const;
  bool visible_for_selection(std::size_t index) const;
  bool confidence_stable(std::size_t index) const;
  std::size_t confidence_stable_count() const;
  void finish_polygon_selection();

  LoadedPointCloud cloud_;
  QString filename_;
  CameraController camera_;
  QOpenGLBuffer cloud_buffer_;
  QOpenGLBuffer status_buffer_;
  QOpenGLBuffer confidence_buffer_;
  QOpenGLBuffer confidence_status_buffer_;
  QOpenGLBuffer confidence_color_buffer_;
  QOpenGLBuffer axis_buffer_;
  std::unique_ptr<QOpenGLShaderProgram> shader_;
  QTimer timer_;
  QElapsedTimer fps_timer_;
  int frame_count_ = 0;
  float fps_ = 0.0F;
  float point_size_ = 2.0F;
  bool show_axis_ = true;
  bool dark_background_ = false;
  PointColorMode color_mode_ = PointColorMode::Height;
  const SpatialConfidenceModel *confidence_model_ = nullptr;  // owned by MainWindow
  const GeometryEvidenceModel *geometry_model_ = nullptr; // read-only, owned by MainWindow
  const SpatialConfidenceEditor *confidence_editor_ = nullptr;  // intent preview, separate
  SelectionManager *confidence_selection_manager_ = nullptr;
  Eigen::Vector3f confidence_min_bound_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f confidence_max_bound_ = Eigen::Vector3f::Zero();
  bool stable_only_ = false;
  bool confidence_status_dirty_ = true;
  bool confidence_colors_dirty_ = true;
  QString cached_stats_text_;
  bool gl_ready_ = false;
  struct AuxiliaryCloud {
    QOpenGLBuffer positions{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer statuses{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer colors{QOpenGLBuffer::VertexBuffer};
    std::vector<float> xyz;
    QVector3D color;
    bool visible = false;
    bool dirty = true;
  };
  std::array<AuxiliaryCloud, 4> auxiliary_clouds_;
  bool query_pick_mode_ = false;
  bool left_drag_ = false;
  bool right_drag_ = false;
  bool selecting_ = false;
  bool status_buffer_dirty_ = true;
  SelectionBox selection_box_;
  QPolygon pending_polygon_;
  SelectionManager *selection_manager_ = nullptr;
  InteractionMode mode_ = InteractionMode::Navigate;
  SelectionTool tool_ = SelectionTool::ScreenRect;
  bool z_window_enabled_ = false;
  double z_window_min_ = -1.0;
  double z_window_max_ = 3.0;
  double sphere_radius_ = 0.5;
  QPoint last_mouse_position_;
};

}  // namespace agt_map_studio
