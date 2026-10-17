#include "viewer/PointCloudViewer.hpp"

#include "confidence/ConfidenceColor.hpp"
#include "geometry/GeometryColor.hpp"

#include <QPolygonF>
#include <QVector2D>

#include <QKeyEvent>
#include <QLinearGradient>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <stdexcept>
#include <utility>

namespace agt_map_studio {

PointCloudViewer::PointCloudViewer(QWidget *parent)
    : QOpenGLWidget(parent), cloud_buffer_(QOpenGLBuffer::VertexBuffer),
      status_buffer_(QOpenGLBuffer::VertexBuffer),
      confidence_buffer_(QOpenGLBuffer::VertexBuffer),
      confidence_status_buffer_(QOpenGLBuffer::VertexBuffer),
      confidence_color_buffer_(QOpenGLBuffer::VertexBuffer),
      axis_buffer_(QOpenGLBuffer::VertexBuffer) {
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  connect(&timer_, &QTimer::timeout, this, &PointCloudViewer::tick);
  timer_.start(16);
}

PointCloudViewer::~PointCloudViewer() {
  if (gl_ready_) {
    makeCurrent();
    cloud_buffer_.destroy();
    status_buffer_.destroy();
    confidence_buffer_.destroy();
    confidence_status_buffer_.destroy();
    confidence_color_buffer_.destroy();
    axis_buffer_.destroy();
    for (auto &layer : auxiliary_clouds_) {
      layer.positions.destroy();
      layer.statuses.destroy();
      layer.colors.destroy();
    }
    doneCurrent();
  }
}

void PointCloudViewer::set_cloud(LoadedPointCloud cloud,
                                 const QString &filename) {
  cloud_ = std::move(cloud);
  filename_ = filename;
  for (auto &layer : auxiliary_clouds_) {
    layer.xyz.clear();
    layer.visible = false;
    layer.dirty = true;
  }
  status_buffer_dirty_ = true;
  if (has_cloud()) {
    reset_camera();
  }
  if (gl_ready_) {
    makeCurrent();
    upload_cloud();
    upload_auxiliary_clouds();
    doneCurrent();
  }
  emit_stats();
  update();
}

void PointCloudViewer::set_auxiliary_cloud(AuxiliaryLayer layer_id,
                                            const LoadedPointCloud &cloud,
                                            bool visible, float opacity) {
  auto &layer = auxiliary_clouds_.at(static_cast<std::size_t>(layer_id));
  layer.xyz = cloud.xyz;
  layer.visible = visible && !layer.xyz.empty();
  const QVector3D colors[] = {QVector3D(0.20F, 0.90F, 0.35F),
                              QVector3D(0.70F, 0.45F, 1.00F),
                              QVector3D(0.05F, 0.90F, 1.00F),
                              QVector3D(1.00F, 0.34F, 0.08F),
                              QVector3D(1.00F, 0.52F, 0.02F)};
  layer.color = colors[static_cast<std::size_t>(layer_id)];
  layer.opacity = std::clamp(opacity, 0.05F, 1.0F);
  layer.dirty = true;
  if (gl_ready_) {
    makeCurrent();
    upload_auxiliary_clouds();
    doneCurrent();
  }
  update();
}

void PointCloudViewer::set_primary_visible(bool visible) {
  if (primary_visible_ == visible) return;
  primary_visible_ = visible;
  update();
}

void PointCloudViewer::set_auxiliary_opacity(AuxiliaryLayer layer_id, float opacity) {
  auxiliary_clouds_.at(static_cast<std::size_t>(layer_id)).opacity = std::clamp(opacity, 0.05F, 1.0F);
  update();
}

void PointCloudViewer::set_auxiliary_visible(AuxiliaryLayer layer_id,
                                              bool visible) {
  auto &layer = auxiliary_clouds_.at(static_cast<std::size_t>(layer_id));
  layer.visible = visible && !layer.xyz.empty();
  update();
}

void PointCloudViewer::clear_auxiliary_cloud(AuxiliaryLayer layer_id) {
  auto &layer = auxiliary_clouds_.at(static_cast<std::size_t>(layer_id));
  layer.xyz.clear();
  layer.visible = false;
  layer.dirty = true;
  if (gl_ready_) {
    makeCurrent();
    upload_auxiliary_clouds();
    doneCurrent();
  }
  update();
}

void PointCloudViewer::set_camera_speeds(float speed, float fast_speed) {
  camera_.set_speed(speed);
  camera_.set_fast_speed(fast_speed);
}

void PointCloudViewer::set_show_axis(bool enabled) {
  show_axis_ = enabled;
  update();
}

void PointCloudViewer::set_dark_background(bool enabled) {
  dark_background_ = enabled;
  update();
}

void PointCloudViewer::set_height_coloring(bool enabled) {
  set_color_mode(enabled ? PointColorMode::Height : PointColorMode::Solid);
}

bool PointCloudViewer::confidence_mode() const {
  return has_confidence() && color_mode_ != PointColorMode::Height &&
         color_mode_ != PointColorMode::Solid;
}

bool PointCloudViewer::showing_confidence() const { return confidence_mode(); }

const std::vector<float> &PointCloudViewer::active_xyz() const {
  return confidence_mode() ? confidence_model_->xyz() : cloud_.xyz;
}

SelectionManager *PointCloudViewer::active_selection_manager() const {
  return confidence_mode() ? confidence_selection_manager_ : selection_manager_;
}

void PointCloudViewer::emit_stats() {
  cached_stats_text_ = stats_text();
  emit stats_changed(cached_stats_text_);
}

void PointCloudViewer::set_color_mode(PointColorMode mode) {
  if (mode != PointColorMode::Height && mode != PointColorMode::Solid && !has_confidence()) return;
  if (is_geometry_color_mode(mode) && (!geometry_model_ || geometry_model_->empty())) return;
  color_mode_ = mode;
  confidence_colors_dirty_ = true;
  confidence_status_dirty_ = true;
  emit_stats();
  update();
}

void PointCloudViewer::set_confidence_model(const SpatialConfidenceModel *model) {
  confidence_model_ = model && !model->empty() ? model : nullptr;
  confidence_status_dirty_ = true;
  confidence_colors_dirty_ = true;
  if (confidence_model_) {
    confidence_min_bound_ = Eigen::Vector3f::Constant(std::numeric_limits<float>::infinity());
    confidence_max_bound_ = -confidence_min_bound_;
    const auto &xyz = confidence_model_->xyz();
    for (std::size_t i = 0; i < xyz.size(); i += 3U) {
      const Eigen::Vector3f p(xyz[i], xyz[i + 1U], xyz[i + 2U]);
      confidence_min_bound_ = confidence_min_bound_.cwiseMin(p);
      confidence_max_bound_ = confidence_max_bound_.cwiseMax(p);
    }
  } else {
    color_mode_ = PointColorMode::Height;
    confidence_min_bound_.setZero();
    confidence_max_bound_.setZero();
  }
  if (gl_ready_) {
    makeCurrent();
    upload_confidence_cloud();
    doneCurrent();
  }
  emit_stats();
  update();
}

void PointCloudViewer::set_geometry_model(const GeometryEvidenceModel *model) {
  geometry_model_ = model && !model->empty() ? model : nullptr;
  if (!geometry_model_ && is_geometry_color_mode(color_mode_)) {
    color_mode_ = has_confidence() ? PointColorMode::AutoConfidence : PointColorMode::Height;
  }
  confidence_colors_dirty_ = true;
  emit_stats();
  update();
}

void PointCloudViewer::set_confidence_editor(const SpatialConfidenceEditor *editor) {
  confidence_editor_ = editor;
  refresh_confidence_preview();
}

void PointCloudViewer::set_confidence_selection_manager(SelectionManager *manager) {
  confidence_selection_manager_ = manager;
  confidence_status_dirty_ = true;
  emit_stats();
  update();
}

void PointCloudViewer::set_stable_only(bool enabled) {
  stable_only_ = enabled;
  confidence_status_dirty_ = true;
  emit_stats();
  update();
}

void PointCloudViewer::refresh_confidence_preview() {
  confidence_colors_dirty_ = true;
  confidence_status_dirty_ = true;
  emit_stats();
  update();
}

void PointCloudViewer::set_point_size(float size) {
  point_size_ = std::clamp(size, 1.0F, 12.0F);
  update();
}

void PointCloudViewer::adjust_point_size(float delta) {
  set_point_size(point_size_ + delta);
}

void PointCloudViewer::set_selection_manager(SelectionManager *manager) {
  selection_manager_ = manager;
  status_buffer_dirty_ = true;
  update();
}

void PointCloudViewer::set_annotation_overlays(const QVector<AnnotationPolygonOverlay> &overlays) {
  annotation_overlays_ = overlays;
  update();
}

void PointCloudViewer::set_annotation_mode(bool enabled) {
  annotation_mode_ = enabled;
  annotation_vertex_dragging_ = false;
  annotation_drawing_ = false;
  pending_annotation_xy_.clear();
  setFocus();
  update();
}

void PointCloudViewer::begin_annotation_polygon() {
  begin_annotation_geometry(QStringLiteral("polygon_xy"));
}

void PointCloudViewer::begin_annotation_geometry(const QString &geometry_kind) {
  annotation_mode_ = true;
  annotation_drawing_ = true;
  annotation_geometry_kind_ = geometry_kind;
  annotation_vertex_dragging_ = false;
  pending_annotation_xy_.clear();
  setFocus();
  update();
}

void PointCloudViewer::finish_annotation_polygon() {
  const int minimum = annotation_geometry_kind_ == QStringLiteral("polygon_xy") ? 3 : 2;
  if (pending_annotation_xy_.size() >= minimum) {
    emit annotation_geometry_created(annotation_geometry_kind_, pending_annotation_xy_);
    if (annotation_geometry_kind_ == QStringLiteral("polygon_xy"))
      emit annotation_polygon_created(pending_annotation_xy_);
  }
  pending_annotation_xy_.clear();
  annotation_drawing_ = false;
  update();
}

void PointCloudViewer::cancel_annotation_polygon() {
  pending_annotation_xy_.clear();
  annotation_drawing_ = false;
  annotation_vertex_dragging_ = false;
  update();
}

void PointCloudViewer::set_selected_annotation(const QString &annotation_id) {
  selected_annotation_id_ = annotation_id;
  selected_annotation_vertex_index_ = -1;
  for (auto &overlay : annotation_overlays_) overlay.selected = overlay.annotation_id == annotation_id;
  update();
}

void PointCloudViewer::set_mode(InteractionMode mode) {
  mode_ = mode;
  selecting_ = false;
  pending_polygon_.clear();
  if (mode_ == InteractionMode::Navigate) selection_box_ = SelectionBox();
  emit_stats();
  update();
}

void PointCloudViewer::set_selection_tool(SelectionTool tool) {
  tool_ = tool;
  selecting_ = false;
  pending_polygon_.clear();
  selection_box_ = SelectionBox();
  emit_stats();
  update();
}

void PointCloudViewer::set_z_window(bool enabled, double z_min, double z_max) {
  z_window_enabled_ = enabled;
  z_window_min_ = std::min(z_min, z_max);
  z_window_max_ = std::max(z_min, z_max);
  camera_.focus_height(zoom_anchor_height());
  emit_stats();
  update();
}

void PointCloudViewer::set_chinese_ui(bool enabled) {
  chinese_ui_ = enabled;
  emit_stats();
  update();
}

bool PointCloudViewer::passes_z_window(float z) const {
  return !z_window_enabled_ || (z >= z_window_min_ && z <= z_window_max_);
}

bool PointCloudViewer::confidence_stable(std::size_t index) const {
  return confidence_editor_ ? confidence_editor_->stable_preview(index)
                            : confidence_model_->is_stable_preview(index);
}

std::size_t PointCloudViewer::confidence_stable_count() const {
  return confidence_editor_ ? confidence_editor_->stable_preview_count()
                            : confidence_model_->stable_preview_count();
}

bool PointCloudViewer::visible_for_selection(std::size_t index) const {
  const auto *manager = active_selection_manager();
  if (!manager || index >= manager->statuses().size()) return false;
  if (confidence_mode()) {
    return !stable_only_ || confidence_stable(index);
  }
  return manager->statuses()[index] != PointStatus::DELETED;
}

void PointCloudViewer::cancel_pending_polygon() {
  pending_polygon_.clear();
  selecting_ = false;
  update();
}

void PointCloudViewer::signal_confidence_selection() {
  if (!confidence_mode()) return;
  const auto *manager = active_selection_manager();
  const auto index = manager && !manager->selected_indices().empty()
                         ? manager->selected_indices().front()
                         : static_cast<std::size_t>(-1);
  emit confidence_voxel_selected(index);
}

void PointCloudViewer::mark_edit_state_dirty() {
  if (confidence_mode()) {
    confidence_status_dirty_ = true;
    signal_confidence_selection();
  } else {
    status_buffer_dirty_ = true;
  }
  emit_stats();
  update();
}

void PointCloudViewer::isometric_view() {
  camera_.set_isometric();
  update();
}

void PointCloudViewer::front_view() {
  camera_.set_front();
  update();
}

void PointCloudViewer::top_view() {
  camera_.set_top();
  update();
}

void PointCloudViewer::reset_camera() {
  if (confidence_mode()) {
    camera_.reset(QVector3D(confidence_min_bound_.x(), confidence_min_bound_.y(), confidence_min_bound_.z()),
                  QVector3D(confidence_max_bound_.x(), confidence_max_bound_.y(), confidence_max_bound_.z()));
  } else if (has_cloud()) {
    const QVector3D min_bound(cloud_.min_bound.x(), cloud_.min_bound.y(),
                              cloud_.min_bound.z());
    const QVector3D max_bound(cloud_.max_bound.x(), cloud_.max_bound.y(),
                              cloud_.max_bound.z());
    camera_.reset(min_bound, max_bound);
  } else {
    camera_ = CameraController();
  }
  camera_.focus_height(zoom_anchor_height());
  update();
}

void PointCloudViewer::zoom_by(float wheel_delta) {
  camera_.focus_height(zoom_anchor_height());
  camera_.zoom(wheel_delta);
  update();
}

float PointCloudViewer::zoom_anchor_height() const {
  if (z_window_enabled_) {
    return static_cast<float>((z_window_min_ + z_window_max_) * 0.5);
  }
  if (confidence_mode()) {
    return (confidence_min_bound_.z() + confidence_max_bound_.z()) * 0.5F;
  }
  return has_cloud() ? cloud_.center().z() : camera_.target().z();
}

QString PointCloudViewer::stats_text() const {
  const auto *manager = active_selection_manager();
  const std::size_t selected = manager ? manager->selected_count() : 0U;
  QString text;
  if (confidence_mode()) {
    text = chinese_ui_
        ? QStringLiteral("单期置信度体素：%1 | 已选：%2 | 稳定预览：%3 | 模式：%4")
        : QStringLiteral("Single-session confidence voxels: %1 | Selected: %2 | Stable preview: %3 | Mode: %4");
    text = text
        .arg(static_cast<qulonglong>(confidence_model_->voxels().size()))
        .arg(static_cast<qulonglong>(selected))
        .arg(static_cast<qulonglong>(confidence_stable_count()))
        .arg(mode_text());
    if (stable_only_) text += chinese_ui_ ? QStringLiteral(" | 仅显示稳定预览") : QStringLiteral(" | Show Stable Only");
    if (is_geometry_color_mode(color_mode_))
      text += chinese_ui_ ? QStringLiteral(" | 几何只读（非置信度）")
                          : QStringLiteral(" | Geometry read-only (not confidence)");
  } else {
    const QString name = filename_.isEmpty() ? QStringLiteral("(none)") : filename_;
    const std::size_t deleted = selection_manager_ ? selection_manager_->deleted_count() : 0U;
    const std::size_t visible = selection_manager_ ? selection_manager_->visible_count() : point_count();
    text = chinese_ui_
        ? QStringLiteral("文件：%1 | 总点数：%2 | 已选：%3 | 已删除：%4 | 可见：%5 | 模式：%6")
        : QStringLiteral("File: %1 | Total: %2 | Selected: %3 | Deleted: %4 | Visible: %5 | Mode: %6");
    text = text
        .arg(name)
        .arg(static_cast<qulonglong>(point_count()))
        .arg(static_cast<qulonglong>(selected))
        .arg(static_cast<qulonglong>(deleted))
        .arg(static_cast<qulonglong>(visible))
        .arg(mode_text());
  }
  if (mode_ != InteractionMode::Navigate)
    text += chinese_ui_ ? QStringLiteral(" / %1").arg(tool_text())
                        : QStringLiteral(" / %1").arg(tool_text());
  if (z_window_enabled_) {
    text += chinese_ui_
        ? QStringLiteral(" | Z裁剪 [%1, %2] 米").arg(z_window_min_, 0, 'f', 2).arg(z_window_max_, 0, 'f', 2)
        : QStringLiteral(" | Z [%1, %2]").arg(z_window_min_, 0, 'f', 2).arg(z_window_max_, 0, 'f', 2);
  }
  return text + (chinese_ui_ ? QStringLiteral(" | 帧率：%1").arg(fps_, 0, 'f', 1)
                             : QStringLiteral(" | FPS: %1").arg(fps_, 0, 'f', 1));
}

QString PointCloudViewer::tool_text() const {
  switch (tool_) {
    case SelectionTool::PolygonPrism: return chinese_ui_ ? QStringLiteral("多边形") : QStringLiteral("Polygon");
    case SelectionTool::Sphere:
      return chinese_ui_ ? QStringLiteral("球形 %1米").arg(sphere_radius_, 0, 'f', 2)
                         : QStringLiteral("Sphere %1m").arg(sphere_radius_, 0, 'f', 2);
    default: return chinese_ui_ ? QStringLiteral("矩形") : QStringLiteral("Rect");
  }
}

QString PointCloudViewer::mode_text() const {
  switch (mode_) {
    case InteractionMode::Select: return chinese_ui_ ? QStringLiteral("选择") : QStringLiteral("Select");
    case InteractionMode::Delete: return chinese_ui_ ? QStringLiteral("删除") : QStringLiteral("Delete");
    default: return chinese_ui_ ? QStringLiteral("浏览") : QStringLiteral("Navigate");
  }
}

bool PointCloudViewer::save_view(const QString &path, QString *error) const {
  std::ofstream stream(path.toStdString());
  if (!stream) {
    if (error) *error = QStringLiteral("Cannot write view file: %1").arg(path);
    return false;
  }
  YAML::Emitter emitter;
  const QVector3D position = camera_.position();
  const QVector3D target = camera_.target();
  emitter << YAML::BeginMap;
  emitter << YAML::Key << "source_pcd" << YAML::Value << filename_.toStdString();
  emitter << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "position" << YAML::Value << YAML::Flow
          << YAML::BeginSeq << position.x() << position.y() << position.z()
          << YAML::EndSeq;
  emitter << YAML::Key << "target" << YAML::Value << YAML::Flow
          << YAML::BeginSeq << target.x() << target.y() << target.z()
          << YAML::EndSeq;
  emitter << YAML::Key << "distance" << YAML::Value << camera_.distance();
  emitter << YAML::EndMap;
  emitter << YAML::Key << "viewer" << YAML::Value << YAML::BeginMap;
  emitter << YAML::Key << "point_size" << YAML::Value << point_size_;
  emitter << YAML::Key << "show_axis" << YAML::Value << show_axis_;
  emitter << YAML::Key << "dark_background" << YAML::Value << dark_background_;
  const char *color = "height";
  switch (color_mode_) {
    case PointColorMode::Height: color = "height"; break;
    case PointColorMode::Solid: color = "solid"; break;
    case PointColorMode::AutoConfidence: color = "auto_confidence"; break;
    case PointColorMode::FinalConfidence: color = "final_confidence"; break;
    case PointColorMode::ObservationScore: color = "observation_score"; break;
    case PointColorMode::PersistenceScore: color = "persistence_score"; break;
  }
  emitter << YAML::Key << "color_mode" << YAML::Value << color;
  emitter << YAML::EndMap << YAML::EndMap;
  stream << emitter.c_str() << '\n';
  if (!stream.good()) {
    if (error) *error = QStringLiteral("Failed while writing view file: %1").arg(path);
    return false;
  }
  return true;
}

void PointCloudViewer::initializeGL() {
  initializeOpenGLFunctions();
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_PROGRAM_POINT_SIZE);

  shader_ = std::make_unique<QOpenGLShaderProgram>();
  const char *vertex_shader = R"glsl(
    attribute vec3 a_position;
    attribute float a_status;
    attribute vec3 a_rgb;
    uniform mat4 u_mvp;
    uniform float u_point_size;
    varying float v_height;
    varying float v_status;
    varying vec3 v_rgb;
    void main() {
      gl_Position = u_mvp * vec4(a_position, 1.0);
      gl_PointSize = u_point_size;
      v_height = a_position.z;
      v_status = a_status;
      v_rgb = a_rgb;
    }
  )glsl";
  const char *fragment_shader = R"glsl(
    uniform vec4 u_color;
    uniform int u_height_coloring;
    uniform int u_confidence_mode;
    uniform float u_z_min;
    uniform float u_z_max;
    uniform int u_clip_z_enabled;
    uniform float u_clip_z_min;
    uniform float u_clip_z_max;
    uniform float u_layer_alpha;
    varying float v_height;
    varying float v_status;
    varying vec3 v_rgb;

    vec3 height_color(float value) {
      float range = max(u_z_max - u_z_min, 0.000001);
      float t = clamp((value - u_z_min) / range, 0.0, 1.0);
      if (t < 0.25) return mix(vec3(0.18, 0.24, 0.86), vec3(0.0, 0.75, 0.86), t / 0.25);
      if (t < 0.5) return mix(vec3(0.0, 0.75, 0.86), vec3(0.15, 0.75, 0.30), (t - 0.25) / 0.25);
      if (t < 0.75) return mix(vec3(0.15, 0.75, 0.30), vec3(0.95, 0.80, 0.12), (t - 0.5) / 0.25);
      return mix(vec3(0.95, 0.80, 0.12), vec3(0.86, 0.10, 0.10), (t - 0.75) / 0.25);
    }

    void main() {
      if (u_clip_z_enabled == 1 &&
          (v_height < u_clip_z_min || v_height > u_clip_z_max)) discard;
      if (v_status > 2.5) {
        discard;
      } else if (v_status > 1.5) {
        gl_FragColor = vec4(0.9, 0.05, 0.05, 0.85);
      } else if (v_status > 0.5) {
        gl_FragColor = vec4(1.0, 0.75, 0.05, 1.0);
      } else if (u_confidence_mode == 1) {
        gl_FragColor = vec4(v_rgb, u_layer_alpha);
      } else {
        gl_FragColor = u_height_coloring == 1
            ? vec4(height_color(v_height), 1.0)
            : u_color;
      }
    }
  )glsl";
  if (!shader_->addShaderFromSourceCode(QOpenGLShader::Vertex, vertex_shader) ||
      !shader_->addShaderFromSourceCode(QOpenGLShader::Fragment,
                                        fragment_shader) ||
      !shader_->link()) {
    shader_.reset();
    return;
  }

  cloud_buffer_.create();
  status_buffer_.create();
  confidence_buffer_.create();
  confidence_status_buffer_.create();
  confidence_color_buffer_.create();
  axis_buffer_.create();
  for (auto &layer : auxiliary_clouds_) {
    layer.positions.create();
    layer.statuses.create();
    layer.colors.create();
  }
  const float axis[] = {
      0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
  };
  axis_buffer_.bind();
  axis_buffer_.allocate(axis, sizeof(axis));
  axis_buffer_.release();
  gl_ready_ = true;
  upload_cloud();
  upload_statuses();
  upload_confidence_cloud();
  upload_confidence_statuses();
  upload_confidence_colors();
  upload_auxiliary_clouds();
}

void PointCloudViewer::resizeGL(int width, int height) {
  camera_.set_viewport(QSize(width, height));
  glViewport(0, 0, width, height);
}

void PointCloudViewer::paintGL() {
  const QColor background = dark_background_ ? QColor(18, 22, 28)
                                             : QColor(255, 255, 255);
  glClearColor(background.redF(), background.greenF(), background.blueF(), 1.0F);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  const QMatrix4x4 mvp = camera_.projection_matrix() * camera_.view_matrix();
  const bool confidence = confidence_mode();
  if (confidence) {
    if (confidence_status_dirty_) upload_confidence_statuses();
    if (confidence_colors_dirty_) upload_confidence_colors();
  } else if (status_buffer_dirty_) {
    upload_statuses();
  }
  if (std::any_of(auxiliary_clouds_.begin(), auxiliary_clouds_.end(),
                  [](const AuxiliaryCloud &layer) { return layer.dirty; })) {
    upload_auxiliary_clouds();
  }
  if (shader_) {
    shader_->bind();
    shader_->setUniformValue("u_mvp", mvp);
    shader_->setUniformValue("u_point_size", point_size_);
    shader_->setUniformValue("u_height_coloring", color_mode_ == PointColorMode::Height ? 1 : 0);
    shader_->setUniformValue("u_confidence_mode", confidence ? 1 : 0);
    shader_->setUniformValue("u_z_min", cloud_.min_bound.z());
    shader_->setUniformValue("u_z_max", cloud_.max_bound.z());
    shader_->setUniformValue("u_clip_z_enabled", z_window_enabled_ ? 1 : 0);
    shader_->setUniformValue("u_clip_z_min", static_cast<float>(z_window_min_));
    shader_->setUniformValue("u_clip_z_max", static_cast<float>(z_window_max_));
    shader_->setUniformValue("u_layer_alpha", 1.0F);
    shader_->setUniformValue(
        "u_color", dark_background_ ? QVector4D(1.0F, 1.0F, 1.0F, 1.0F)
                                     : QVector4D(0.12F, 0.12F, 0.12F, 1.0F));
    QOpenGLBuffer &positions = confidence ? confidence_buffer_ : cloud_buffer_;
    QOpenGLBuffer &statuses = confidence ? confidence_status_buffer_ : status_buffer_;
    if ((confidence || primary_visible_) && !active_xyz().empty() && positions.isCreated() && statuses.isCreated() &&
        (!confidence || confidence_color_buffer_.isCreated())) {
      positions.bind();
      shader_->enableAttributeArray("a_position");
      shader_->setAttributeBuffer("a_position", GL_FLOAT, 0, 3);
      positions.release();
      statuses.bind();
      shader_->enableAttributeArray("a_status");
      shader_->setAttributeBuffer("a_status", GL_FLOAT, 0, 1);
      statuses.release();
      if (confidence) {
        confidence_color_buffer_.bind();
        shader_->enableAttributeArray("a_rgb");
        shader_->setAttributeBuffer("a_rgb", GL_FLOAT, 0, 3);
        confidence_color_buffer_.release();
      }
      positions.bind();
      glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(active_xyz().size() / 3U));
      shader_->disableAttributeArray("a_position");
      shader_->disableAttributeArray("a_status");
      if (confidence) shader_->disableAttributeArray("a_rgb");
      positions.release();
    }
    shader_->setUniformValue("u_height_coloring", 0);
    shader_->setUniformValue("u_confidence_mode", 1);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (std::size_t layer_index = 0; layer_index < auxiliary_clouds_.size(); ++layer_index) {
      auto &layer = auxiliary_clouds_[layer_index];
      if (!layer.visible || layer.xyz.empty() || !layer.positions.isCreated() ||
          !layer.statuses.isCreated() || !layer.colors.isCreated()) continue;
      const bool comparison_layer = layer_index == static_cast<std::size_t>(AuxiliaryLayer::Comparison);
      shader_->setUniformValue("u_point_size", comparison_layer
          ? std::max(1.25F, point_size_ * 0.8F)
          : std::max(3.0F, point_size_ + 1.5F));
      shader_->setUniformValue("u_layer_alpha", layer.opacity);
      layer.positions.bind();
      shader_->enableAttributeArray("a_position");
      shader_->setAttributeBuffer("a_position", GL_FLOAT, 0, 3);
      layer.statuses.bind();
      shader_->enableAttributeArray("a_status");
      shader_->setAttributeBuffer("a_status", GL_FLOAT, 0, 1);
      layer.colors.bind();
      shader_->enableAttributeArray("a_rgb");
      shader_->setAttributeBuffer("a_rgb", GL_FLOAT, 0, 3);
      glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(layer.xyz.size() / 3U));
      shader_->disableAttributeArray("a_position");
      shader_->disableAttributeArray("a_status");
      shader_->disableAttributeArray("a_rgb");
      layer.colors.release();
      layer.statuses.release();
      layer.positions.release();
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    shader_->setUniformValue("u_layer_alpha", 1.0F);
    if (show_axis_ && axis_buffer_.isCreated()) {
      // The coordinate axes are a navigation aid and must remain visible when
      // the point-cloud height clip is active.
      shader_->setUniformValue("u_clip_z_enabled", 0);
      draw_axes(mvp);
      shader_->setUniformValue("u_clip_z_enabled", z_window_enabled_ ? 1 : 0);
    }
    shader_->release();
  }

  ++frame_count_;
  if (!fps_timer_.isValid()) fps_timer_.start();
  const qint64 elapsed = fps_timer_.elapsed();
  if (elapsed >= 500) {
    fps_ = static_cast<float>(frame_count_) * 1000.0F /
           static_cast<float>(elapsed);
    frame_count_ = 0;
    fps_timer_.restart();
    emit_stats();
  }

  QPainter painter(this);
  painter.setPen(dark_background_ ? Qt::white : Qt::black);
  painter.drawText(12, 22, cached_stats_text_.isEmpty() ? stats_text() : cached_stats_text_);
  if (z_window_enabled_) {
    const QString clip_label = chinese_ui_
        ? QStringLiteral("当前显示高度：Z [%1, %2] 米")
              .arg(z_window_min_, 0, 'f', 2).arg(z_window_max_, 0, 'f', 2)
        : QStringLiteral("Visible height: Z [%1, %2] m")
              .arg(z_window_min_, 0, 'f', 2).arg(z_window_max_, 0, 'f', 2);
    painter.drawText(12, 44, clip_label);
  }
  const QMatrix4x4 overlay_mvp = camera_.projection_matrix() * camera_.view_matrix();
  for (const auto &overlay : annotation_overlays_) {
    QPolygonF screen_polygon;
    for (const auto &point : overlay.vertices_xy_m) {
      const auto screen = project_xy(point, overlay_mvp);
      if (screen) screen_polygon << QPointF(*screen);
    }
    if (screen_polygon.isEmpty()) continue;
    QColor fill = overlay.color;
    fill.setAlpha(overlay.selected ? 55 : 28);
    painter.setPen(QPen(overlay.color, overlay.selected ? 3.0 : 2.0));
    if (overlay.geometry_kind == QStringLiteral("polygon_xy") && screen_polygon.size() >= 3) {
      painter.setBrush(fill);
      painter.drawPolygon(screen_polygon);
    } else if (overlay.geometry_kind == QStringLiteral("polyline_xy") && screen_polygon.size() >= 2) {
      painter.setBrush(Qt::NoBrush);
      painter.drawPolyline(screen_polygon);
    }
    painter.setBrush(Qt::white);
    for (const auto &point : screen_polygon) {
      const qreal radius = overlay.geometry_kind == QStringLiteral("point_xyz") ? 6.0
                           : overlay.selected ? 5.0 : 3.0;
      painter.drawEllipse(point, radius, radius);
    }
    if (overlay.selected) {
      painter.setPen(overlay.color.darker(150));
      painter.drawText(screen_polygon.first() + QPointF(7.0, -7.0), overlay.annotation_type);
    }
  }
  if (annotation_drawing_ && !pending_annotation_xy_.isEmpty()) {
    QPolygonF pending;
    for (const auto &point : pending_annotation_xy_) {
      const auto screen = project_xy(point, overlay_mvp);
      if (screen) pending << QPointF(*screen);
    }
    const auto cursor_world = unproject_to_ground(last_mouse_position_, 0.0F);
    if (cursor_world) {
      const auto cursor = project_xy(QPointF(cursor_world->x(), cursor_world->y()), overlay_mvp);
      if (cursor) pending << QPointF(*cursor);
    }
    painter.setPen(QPen(QColor(255, 150, 20), 2.0, Qt::DashLine));
    painter.setBrush(QColor(255, 150, 20, 35));
    painter.drawPolyline(pending);
    if (annotation_geometry_kind_ == QStringLiteral("polygon_xy") && pending.size() >= 3)
      painter.drawLine(pending.last(), pending.first());
    painter.setPen(dark_background_ ? Qt::white : Qt::black);
    painter.drawText(12, z_window_enabled_ ? 66 : 46,
        chinese_ui_
            ? QStringLiteral("标注几何：%1 个顶点 | Enter 完成 | Backspace 撤销 | Esc 取消")
                  .arg(pending_annotation_xy_.size())
            : QStringLiteral("Annotation geometry: %1 vertices | Enter finish | Backspace undo | Esc cancel")
                  .arg(pending_annotation_xy_.size()));
  }
  if (selecting_ && tool_ == SelectionTool::ScreenRect && selection_box_.is_valid()) {
    QPen pen(QColor(30, 120, 255), 2, Qt::DashLine);
    painter.setPen(pen);
    painter.setBrush(QColor(50, 140, 255, 35));
    painter.drawRect(selection_box_.rect());
  }
  if (tool_ == SelectionTool::PolygonPrism && !pending_polygon_.isEmpty() &&
      mode_ != InteractionMode::Navigate) {
    QPen pen(QColor(30, 120, 255), 2, Qt::DashLine);
    painter.setPen(pen);
    painter.setBrush(QColor(50, 140, 255, 35));
    QPolygon preview = pending_polygon_;
    preview << last_mouse_position_;
    painter.drawPolygon(preview);
    painter.drawText(pending_polygon_.last() + QPoint(8, -8),
                     QStringLiteral("%1 pts, double-click or Enter to close, Esc cancels")
                         .arg(pending_polygon_.size()));
  }
  if (tool_ == SelectionTool::Sphere && mode_ != InteractionMode::Navigate) {
    painter.setPen(QPen(QColor(30, 120, 255), 1, Qt::DashLine));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(last_mouse_position_, 12, 12);
    painter.drawText(last_mouse_position_ + QPoint(16, 4),
                     QStringLiteral("click: sphere r=%1 m").arg(sphere_radius_, 0, 'f', 2));
  }
  if (confidence || (color_mode_ == PointColorMode::Height && has_cloud())) {
    const int legend_width = 180;
    const int legend_height = 12;
    const int legend_x = std::max(12, width() - legend_width - 18);
    const int legend_y = 14;
    QLinearGradient gradient(legend_x, legend_y, legend_x + legend_width, legend_y);
    const bool axes_legend = color_mode_ == PointColorMode::GeometryNormalShape ||
                             color_mode_ == PointColorMode::GeometryTranslationWeak ||
                             color_mode_ == PointColorMode::GeometryRotationWeak;
    if (confidence && axes_legend) {
      gradient.setColorAt(0.0, QColor(235, 50, 50));
      gradient.setColorAt(0.5, QColor(50, 230, 50));
      gradient.setColorAt(1.0, QColor(50, 80, 235));
    } else if (confidence) {
      for (int step = 0; step <= 4; ++step) {
        const float fraction = step / 4.0F;
        const auto c = confidence_color(fraction);
        gradient.setColorAt(fraction, QColor::fromRgbF(c.r, c.g, c.b));
      }
    } else {
      gradient.setColorAt(0.0, QColor(45, 60, 220));
      gradient.setColorAt(0.25, QColor(0, 190, 220));
      gradient.setColorAt(0.5, QColor(40, 190, 80));
      gradient.setColorAt(0.75, QColor(245, 210, 35));
      gradient.setColorAt(1.0, QColor(220, 35, 35));
    }
    painter.fillRect(legend_x, legend_y, legend_width, legend_height, gradient);
    painter.drawRect(legend_x, legend_y, legend_width, legend_height);
    if (confidence) {
    painter.drawText(legend_x, legend_y + 30,
          axes_legend ? QStringLiteral("R                 G                 B")
                      : QStringLiteral("0.0 low"));
      if (!axes_legend) painter.drawText(legend_x + 120, legend_y + 30, QStringLiteral("1.0 high"));
      const char *name = "Auto Confidence";
      if (color_mode_ == PointColorMode::FinalConfidence) name = "Final Confidence";
      if (color_mode_ == PointColorMode::ObservationScore) name = "Observation Score";
      if (color_mode_ == PointColorMode::PersistenceScore) name = "Persistence Evidence";
      if (color_mode_ == PointColorMode::GeometryNormalShape) name = "PCA: R=linearity G=planarity B=scattering";
      if (color_mode_ == PointColorMode::GeometryTranslationQ) name = "Ht Q: directional diversity (not confidence)";
      if (color_mode_ == PointColorMode::GeometryRotationQ) name = "Hr Q: rotational diversity (not confidence)";
      if (color_mode_ == PointColorMode::GeometryTranslationWeak) name = "Ht weak axis: |map x/y/z|";
      if (color_mode_ == PointColorMode::GeometryRotationWeak) name = "Hr weak axis: |map x/y/z|";
      painter.drawText(legend_x - 140, legend_y + 46, QString::fromLatin1(name));
      if (is_geometry_color_mode(color_mode_)) {
        painter.drawText(legend_x, legend_y + 62, QStringLiteral("gray = insufficient evidence"));
      }
    } else {
      painter.drawText(legend_x, legend_y + 30,
                       chinese_ui_
                           ? QStringLiteral("全图 Z 低：%1").arg(cloud_.min_bound.z(), 0, 'f', 2)
                           : QStringLiteral("Map Z low: %1").arg(cloud_.min_bound.z(), 0, 'f', 2));
      painter.drawText(legend_x + 105, legend_y + 30,
                       chinese_ui_
                           ? QStringLiteral("全图高：%1").arg(cloud_.max_bound.z(), 0, 'f', 2)
                           : QStringLiteral("Map high: %1").arg(cloud_.max_bound.z(), 0, 'f', 2));
    }
  }
  painter.end();
}

void PointCloudViewer::draw_axes(const QMatrix4x4 &mvp) {
  shader_->setUniformValue("u_mvp", mvp);
  shader_->setUniformValue("u_point_size", 1.0F);
  shader_->setUniformValue("u_height_coloring", 0);
  shader_->setUniformValue("u_confidence_mode", 0);
  axis_buffer_.bind();
  shader_->enableAttributeArray("a_position");
  shader_->setAttributeBuffer("a_position", GL_FLOAT, 0, 3);
  shader_->setUniformValue("u_color", QVector4D(0.9F, 0.1F, 0.1F, 1.0F));
  glDrawArrays(GL_LINES, 0, 2);
  shader_->setUniformValue("u_color", QVector4D(0.1F, 0.7F, 0.1F, 1.0F));
  glDrawArrays(GL_LINES, 2, 2);
  shader_->setUniformValue("u_color", QVector4D(0.1F, 0.3F, 1.0F, 1.0F));
  glDrawArrays(GL_LINES, 4, 2);
  shader_->disableAttributeArray("a_position");
  axis_buffer_.release();
}

void PointCloudViewer::upload_cloud() {
  if (!gl_ready_ || !cloud_buffer_.isCreated()) return;
  cloud_buffer_.bind();
  cloud_buffer_.setUsagePattern(QOpenGLBuffer::StaticDraw);
  cloud_buffer_.allocate(cloud_.xyz.data(),
                         static_cast<int>(cloud_.xyz.size() * sizeof(float)));
  cloud_buffer_.release();
}

void PointCloudViewer::upload_statuses() {
  if (!gl_ready_ || !status_buffer_.isCreated()) return;
  std::vector<float> values(cloud_.point_count(), 0.0F);
  if (selection_manager_ && selection_manager_->statuses().size() == values.size()) {
    const auto &statuses = selection_manager_->statuses();
    const bool hide_deleted = selection_manager_->hide_deleted();
    const bool isolate = selection_manager_->isolate_selected() &&
                         selection_manager_->selected_count() > 0U;
    for (std::size_t i = 0; i < statuses.size(); ++i) {
      float value = static_cast<float>(statuses[i]);
      if ((hide_deleted && statuses[i] == PointStatus::DELETED) ||
          (isolate && statuses[i] == PointStatus::VISIBLE)) {
        value = 3.0F;  // hidden: discarded by the fragment shader
      }
      values[i] = value;
    }
  }
  status_buffer_.bind();
  status_buffer_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
  if (!values.empty()) {
    status_buffer_.allocate(values.data(),
                            static_cast<int>(values.size() * sizeof(float)));
  } else {
    status_buffer_.allocate(nullptr, 0);
  }
  status_buffer_.release();
  status_buffer_dirty_ = false;
}

void PointCloudViewer::upload_confidence_cloud() {
  if (!gl_ready_ || !confidence_buffer_.isCreated()) return;
  const auto *xyz = confidence_model_ ? &confidence_model_->xyz() : nullptr;
  confidence_buffer_.bind();
  confidence_buffer_.setUsagePattern(QOpenGLBuffer::StaticDraw);
  confidence_buffer_.allocate(!xyz || xyz->empty() ? nullptr : xyz->data(),
                              xyz ? static_cast<int>(xyz->size() * sizeof(float)) : 0);
  confidence_buffer_.release();
}

void PointCloudViewer::upload_confidence_statuses() {
  if (!gl_ready_ || !confidence_status_buffer_.isCreated()) return;
  const auto count = confidence_model_ ? confidence_model_->voxels().size() : 0U;
  std::vector<float> statuses(count, 0.0F);
  const bool matched = confidence_selection_manager_ &&
                       confidence_selection_manager_->statuses().size() == count;
  const bool isolate = matched && confidence_selection_manager_->isolate_selected() &&
                       confidence_selection_manager_->selected_count() != 0U;
  for (std::size_t i = 0; i < count; ++i) {
    if (stable_only_ && !confidence_stable(i)) {
      statuses[i] = 3.0F;  // hidden; not part of formal stable_map.pcd
    } else if (matched) {
      const auto state = confidence_selection_manager_->statuses()[i];
      statuses[i] = isolate && state != PointStatus::SELECTED ? 3.0F
                    : static_cast<float>(state);
    }
  }
  confidence_status_buffer_.bind();
  confidence_status_buffer_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
  confidence_status_buffer_.allocate(statuses.empty() ? nullptr : statuses.data(),
                                     static_cast<int>(statuses.size() * sizeof(float)));
  confidence_status_buffer_.release();
  confidence_status_dirty_ = false;
}

void PointCloudViewer::upload_confidence_colors() {
  if (!gl_ready_ || !confidence_color_buffer_.isCreated()) return;
  std::vector<float> colors;
  if (confidence_model_) {
    colors.reserve(confidence_model_->voxels().size() * 3U);
    for (std::size_t i = 0; i < confidence_model_->voxels().size(); ++i) {
      const auto &v = confidence_model_->voxels()[i];
      ConfidenceRgb c;
      if (is_geometry_color_mode(color_mode_) && geometry_model_) {
        const auto *g = geometry_model_->at_confidence_index(i);
        if (!g) throw std::logic_error("geometry/confidence index correspondence lost");
        GeometryColorMode channel = GeometryColorMode::NormalShape;
        switch (color_mode_) {
          case PointColorMode::GeometryTranslationQ: channel = GeometryColorMode::TranslationQ; break;
          case PointColorMode::GeometryRotationQ: channel = GeometryColorMode::RotationQ; break;
          case PointColorMode::GeometryTranslationWeak: channel = GeometryColorMode::TranslationWeak; break;
          case PointColorMode::GeometryRotationWeak: channel = GeometryColorMode::RotationWeak; break;
          default: break;
        }
        c = geometry_color(*g, channel);
      } else {
        float score = v.auto_confidence;
        switch (color_mode_) {
          case PointColorMode::FinalConfidence:
            score = confidence_editor_ ? confidence_editor_->preview_final(i)
                                       : v.final_confidence;
            break;
          case PointColorMode::ObservationScore: score = v.observation_score; break;
          case PointColorMode::PersistenceScore: score = v.persistence_score; break;
          default: break;
        }
        c = confidence_color(score);
      }
      colors.insert(colors.end(), {c.r, c.g, c.b});
    }
  }
  confidence_color_buffer_.bind();
  confidence_color_buffer_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
  confidence_color_buffer_.allocate(colors.empty() ? nullptr : colors.data(),
                                    static_cast<int>(colors.size() * sizeof(float)));
  confidence_color_buffer_.release();
  confidence_colors_dirty_ = false;
}

void PointCloudViewer::upload_auxiliary_clouds() {
  if (!gl_ready_) return;
  for (auto &layer : auxiliary_clouds_) {
    if (!layer.dirty || !layer.positions.isCreated() ||
        !layer.statuses.isCreated() || !layer.colors.isCreated()) continue;
    layer.positions.bind();
    layer.positions.setUsagePattern(QOpenGLBuffer::StaticDraw);
    layer.positions.allocate(layer.xyz.empty() ? nullptr : layer.xyz.data(),
                             static_cast<int>(layer.xyz.size() * sizeof(float)));
    layer.positions.release();
    std::vector<float> statuses(layer.xyz.size() / 3U, 0.0F);
    layer.statuses.bind();
    layer.statuses.setUsagePattern(QOpenGLBuffer::StaticDraw);
    layer.statuses.allocate(statuses.empty() ? nullptr : statuses.data(),
                            static_cast<int>(statuses.size() * sizeof(float)));
    layer.statuses.release();
    std::vector<float> colors;
    colors.reserve(statuses.size() * 3U);
    for (std::size_t i = 0; i < statuses.size(); ++i) {
      colors.insert(colors.end(), {layer.color.x(), layer.color.y(), layer.color.z()});
    }
    layer.colors.bind();
    layer.colors.setUsagePattern(QOpenGLBuffer::StaticDraw);
    layer.colors.allocate(colors.empty() ? nullptr : colors.data(),
                          static_cast<int>(colors.size() * sizeof(float)));
    layer.colors.release();
    layer.dirty = false;
  }
}

void PointCloudViewer::tick() {
  camera_.update(0.016F);
  update();
}

void PointCloudViewer::keyPressEvent(QKeyEvent *event) {
  // Mode switching is owned by MainWindow (toolbar + F1/F2/F3), so the WASD
  // camera keys are never shadowed here.
  if (annotation_mode_ && event->key() == Qt::Key_Escape) {
    cancel_annotation_polygon();
    event->accept();
    return;
  }
  if (annotation_mode_ && event->key() == Qt::Key_Backspace && annotation_drawing_) {
    if (!pending_annotation_xy_.isEmpty()) pending_annotation_xy_.removeLast();
    update();
    event->accept();
    return;
  }
  if (annotation_mode_ && (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) && annotation_drawing_) {
    finish_annotation_polygon();
    event->accept();
    return;
  }
  if (annotation_mode_ && event->key() == Qt::Key_Delete) {
    if (selected_annotation_vertex_index_ >= 0)
      emit annotation_vertex_delete_requested(selected_annotation_id_, selected_annotation_vertex_index_);
    else
      emit annotation_delete_requested(selected_annotation_id_);
    event->accept();
    return;
  }
  if (!annotation_mode_ && event->key() == Qt::Key_Escape) {
    cancel_pending_polygon();
    if (auto *manager = active_selection_manager()) {
      manager->clear_selection();
      mark_edit_state_dirty();
    }
    event->accept();
    return;
  }
  if (!annotation_mode_ && (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) &&
      tool_ == SelectionTool::PolygonPrism && pending_polygon_.size() >= 3) {
    finish_polygon_selection();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_0) {
    isometric_view();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_1) {
    front_view();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_2) {
    top_view();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_Delete && confidence_mode()) {
    // Confidence voxels are never deleted from map.pcd or from the evidence.
    emit delete_requested_outside_delete_mode();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_Delete && selection_manager_) {
    if (mode_ != InteractionMode::Delete) {
      emit delete_requested_outside_delete_mode();
    } else if (selection_manager_->delete_selected()) {
      mark_edit_state_dirty();
    }
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_R) {
    reset_camera();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_Plus || event->key() == Qt::Key_Equal) {
    adjust_point_size(0.5F);
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_Minus) {
    adjust_point_size(-0.5F);
    event->accept();
    return;
  }
  camera_.set_key(event->key(), true);
  event->accept();
}

void PointCloudViewer::keyReleaseEvent(QKeyEvent *event) {
  camera_.set_key(event->key(), false);
  event->accept();
}

void PointCloudViewer::mousePressEvent(QMouseEvent *event) {
  setFocus();
  last_mouse_position_ = event->pos();
  if (annotation_mode_ && event->button() == Qt::LeftButton) {
    if (annotation_drawing_) {
      const auto world = unproject_to_ground(event->pos(), 0.0F);
      if (world) {
        pending_annotation_xy_.push_back(QPointF(world->x(), world->y()));
        if (annotation_geometry_kind_ == QStringLiteral("point_xyz")) {
          emit annotation_geometry_created(annotation_geometry_kind_, pending_annotation_xy_);
          pending_annotation_xy_.clear();
          annotation_drawing_ = false;
        }
      }
    } else {
      int vertex = -1;
      if (pick_annotation_vertex(event->pos(), &vertex)) {
        selected_annotation_vertex_index_ = vertex;
        annotation_vertex_dragging_ = true;
        dragged_vertex_index_ = vertex;
        dragged_annotation_index_ = -1;
        for (int i = 0; i < annotation_overlays_.size(); ++i) {
          if (annotation_overlays_[i].annotation_id == selected_annotation_id_) {
            dragged_annotation_index_ = i;
            drag_original_vertices_ = annotation_overlays_[i].vertices_xy_m;
            break;
          }
        }
      } else {
        selected_annotation_vertex_index_ = -1;
      }
    }
    update();
    event->accept();
    return;
  }
  if (annotation_mode_ && event->button() == Qt::RightButton && annotation_drawing_) {
    finish_annotation_polygon();
    event->accept();
    return;
  }
  if (query_pick_mode_ && event->button() == Qt::LeftButton && has_cloud()) {
    query_pick_mode_ = false;
    left_drag_ = false;
    const auto point = unproject_to_ground(event->pos(), cloud_.center().z());
    if (point) emit map_point_selected(point->x(), point->y(), point->z());
    event->accept();
    return;
  }
  if (confidence_mode() && event->button() == Qt::LeftButton &&
      (event->modifiers() & Qt::ControlModifier)) {
    pick_confidence_voxel(event->pos());
    event->accept();
    return;
  }
  left_drag_ = event->button() == Qt::LeftButton &&
               mode_ == InteractionMode::Navigate;
  right_drag_ = event->button() == Qt::RightButton;
  if (event->button() == Qt::LeftButton && mode_ != InteractionMode::Navigate) {
    if (tool_ == SelectionTool::ScreenRect) {
      selecting_ = true;
      selection_box_.set_start(event->pos());
      selection_box_.set_end(event->pos());
    } else if (tool_ == SelectionTool::PolygonPrism) {
      pending_polygon_ << event->pos();
      selecting_ = true;
    } else if (tool_ == SelectionTool::Sphere) {
      select_sphere_at(event->pos(), sphere_radius_);
    }
  }
  event->accept();
}

void PointCloudViewer::mouseDoubleClickEvent(QMouseEvent *event) {
  if (annotation_mode_ && event->button() == Qt::LeftButton && annotation_drawing_) {
    if (!pending_annotation_xy_.isEmpty()) pending_annotation_xy_.removeLast();
    finish_annotation_polygon();
    event->accept();
    return;
  }
  if (event->button() == Qt::LeftButton && mode_ != InteractionMode::Navigate &&
      tool_ == SelectionTool::PolygonPrism) {
    // The first click of the double-click already appended a vertex; drop it.
    if (!pending_polygon_.isEmpty()) pending_polygon_.removeLast();
    finish_polygon_selection();
    event->accept();
    return;
  }
  QOpenGLWidget::mouseDoubleClickEvent(event);
}

void PointCloudViewer::mouseMoveEvent(QMouseEvent *event) {
  const QPoint delta = event->pos() - last_mouse_position_;
  last_mouse_position_ = event->pos();
  if (annotation_mode_) {
    const auto world = unproject_to_ground(event->pos(), 0.0F);
    if (world) emit annotation_coordinate_changed(world->x(), world->y(), world->z());
    if (annotation_vertex_dragging_ && dragged_annotation_index_ >= 0 && dragged_vertex_index_ >= 0) {
      const auto point = unproject_to_ground(event->pos(), 0.0F);
      if (point && dragged_vertex_index_ < annotation_overlays_[dragged_annotation_index_].vertices_xy_m.size()) {
        annotation_overlays_[dragged_annotation_index_].vertices_xy_m[dragged_vertex_index_] = QPointF(point->x(), point->y());
      }
    }
    update();
    event->accept();
    return;
  }
  if (selecting_ && tool_ == SelectionTool::ScreenRect) {
    selection_box_.set_end(event->pos());
  } else if (left_drag_) {
    camera_.orbit(delta.x(), delta.y());
  }
  if (right_drag_) camera_.pan(delta.x(), delta.y());
  update();
  event->accept();
}

void PointCloudViewer::mouseReleaseEvent(QMouseEvent *event) {
  if (annotation_mode_ && event->button() == Qt::LeftButton && annotation_vertex_dragging_) {
    finish_annotation_vertex_drag();
    event->accept();
    return;
  }
  if (event->button() == Qt::LeftButton && selecting_ && tool_ == SelectionTool::ScreenRect) {
    selecting_ = false;
    if (selection_box_.is_valid()) select_screen_rect(selection_box_);
  }
  if (event->button() == Qt::LeftButton) left_drag_ = false;
  if (event->button() == Qt::RightButton) right_drag_ = false;
  event->accept();
}

void PointCloudViewer::pick_confidence_voxel(const QPoint &screen) {
  left_drag_ = false;
  selecting_ = false;
  if (!confidence_mode() || !confidence_selection_manager_) return;
  const QMatrix4x4 mvp = camera_.projection_matrix() * camera_.view_matrix();
  std::size_t best = static_cast<std::size_t>(-1);
  qint64 best_distance = 12 * 12;
  for (std::size_t i = 0; i < confidence_model_->voxels().size(); ++i) {
    if (stable_only_ && !confidence_stable(i)) continue;
    const auto p = project(i, mvp);
    if (!p) continue;
    const qint64 dx = static_cast<qint64>(p->x()) - screen.x();
    const qint64 dy = static_cast<qint64>(p->y()) - screen.y();
    const qint64 distance = dx * dx + dy * dy;
    if (distance < best_distance) { best_distance = distance; best = i; }
  }
  if (best == static_cast<std::size_t>(-1)) return;
  SelectionGeometry geometry;
  geometry.rule_type = "remove_box";  // reused selection geometry, not exported as a delete
  geometry.box.valid = true;
  geometry.box.min = confidence_model_->voxels()[best].center;
  geometry.box.max = geometry.box.min;
  confidence_selection_manager_->select_points({best}, geometry);
  mark_edit_state_dirty();
}

std::optional<QPoint> PointCloudViewer::project(std::size_t i, const QMatrix4x4 &mvp) const {
  const auto &xyz = active_xyz();
  const QVector4D clip(mvp * QVector4D(xyz[i * 3U], xyz[i * 3U + 1U],
                                       xyz[i * 3U + 2U], 1.0F));
  if (clip.w() <= 0.0F) return std::nullopt;
  const QVector3D ndc = clip.toVector3DAffine();
  if (!std::isfinite(ndc.x()) || !std::isfinite(ndc.y()) ||
      ndc.x() < -2.0F || ndc.x() > 2.0F ||
      ndc.y() < -2.0F || ndc.y() > 2.0F) return std::nullopt;
  return QPoint(qRound((ndc.x() + 1.0F) * 0.5F * width()),
                qRound((1.0F - ndc.y()) * 0.5F * height()));
}

std::optional<QPoint> PointCloudViewer::project_xy(const QPointF &point, const QMatrix4x4 &mvp) const {
  const QVector4D clip = mvp * QVector4D(static_cast<float>(point.x()), static_cast<float>(point.y()), 0.0F, 1.0F);
  if (clip.w() <= 0.0F) return std::nullopt;
  const QVector3D ndc = clip.toVector3DAffine();
  if (!std::isfinite(ndc.x()) || !std::isfinite(ndc.y()) || ndc.x() < -2.0F || ndc.x() > 2.0F ||
      ndc.y() < -2.0F || ndc.y() > 2.0F) return std::nullopt;
  return QPoint(qRound((ndc.x() + 1.0F) * 0.5F * width()),
                qRound((1.0F - ndc.y()) * 0.5F * height()));
}

bool PointCloudViewer::pick_annotation_vertex(const QPoint &screen, int *vertex_index) const {
  if (!vertex_index || selected_annotation_id_.isEmpty()) return false;
  const auto mvp = camera_.projection_matrix() * camera_.view_matrix();
  for (const auto &overlay : annotation_overlays_) {
    if (overlay.annotation_id != selected_annotation_id_) continue;
    qint64 best = 12 * 12;
    int index = -1;
    for (int i = 0; i < overlay.vertices_xy_m.size(); ++i) {
      const auto projected = project_xy(overlay.vertices_xy_m[i], mvp);
      if (!projected) continue;
      const qint64 dx = static_cast<qint64>(projected->x()) - screen.x();
      const qint64 dy = static_cast<qint64>(projected->y()) - screen.y();
      const qint64 distance = dx * dx + dy * dy;
      if (distance < best) { best = distance; index = i; }
    }
    if (index >= 0) { *vertex_index = index; return true; }
  }
  return false;
}

void PointCloudViewer::finish_annotation_vertex_drag() {
  if (dragged_annotation_index_ >= 0 && dragged_vertex_index_ >= 0 &&
      dragged_annotation_index_ < annotation_overlays_.size()) {
    const auto &overlay = annotation_overlays_[dragged_annotation_index_];
    if (overlay.vertices_xy_m != drag_original_vertices_) {
      emit annotation_geometry_edited(overlay.annotation_id, overlay.geometry_kind,
                                      overlay.vertices_xy_m);
      if (overlay.geometry_kind == QStringLiteral("polygon_xy"))
        emit annotation_polygon_edited(overlay.annotation_id, overlay.vertices_xy_m);
    }
  }
  annotation_vertex_dragging_ = false;
  dragged_annotation_index_ = -1;
  dragged_vertex_index_ = -1;
  drag_original_vertices_.clear();
}

std::optional<Eigen::Vector3f> PointCloudViewer::unproject_to_ground(const QPoint &screen,
                                                                     float z) const {
  // Intersect the pick ray with the horizontal plane at height z (map frame).
  const QMatrix4x4 mvp = camera_.projection_matrix() * camera_.view_matrix();
  bool invertible = false;
  const QMatrix4x4 inverse = mvp.inverted(&invertible);
  if (!invertible || width() <= 0 || height() <= 0) return std::nullopt;
  const float nx = 2.0F * static_cast<float>(screen.x()) / static_cast<float>(width()) - 1.0F;
  const float ny = 1.0F - 2.0F * static_cast<float>(screen.y()) / static_cast<float>(height());
  const QVector4D near_h = inverse * QVector4D(nx, ny, -1.0F, 1.0F);
  const QVector4D far_h = inverse * QVector4D(nx, ny, 1.0F, 1.0F);
  if (qFuzzyIsNull(near_h.w()) || qFuzzyIsNull(far_h.w())) return std::nullopt;
  const QVector3D origin = near_h.toVector3DAffine();
  const QVector3D direction = far_h.toVector3DAffine() - origin;
  if (qFuzzyIsNull(direction.z())) return std::nullopt;
  const float t = (z - origin.z()) / direction.z();
  if (t < 0.0F) return std::nullopt;
  const QVector3D hit = origin + direction * t;
  return Eigen::Vector3f(hit.x(), hit.y(), hit.z());
}

namespace {

void grow_box(AxisAlignedBoundingBox &box, const Eigen::Vector3f &point) {
  if (!box.valid) {
    box.min = point;
    box.max = point;
    box.valid = true;
  } else {
    box.min = box.min.cwiseMin(point);
    box.max = box.max.cwiseMax(point);
  }
}

}  // namespace

void PointCloudViewer::select_screen_rect(const SelectionBox &box) {
  auto *manager = active_selection_manager();
  const auto &xyz = active_xyz();
  if (!manager || !box.is_valid() || xyz.empty() ||
      manager->statuses().size() != (xyz.size() / 3U)) {
    return;
  }
  const QMatrix4x4 mvp = camera_.projection_matrix() * camera_.view_matrix();
  std::vector<std::size_t> indices;
  SelectionGeometry geometry;
  geometry.rule_type = "remove_box";
  for (std::size_t i = 0; i < (xyz.size() / 3U); ++i) {
    const float z = xyz[i * 3U + 2U];
    if (!passes_z_window(z)) continue;
    const auto screen = project(i, mvp);
    if (!screen || !box.contains(*screen)) continue;
    if (!visible_for_selection(i)) continue;
    indices.push_back(i);
    grow_box(geometry.box, Eigen::Vector3f(xyz[i * 3U], xyz[i * 3U + 1U], z));
  }
  if (z_window_enabled_ && geometry.box.valid) {
    geometry.box.min.z() = static_cast<float>(z_window_min_);
    geometry.box.max.z() = static_cast<float>(z_window_max_);
  }
  manager->select_points(indices, geometry);
  mark_edit_state_dirty();
}

void PointCloudViewer::rebuild_selection_box_from_points() {
  auto *manager = active_selection_manager();
  const auto &xyz = active_xyz();
  if (!manager || manager->statuses().size() != (xyz.size() / 3U)) return;
  SelectionGeometry geometry;
  geometry.rule_type = "remove_box";
  const auto &statuses = manager->statuses();
  for (std::size_t i = 0; i < statuses.size(); ++i) {
    if (statuses[i] != PointStatus::SELECTED) continue;
    grow_box(geometry.box, Eigen::Vector3f(xyz[i * 3U], xyz[i * 3U + 1U],
                                           xyz[i * 3U + 2U]));
  }
  manager->set_selection_geometry(geometry);
  mark_edit_state_dirty();
}

void PointCloudViewer::finish_polygon_selection() {
  const QPolygon polygon = pending_polygon_;
  pending_polygon_.clear();
  selecting_ = false;
  if (polygon.size() >= 3) select_screen_polygon(polygon);
  update();
}

void PointCloudViewer::select_screen_polygon(const QPolygon &polygon) {
  auto *manager = active_selection_manager();
  const auto &xyz = active_xyz();
  if (!manager || polygon.size() < 3 || xyz.empty() ||
      manager->statuses().size() != (xyz.size() / 3U)) {
    return;
  }
  const QMatrix4x4 mvp = camera_.projection_matrix() * camera_.view_matrix();
  std::vector<std::size_t> indices;
  SelectionGeometry geometry;
  geometry.rule_type = "remove_polygon";
  // Map-frame footprint: unproject each screen vertex onto the lowest cloud
  // height so the rule is reproducible outside this camera pose. This is
  // exact for the top view and an approximation for oblique views; the
  // exported rule therefore also carries the measured point AABB.
  const float ground_z = (confidence_mode() ? confidence_min_bound_ : cloud_.min_bound).z();
  bool footprint_ok = true;
  for (const QPoint &vertex : polygon) {
    const auto hit = unproject_to_ground(vertex, ground_z);
    if (!hit) {
      footprint_ok = false;
      break;
    }
    geometry.polygon_xy.emplace_back(hit->x(), hit->y());
  }
  for (std::size_t i = 0; i < (xyz.size() / 3U); ++i) {
    const float z = xyz[i * 3U + 2U];
    if (!passes_z_window(z)) continue;
    const auto screen = project(i, mvp);
    if (!screen || !polygon.containsPoint(*screen, Qt::OddEvenFill)) continue;
    if (!visible_for_selection(i)) continue;
    indices.push_back(i);
    grow_box(geometry.box, Eigen::Vector3f(xyz[i * 3U], xyz[i * 3U + 1U], z));
  }
  if (!footprint_ok || geometry.polygon_xy.size() < 3U) {
    geometry.rule_type = "remove_box";
    geometry.polygon_xy.clear();
  } else {
    geometry.has_z_range = true;
    geometry.z_min = z_window_enabled_ ? z_window_min_
                                       : (geometry.box.valid ? geometry.box.min.z() : 0.0);
    geometry.z_max = z_window_enabled_ ? z_window_max_
                                       : (geometry.box.valid ? geometry.box.max.z() : 0.0);
  }
  manager->select_points(indices, geometry);
  mark_edit_state_dirty();
}

void PointCloudViewer::select_height_band(double z_min, double z_max) {
  auto *manager = active_selection_manager();
  const auto &xyz = active_xyz();
  if (!manager || xyz.empty() ||
      manager->statuses().size() != (xyz.size() / 3U)) {
    return;
  }
  const double low = std::min(z_min, z_max);
  const double high = std::max(z_min, z_max);
  std::vector<std::size_t> indices;
  SelectionGeometry geometry;
  geometry.rule_type = "remove_height_band";
  geometry.z_min = low;
  geometry.z_max = high;
  geometry.has_z_range = true;
  geometry.box.min = (confidence_mode() ? confidence_min_bound_ : cloud_.min_bound);
  geometry.box.max = (confidence_mode() ? confidence_max_bound_ : cloud_.max_bound);
  geometry.box.min.z() = static_cast<float>(low);
  geometry.box.max.z() = static_cast<float>(high);
  geometry.box.valid = true;
  for (std::size_t i = 0; i < (xyz.size() / 3U); ++i) {
    const float z = xyz[i * 3U + 2U];
    if (z < low || z > high) continue;
    if (!visible_for_selection(i)) continue;
    indices.push_back(i);
  }
  manager->select_points(indices, geometry);
  mark_edit_state_dirty();
}

void PointCloudViewer::select_sphere_at(const QPoint &screen, double radius_m) {
  auto *manager = active_selection_manager();
  const auto &xyz = active_xyz();
  if (!manager || xyz.empty() || radius_m <= 0.0 ||
      manager->statuses().size() != (xyz.size() / 3U)) {
    return;
  }
  // Pick the nearest visible point under the cursor as the sphere centre.
  const QMatrix4x4 mvp = camera_.projection_matrix() * camera_.view_matrix();
  std::optional<std::size_t> best;
  int best_distance = 12 * 12;
  for (std::size_t i = 0; i < (xyz.size() / 3U); ++i) {
    if (!visible_for_selection(i)) continue;
    const auto projected = project(i, mvp);
    if (!projected) continue;
    const QPoint delta = *projected - screen;
    const int distance = delta.x() * delta.x() + delta.y() * delta.y();
    if (distance < best_distance) {
      best_distance = distance;
      best = i;
    }
  }
  if (!best) return;
  const Eigen::Vector3f center(xyz[*best * 3U], xyz[*best * 3U + 1U],
                               xyz[*best * 3U + 2U]);
  const float radius = static_cast<float>(radius_m);
  std::vector<std::size_t> indices;
  SelectionGeometry geometry;
  geometry.rule_type = "remove_sphere";
  geometry.center = center.cast<double>();
  geometry.radius = radius_m;
  geometry.box.min = (center.array() - radius).matrix();
  geometry.box.max = (center.array() + radius).matrix();
  geometry.box.valid = true;
  for (std::size_t i = 0; i < (xyz.size() / 3U); ++i) {
    if (!visible_for_selection(i)) continue;
    const Eigen::Vector3f point(xyz[i * 3U], xyz[i * 3U + 1U],
                                xyz[i * 3U + 2U]);
    if ((point - center).squaredNorm() <= radius * radius) indices.push_back(i);
  }
  manager->select_points(indices, geometry);
  mark_edit_state_dirty();
}

void PointCloudViewer::wheelEvent(QWheelEvent *event) {
  // High-resolution wheels and touchpads often report pixelDelta without an
  // angleDelta. Normalize pixels to wheel-like units so both devices zoom.
  float wheel_delta = static_cast<float>(event->angleDelta().y());
  if (qFuzzyIsNull(wheel_delta)) {
    wheel_delta = static_cast<float>(event->pixelDelta().y()) * 8.0F;
  }
  if (qFuzzyIsNull(wheel_delta)) {
    event->ignore();
    return;
  }

  // Keep the camera target on the visible height band. The source cloud may
  // extend far above a clipped greenhouse view, so its full-cloud center is
  // not a useful zoom target.
  const float anchor_z = zoom_anchor_height();
  camera_.focus_height(anchor_z);
  // Zoom around the fixed viewport center. Cursor-anchored zoom pans the
  // whole greenhouse as the pointer moves during a wheel gesture, making ROI
  // outlines appear to jump around.
  camera_.zoom(wheel_delta);
  update();
  event->accept();
}

}  // namespace agt_map_studio
