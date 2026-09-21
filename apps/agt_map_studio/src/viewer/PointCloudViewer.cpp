#include "viewer/PointCloudViewer.hpp"

#include <QPolygonF>
#include <QVector2D>

#include <algorithm>

#include <QKeyEvent>
#include <QLinearGradient>
#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <utility>

namespace agt_map_studio {

PointCloudViewer::PointCloudViewer(QWidget *parent)
    : QOpenGLWidget(parent), cloud_buffer_(QOpenGLBuffer::VertexBuffer),
      status_buffer_(QOpenGLBuffer::VertexBuffer),
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
    axis_buffer_.destroy();
    doneCurrent();
  }
}

void PointCloudViewer::set_cloud(LoadedPointCloud cloud,
                                 const QString &filename) {
  cloud_ = std::move(cloud);
  filename_ = filename;
  status_buffer_dirty_ = true;
  if (has_cloud()) {
    reset_camera();
  }
  if (gl_ready_) {
    makeCurrent();
    upload_cloud();
    doneCurrent();
  }
  emit stats_changed(stats_text());
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
  height_coloring_ = enabled;
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

void PointCloudViewer::set_mode(InteractionMode mode) {
  mode_ = mode;
  selecting_ = false;
  pending_polygon_.clear();
  if (mode_ == InteractionMode::Navigate) selection_box_ = SelectionBox();
  emit stats_changed(stats_text());
  update();
}

void PointCloudViewer::set_selection_tool(SelectionTool tool) {
  tool_ = tool;
  selecting_ = false;
  pending_polygon_.clear();
  selection_box_ = SelectionBox();
  emit stats_changed(stats_text());
  update();
}

void PointCloudViewer::set_z_window(bool enabled, double z_min, double z_max) {
  z_window_enabled_ = enabled;
  z_window_min_ = std::min(z_min, z_max);
  z_window_max_ = std::max(z_min, z_max);
  emit stats_changed(stats_text());
}

bool PointCloudViewer::passes_z_window(float z) const {
  return !z_window_enabled_ || (z >= z_window_min_ && z <= z_window_max_);
}

void PointCloudViewer::cancel_pending_polygon() {
  pending_polygon_.clear();
  selecting_ = false;
  update();
}

void PointCloudViewer::mark_edit_state_dirty() {
  status_buffer_dirty_ = true;
  emit stats_changed(stats_text());
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
  if (has_cloud()) {
    const QVector3D min_bound(cloud_.min_bound.x(), cloud_.min_bound.y(),
                              cloud_.min_bound.z());
    const QVector3D max_bound(cloud_.max_bound.x(), cloud_.max_bound.y(),
                              cloud_.max_bound.z());
    camera_.reset(min_bound, max_bound);
  } else {
    camera_ = CameraController();
  }
  update();
}

QString PointCloudViewer::stats_text() const {
  const QString name = filename_.isEmpty() ? QStringLiteral("(none)")
                                           : filename_;
  const std::size_t deleted = selection_manager_ ? selection_manager_->deleted_count() : 0U;
  const std::size_t visible = selection_manager_ ? selection_manager_->visible_count()
                                                 : point_count();
  const std::size_t selected = selection_manager_ ? selection_manager_->selected_count() : 0U;
  QString text = QStringLiteral("File: %1 | Total: %2 | Selected: %3 | Deleted: %4 | Visible: %5 | Mode: %6")
      .arg(name)
      .arg(static_cast<qulonglong>(point_count()))
      .arg(static_cast<qulonglong>(selected))
      .arg(static_cast<qulonglong>(deleted))
      .arg(static_cast<qulonglong>(visible))
      .arg(mode_text());
  if (mode_ != InteractionMode::Navigate) text += QStringLiteral(" / %1").arg(tool_text());
  if (z_window_enabled_) {
    text += QStringLiteral(" | Z [%1, %2]").arg(z_window_min_, 0, 'f', 2).arg(z_window_max_, 0, 'f', 2);
  }
  return text + QStringLiteral(" | FPS: %1").arg(fps_, 0, 'f', 1);
}

QString PointCloudViewer::tool_text() const {
  switch (tool_) {
    case SelectionTool::PolygonPrism: return QStringLiteral("Polygon");
    case SelectionTool::Sphere: return QStringLiteral("Sphere %1m").arg(sphere_radius_, 0, 'f', 2);
    default: return QStringLiteral("Rect");
  }
}

QString PointCloudViewer::mode_text() const {
  switch (mode_) {
    case InteractionMode::Select: return QStringLiteral("Select");
    case InteractionMode::Delete: return QStringLiteral("Delete");
    default: return QStringLiteral("Navigate");
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
  emitter << YAML::Key << "color_mode" << YAML::Value
          << (height_coloring_ ? "height" : "solid");
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
    uniform mat4 u_mvp;
    uniform float u_point_size;
    varying float v_height;
    varying float v_status;
    void main() {
      gl_Position = u_mvp * vec4(a_position, 1.0);
      gl_PointSize = u_point_size;
      v_height = a_position.z;
      v_status = a_status;
    }
  )glsl";
  const char *fragment_shader = R"glsl(
    uniform vec4 u_color;
    uniform int u_height_coloring;
    uniform float u_z_min;
    uniform float u_z_max;
    varying float v_height;
    varying float v_status;

    vec3 height_color(float value) {
      float range = max(u_z_max - u_z_min, 0.000001);
      float t = clamp((value - u_z_min) / range, 0.0, 1.0);
      if (t < 0.25) return mix(vec3(0.18, 0.24, 0.86), vec3(0.0, 0.75, 0.86), t / 0.25);
      if (t < 0.5) return mix(vec3(0.0, 0.75, 0.86), vec3(0.15, 0.75, 0.30), (t - 0.25) / 0.25);
      if (t < 0.75) return mix(vec3(0.15, 0.75, 0.30), vec3(0.95, 0.80, 0.12), (t - 0.5) / 0.25);
      return mix(vec3(0.95, 0.80, 0.12), vec3(0.86, 0.10, 0.10), (t - 0.75) / 0.25);
    }

    void main() {
      if (v_status > 2.5) {
        discard;
      } else if (v_status > 1.5) {
        gl_FragColor = vec4(0.9, 0.05, 0.05, 0.85);
      } else if (v_status > 0.5) {
        gl_FragColor = vec4(1.0, 0.75, 0.05, 1.0);
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
  axis_buffer_.create();
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
  if (status_buffer_dirty_) upload_statuses();
  if (shader_) {
    shader_->bind();
    shader_->setUniformValue("u_mvp", mvp);
    shader_->setUniformValue("u_point_size", point_size_);
    shader_->setUniformValue("u_height_coloring", height_coloring_ ? 1 : 0);
    shader_->setUniformValue("u_z_min", cloud_.min_bound.z());
    shader_->setUniformValue("u_z_max", cloud_.max_bound.z());
    shader_->setUniformValue(
        "u_color", dark_background_ ? QVector4D(1.0F, 1.0F, 1.0F, 1.0F)
                                     : QVector4D(0.12F, 0.12F, 0.12F, 1.0F));
    if (cloud_.xyz.empty() == false && cloud_buffer_.isCreated() &&
        status_buffer_.isCreated()) {
      cloud_buffer_.bind();
      shader_->enableAttributeArray("a_position");
      shader_->setAttributeBuffer("a_position", GL_FLOAT, 0, 3);
      cloud_buffer_.release();
      status_buffer_.bind();
      shader_->enableAttributeArray("a_status");
      shader_->setAttributeBuffer("a_status", GL_FLOAT, 0, 1);
      status_buffer_.release();
      cloud_buffer_.bind();
      glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(cloud_.point_count()));
      shader_->disableAttributeArray("a_position");
      shader_->disableAttributeArray("a_status");
      cloud_buffer_.release();
    }
    if (show_axis_ && axis_buffer_.isCreated()) {
      draw_axes(mvp);
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
    emit stats_changed(stats_text());
  }

  QPainter painter(this);
  painter.setPen(dark_background_ ? Qt::white : Qt::black);
  painter.drawText(12, 22, stats_text());
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
  if (height_coloring_ && has_cloud()) {
    const int legend_width = 180;
    const int legend_height = 12;
    const int legend_x = std::max(12, width() - legend_width - 18);
    const int legend_y = 14;
    QLinearGradient gradient(legend_x, legend_y,
                              legend_x + legend_width, legend_y);
    gradient.setColorAt(0.0, QColor(45, 60, 220));
    gradient.setColorAt(0.25, QColor(0, 190, 220));
    gradient.setColorAt(0.5, QColor(40, 190, 80));
    gradient.setColorAt(0.75, QColor(245, 210, 35));
    gradient.setColorAt(1.0, QColor(220, 35, 35));
    painter.fillRect(legend_x, legend_y, legend_width, legend_height, gradient);
    painter.drawRect(legend_x, legend_y, legend_width, legend_height);
    painter.drawText(legend_x, legend_y + 30,
                     QStringLiteral("Z low: %1").arg(cloud_.min_bound.z(), 0, 'f', 2));
    painter.drawText(legend_x + 105, legend_y + 30,
                     QStringLiteral("high: %1").arg(cloud_.max_bound.z(), 0, 'f', 2));
  }
  painter.end();
}

void PointCloudViewer::draw_axes(const QMatrix4x4 &mvp) {
  shader_->setUniformValue("u_mvp", mvp);
  shader_->setUniformValue("u_point_size", 1.0F);
  shader_->setUniformValue("u_height_coloring", 0);
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

void PointCloudViewer::tick() {
  camera_.update(0.016F);
  update();
}

void PointCloudViewer::keyPressEvent(QKeyEvent *event) {
  // Mode switching is owned by MainWindow (toolbar + F1/F2/F3), so the WASD
  // camera keys are never shadowed here.
  if (event->key() == Qt::Key_Escape) {
    cancel_pending_polygon();
    if (selection_manager_) {
      selection_manager_->clear_selection();
      mark_edit_state_dirty();
    }
    event->accept();
    return;
  }
  if ((event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) &&
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
  if (event->button() == Qt::LeftButton && selecting_ && tool_ == SelectionTool::ScreenRect) {
    selecting_ = false;
    if (selection_box_.is_valid()) select_screen_rect(selection_box_);
  }
  if (event->button() == Qt::LeftButton) left_drag_ = false;
  if (event->button() == Qt::RightButton) right_drag_ = false;
  event->accept();
}

std::optional<QPoint> PointCloudViewer::project(std::size_t i, const QMatrix4x4 &mvp) const {
  const QVector4D clip(mvp * QVector4D(cloud_.xyz[i * 3U], cloud_.xyz[i * 3U + 1U],
                                       cloud_.xyz[i * 3U + 2U], 1.0F));
  if (clip.w() <= 0.0F) return std::nullopt;
  const QVector3D ndc = clip.toVector3DAffine();
  return QPoint(qRound((ndc.x() + 1.0F) * 0.5F * width()),
                qRound((1.0F - ndc.y()) * 0.5F * height()));
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
  if (!selection_manager_ || !box.is_valid() || cloud_.xyz.empty() ||
      selection_manager_->statuses().size() != cloud_.point_count()) {
    return;
  }
  const QMatrix4x4 mvp = camera_.projection_matrix() * camera_.view_matrix();
  std::vector<std::size_t> indices;
  SelectionGeometry geometry;
  geometry.rule_type = "remove_box";
  for (std::size_t i = 0; i < cloud_.point_count(); ++i) {
    const float z = cloud_.xyz[i * 3U + 2U];
    if (!passes_z_window(z)) continue;
    const auto screen = project(i, mvp);
    if (!screen || !box.contains(*screen)) continue;
    if (selection_manager_->statuses()[i] == PointStatus::DELETED) continue;
    indices.push_back(i);
    grow_box(geometry.box, Eigen::Vector3f(cloud_.xyz[i * 3U], cloud_.xyz[i * 3U + 1U], z));
  }
  if (z_window_enabled_ && geometry.box.valid) {
    geometry.box.min.z() = static_cast<float>(z_window_min_);
    geometry.box.max.z() = static_cast<float>(z_window_max_);
  }
  selection_manager_->select_points(indices, geometry);
  mark_edit_state_dirty();
}

void PointCloudViewer::rebuild_selection_box_from_points() {
  if (!selection_manager_ || selection_manager_->statuses().size() != cloud_.point_count()) return;
  SelectionGeometry geometry;
  geometry.rule_type = "remove_box";
  const auto &statuses = selection_manager_->statuses();
  for (std::size_t i = 0; i < statuses.size(); ++i) {
    if (statuses[i] != PointStatus::SELECTED) continue;
    grow_box(geometry.box, Eigen::Vector3f(cloud_.xyz[i * 3U], cloud_.xyz[i * 3U + 1U],
                                           cloud_.xyz[i * 3U + 2U]));
  }
  selection_manager_->set_selection_geometry(geometry);
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
  if (!selection_manager_ || polygon.size() < 3 || cloud_.xyz.empty() ||
      selection_manager_->statuses().size() != cloud_.point_count()) {
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
  const float ground_z = cloud_.min_bound.z();
  bool footprint_ok = true;
  for (const QPoint &vertex : polygon) {
    const auto hit = unproject_to_ground(vertex, ground_z);
    if (!hit) {
      footprint_ok = false;
      break;
    }
    geometry.polygon_xy.emplace_back(hit->x(), hit->y());
  }
  for (std::size_t i = 0; i < cloud_.point_count(); ++i) {
    const float z = cloud_.xyz[i * 3U + 2U];
    if (!passes_z_window(z)) continue;
    const auto screen = project(i, mvp);
    if (!screen || !polygon.containsPoint(*screen, Qt::OddEvenFill)) continue;
    if (selection_manager_->statuses()[i] == PointStatus::DELETED) continue;
    indices.push_back(i);
    grow_box(geometry.box, Eigen::Vector3f(cloud_.xyz[i * 3U], cloud_.xyz[i * 3U + 1U], z));
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
  selection_manager_->select_points(indices, geometry);
  mark_edit_state_dirty();
}

void PointCloudViewer::select_height_band(double z_min, double z_max) {
  if (!selection_manager_ || cloud_.xyz.empty() ||
      selection_manager_->statuses().size() != cloud_.point_count()) {
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
  geometry.box.min = cloud_.min_bound;
  geometry.box.max = cloud_.max_bound;
  geometry.box.min.z() = static_cast<float>(low);
  geometry.box.max.z() = static_cast<float>(high);
  geometry.box.valid = true;
  for (std::size_t i = 0; i < cloud_.point_count(); ++i) {
    const float z = cloud_.xyz[i * 3U + 2U];
    if (z < low || z > high) continue;
    if (selection_manager_->statuses()[i] == PointStatus::DELETED) continue;
    indices.push_back(i);
  }
  selection_manager_->select_points(indices, geometry);
  mark_edit_state_dirty();
}

void PointCloudViewer::select_sphere_at(const QPoint &screen, double radius_m) {
  if (!selection_manager_ || cloud_.xyz.empty() || radius_m <= 0.0 ||
      selection_manager_->statuses().size() != cloud_.point_count()) {
    return;
  }
  // Pick the nearest visible point under the cursor as the sphere centre.
  const QMatrix4x4 mvp = camera_.projection_matrix() * camera_.view_matrix();
  std::optional<std::size_t> best;
  int best_distance = 12 * 12;
  for (std::size_t i = 0; i < cloud_.point_count(); ++i) {
    if (selection_manager_->statuses()[i] == PointStatus::DELETED) continue;
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
  const Eigen::Vector3f center(cloud_.xyz[*best * 3U], cloud_.xyz[*best * 3U + 1U],
                               cloud_.xyz[*best * 3U + 2U]);
  const float radius = static_cast<float>(radius_m);
  std::vector<std::size_t> indices;
  SelectionGeometry geometry;
  geometry.rule_type = "remove_sphere";
  geometry.center = center.cast<double>();
  geometry.radius = radius_m;
  geometry.box.min = (center.array() - radius).matrix();
  geometry.box.max = (center.array() + radius).matrix();
  geometry.box.valid = true;
  for (std::size_t i = 0; i < cloud_.point_count(); ++i) {
    if (selection_manager_->statuses()[i] == PointStatus::DELETED) continue;
    const Eigen::Vector3f point(cloud_.xyz[i * 3U], cloud_.xyz[i * 3U + 1U],
                                cloud_.xyz[i * 3U + 2U]);
    if ((point - center).squaredNorm() <= radius * radius) indices.push_back(i);
  }
  selection_manager_->select_points(indices, geometry);
  mark_edit_state_dirty();
}

void PointCloudViewer::wheelEvent(QWheelEvent *event) {
  camera_.zoom(static_cast<float>(event->angleDelta().y()));
  update();
  event->accept();
}

}  // namespace agt_map_studio
