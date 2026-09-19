#include "viewer/PointCloudViewer.hpp"

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
  if (mode_ == InteractionMode::Navigate) selection_box_ = SelectionBox();
  emit stats_changed(stats_text());
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
  return QStringLiteral("File: %1 | Total: %2 | Deleted: %3 | Visible: %4 | Mode: %5 | FPS: %6")
      .arg(name)
      .arg(static_cast<qulonglong>(point_count()))
      .arg(static_cast<qulonglong>(deleted))
      .arg(static_cast<qulonglong>(visible))
      .arg(mode_text())
      .arg(fps_, 0, 'f', 1);
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
      if (v_status > 1.5) {
        gl_FragColor = vec4(0.9, 0.05, 0.05, 1.0);
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
  if (selecting_ && selection_box_.is_valid()) {
    QPen pen(QColor(30, 120, 255), 2, Qt::DashLine);
    painter.setPen(pen);
    painter.setBrush(QColor(50, 140, 255, 35));
    painter.drawRect(selection_box_.rect());
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
    for (std::size_t i = 0; i < statuses.size(); ++i) {
      values[i] = static_cast<float>(statuses[i]);
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
  if (event->key() == Qt::Key_N) {
    set_mode(InteractionMode::Navigate);
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_S) {
    set_mode(InteractionMode::Select);
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_D) {
    set_mode(InteractionMode::Delete);
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
  if (event->key() == Qt::Key_Delete && selection_manager_ &&
      mode_ == InteractionMode::Delete) {
    if (selection_manager_->delete_selected()) mark_edit_state_dirty();
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
    selecting_ = true;
    selection_box_.set_start(event->pos());
    selection_box_.set_end(event->pos());
  }
  event->accept();
}

void PointCloudViewer::mouseMoveEvent(QMouseEvent *event) {
  const QPoint delta = event->pos() - last_mouse_position_;
  last_mouse_position_ = event->pos();
  if (selecting_) {
    selection_box_.set_end(event->pos());
  } else if (left_drag_) {
    camera_.orbit(delta.x(), delta.y());
  }
  if (right_drag_) camera_.pan(delta.x(), delta.y());
  update();
  event->accept();
}

void PointCloudViewer::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton && selecting_) {
    selecting_ = false;
    if (selection_box_.is_valid()) select_screen_rect(selection_box_);
  }
  if (event->button() == Qt::LeftButton) left_drag_ = false;
  if (event->button() == Qt::RightButton) right_drag_ = false;
  event->accept();
}

void PointCloudViewer::select_screen_rect(const SelectionBox &box) {
  if (!selection_manager_ || !box.is_valid() || cloud_.xyz.empty() ||
      selection_manager_->statuses().size() != cloud_.point_count()) {
    return;
  }
  const QMatrix4x4 mvp = camera_.projection_matrix() * camera_.view_matrix();
  std::vector<std::size_t> indices;
  AxisAlignedBoundingBox bounds;
  for (std::size_t i = 0; i < cloud_.point_count(); ++i) {
    const QVector4D clip(mvp * QVector4D(cloud_.xyz[i * 3U], cloud_.xyz[i * 3U + 1U],
                                         cloud_.xyz[i * 3U + 2U], 1.0F));
    if (clip.w() <= 0.0F) continue;
    const QVector3D ndc = clip.toVector3DAffine();
    const QPoint screen(qRound((ndc.x() + 1.0F) * 0.5F * width()),
                        qRound((1.0F - ndc.y()) * 0.5F * height()));
    if (!box.contains(screen)) continue;
    if (selection_manager_->statuses()[i] == PointStatus::DELETED) continue;
    indices.push_back(i);
    const Eigen::Vector3f point(cloud_.xyz[i * 3U], cloud_.xyz[i * 3U + 1U],
                                cloud_.xyz[i * 3U + 2U]);
    if (!bounds.valid) {
      bounds.min = point;
      bounds.max = point;
      bounds.valid = true;
    } else {
      bounds.min = bounds.min.cwiseMin(point);
      bounds.max = bounds.max.cwiseMax(point);
    }
  }
  selection_manager_->select_points(indices, bounds);
  mark_edit_state_dirty();
}

void PointCloudViewer::wheelEvent(QWheelEvent *event) {
  camera_.zoom(static_cast<float>(event->angleDelta().y()));
  update();
  event->accept();
}

}  // namespace agt_map_studio
