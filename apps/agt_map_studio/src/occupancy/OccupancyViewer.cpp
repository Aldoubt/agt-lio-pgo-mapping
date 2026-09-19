#include "occupancy/OccupancyViewer.hpp"

#include "occupancy/RefinementModel.hpp"

#include <QKeyEvent>
#include <QPainter>
#include <QResizeEvent>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace agt_map_studio {

OccupancyViewer::OccupancyViewer(QWidget *parent) : QWidget(parent) {
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  setMinimumSize(320, 240);
}

void OccupancyViewer::set_refinement_model(RefinementModel *model) {
  refinement_model_ = model;
  refresh();
}

void OccupancyViewer::set_map(GridMap map) {
  map_ = std::move(map);
  rebuild_image();
  fit_map();
  cursor_status_.clear();
  emit status_changed(QStringLiteral("2D map: %1 x %2 | resolution: %3 m")
                          .arg(map_.width())
                          .arg(map_.height())
                          .arg(map_.resolution(), 0, 'f', 3));
  update();
}

void OccupancyViewer::clear_map() {
  map_.clear();
  image_ = QImage();
  cursor_status_.clear();
  update();
}

void OccupancyViewer::refresh() {
  rebuild_image();
  update();
}

void OccupancyViewer::set_mode(OccupancyInteractionMode mode) {
  mode_ = mode;
  editing_drag_ = false;
  if (mode_ != OccupancyInteractionMode::Forbidden) forbidden_polygon_world_.clear();
  emit status_changed(QStringLiteral("2D mode: %1").arg(
      mode_ == OccupancyInteractionMode::View
          ? QStringLiteral("View")
          : mode_ == OccupancyInteractionMode::Erase
                ? QStringLiteral("Erase")
                : mode_ == OccupancyInteractionMode::Obstacle
                      ? QStringLiteral("Obstacle")
                      : QStringLiteral("Forbidden")));
  update();
}

void OccupancyViewer::set_obstacle_width(double width_m) {
  if (width_m > 0.0) obstacle_width_m_ = width_m;
  update();
}

void OccupancyViewer::rebuild_image() {
  image_ = QImage();
  if (map_.empty() || map_.width() > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      map_.height() > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    return;
  }
  image_ = QImage(static_cast<int>(map_.width()), static_cast<int>(map_.height()),
                  QImage::Format_Grayscale8);
  for (std::uint32_t grid_y = 0; grid_y < map_.height(); ++grid_y) {
    auto *line = image_.scanLine(static_cast<int>(map_.height() - 1U - grid_y));
    for (std::uint32_t x = 0; x < map_.width(); ++x) {
      const std::int8_t value = refinement_model_
                                    ? refinement_model_->effective_at(x, grid_y)
                                    : map_.at(x, grid_y);
      line[x] = value == GridMap::kOccupied
                    ? 0U
                    : (value == GridMap::kFree ? 255U : 160U);
    }
  }
}

void OccupancyViewer::reset_view() {
  zoom_ = 1.0;
  pan_ = QPointF((width() - image_.width()) * 0.5,
                 (height() - image_.height()) * 0.5);
  update();
}

void OccupancyViewer::fit_map() {
  if (image_.isNull() || width() <= 0 || height() <= 0) return;
  const double margin = 20.0;
  zoom_ = std::clamp(std::min((width() - margin) / image_.width(),
                              (height() - margin) / image_.height()),
                     0.05, 50.0);
  pan_ = QPointF((width() - image_.width() * zoom_) * 0.5,
                 (height() - image_.height() * zoom_) * 0.5);
  update();
}

void OccupancyViewer::paintEvent(QPaintEvent *) {
  QPainter painter(this);
  painter.fillRect(rect(), QColor(42, 48, 56));
  if (!image_.isNull()) {
    painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
    painter.save();
    painter.translate(pan_);
    painter.scale(zoom_, zoom_);
    painter.drawImage(QPointF(0.0, 0.0), image_);
    painter.restore();
  } else {
    painter.setPen(Qt::white);
    painter.drawText(rect(), Qt::AlignCenter,
                     QStringLiteral("Open a Nav2 map.yaml to view occupancy map"));
  }
  painter.setPen(Qt::white);
  const QString dimensions = map_.empty()
                                 ? QStringLiteral("No map")
                                 : QStringLiteral("Map: %1 x %2 | %3 m/cell | zoom %4x")
                                       .arg(map_.width())
                                       .arg(map_.height())
                                       .arg(map_.resolution(), 0, 'f', 3)
                                       .arg(zoom_, 0, 'f', 2);
  painter.fillRect(8, 8, std::min(width() - 16, 360), 24,
                   QColor(0, 0, 0, 150));
  painter.drawText(14, 25, dimensions);
  if (!cursor_status_.isEmpty()) {
    painter.fillRect(8, height() - 34, std::min(width() - 16, 620), 24,
                     QColor(0, 0, 0, 170));
    painter.drawText(14, height() - 17, cursor_status_);
  }
  if (!image_.isNull() && refinement_model_) {
    painter.save();
    painter.setPen(QPen(QColor(220, 30, 30, 220), 2));
    painter.setBrush(QColor(230, 30, 30, 65));
    for (const auto &zone : refinement_model_->forbidden_zones()) {
      QPolygonF polygon;
      for (const auto &point : zone.polygon) polygon << world_to_screen(point);
      if (polygon.size() >= 3) painter.drawPolygon(polygon);
    }
    painter.setPen(QPen(QColor(230, 30, 30, 230), 2, Qt::DashLine));
    painter.setBrush(QColor(230, 30, 30, 45));
    if (mode_ == OccupancyInteractionMode::Erase && editing_drag_) {
      painter.drawRect(QRect(edit_start_screen_, edit_current_screen_).normalized());
    } else if (mode_ == OccupancyInteractionMode::Obstacle && editing_drag_) {
      const double pixel_width = std::max(2.0, obstacle_width_m_ /
                                                    map_.resolution() * zoom_);
      painter.setPen(QPen(QColor(40, 110, 240, 230), pixel_width,
                          Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
      painter.drawLine(edit_start_screen_, edit_current_screen_);
    } else if (mode_ == OccupancyInteractionMode::Forbidden &&
               !forbidden_polygon_world_.isEmpty()) {
      QPolygonF polygon;
      for (const auto &point : forbidden_polygon_world_) {
        polygon << world_to_screen({point.x(), point.y()});
      }
      painter.setPen(QPen(QColor(240, 30, 30, 230), 2, Qt::DashLine));
      painter.setBrush(QColor(240, 30, 30, 65));
      painter.drawPolyline(polygon);
      if (polygon.size() >= 3) painter.drawLine(polygon.last(), polygon.first());
    }
    painter.restore();
  }
}

void OccupancyViewer::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
  if (!has_map()) return;
  if (event->oldSize().isEmpty()) fit_map();
  update();
}

void OccupancyViewer::keyPressEvent(QKeyEvent *event) {
  if (event->key() == Qt::Key_Escape) {
    editing_drag_ = false;
    forbidden_polygon_world_.clear();
    update();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_R) {
    reset_view();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_F) {
    fit_map();
    event->accept();
    return;
  }
  QWidget::keyPressEvent(event);
}

void OccupancyViewer::mousePressEvent(QMouseEvent *event) {
  setFocus();
  if (event->button() == Qt::LeftButton && mode_ == OccupancyInteractionMode::View) {
    panning_ = true;
    last_mouse_position_ = event->pos();
  } else if (event->button() == Qt::LeftButton &&
             mode_ == OccupancyInteractionMode::Forbidden) {
    GridWorldPoint world;
    if (screen_to_world(QPointF(event->pos()), &world)) {
      forbidden_polygon_world_.push_back(QPointF(world.x, world.y));
      update();
    }
  } else if (event->button() == Qt::LeftButton) {
    GridWorldPoint world;
    if (screen_to_world(QPointF(event->pos()), &world)) {
      editing_drag_ = true;
      edit_start_screen_ = event->pos();
      edit_current_screen_ = event->pos();
    }
  }
  event->accept();
}

void OccupancyViewer::mouseMoveEvent(QMouseEvent *event) {
  if (panning_) {
    pan_ += QPointF(event->pos() - last_mouse_position_);
    last_mouse_position_ = event->pos();
    update();
  } else if (editing_drag_) {
    edit_current_screen_ = event->pos();
    update();
  }
  update_cursor_status(QPointF(event->pos()));
  event->accept();
}

void OccupancyViewer::mouseReleaseEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton && panning_) panning_ = false;
  if (event->button() == Qt::LeftButton && editing_drag_) {
    edit_current_screen_ = event->pos();
    GridWorldPoint first;
    GridWorldPoint second;
    const bool valid = screen_to_world(QPointF(edit_start_screen_), &first) &&
                       screen_to_world(QPointF(edit_current_screen_), &second);
    editing_drag_ = false;
    if (valid && mode_ == OccupancyInteractionMode::Erase) {
      emit erase_rectangle_requested(std::min(first.x, second.x),
                                     std::min(first.y, second.y),
                                     std::max(first.x, second.x),
                                     std::max(first.y, second.y));
    } else if (valid && mode_ == OccupancyInteractionMode::Obstacle &&
               edit_start_screen_ != edit_current_screen_) {
      emit obstacle_line_requested(first.x, first.y, second.x, second.y,
                                   obstacle_width_m_);
    }
    update();
  }
  event->accept();
}

void OccupancyViewer::mouseDoubleClickEvent(QMouseEvent *event) {
  if (event->button() == Qt::LeftButton &&
      mode_ == OccupancyInteractionMode::Forbidden) {
    finish_forbidden_polygon();
    event->accept();
    return;
  }
  QWidget::mouseDoubleClickEvent(event);
}

void OccupancyViewer::wheelEvent(QWheelEvent *event) {
  if (image_.isNull()) return;
  const QPointF cursor(event->pos());
  const QPointF image_before = (cursor - pan_) / zoom_;
  const double steps = event->angleDelta().y() / 120.0;
  zoom_ = std::clamp(zoom_ * std::pow(1.15, steps), 0.05, 50.0);
  pan_ = cursor - image_before * zoom_;
  update_cursor_status(cursor);
  update();
  event->accept();
}

void OccupancyViewer::update_cursor_status(const QPointF &position) {
  if (map_.empty() || zoom_ <= 0.0) return;
  const QPointF image_position = (position - pan_) / zoom_;
  const int image_x = static_cast<int>(std::floor(image_position.x()));
  const int image_y = static_cast<int>(std::floor(image_position.y()));
  if (image_x < 0 || image_y < 0 || image_x >= static_cast<int>(map_.width()) ||
      image_y >= static_cast<int>(map_.height())) {
    cursor_status_.clear();
    return;
  }
  const int grid_y = static_cast<int>(map_.height()) - 1 - image_y;
  const GridWorldPoint world = map_.pixel_to_world(image_x, grid_y);
  const std::int8_t occupancy = refinement_model_
                                    ? refinement_model_->effective_at(
                                          static_cast<std::uint32_t>(image_x),
                                          static_cast<std::uint32_t>(grid_y))
                                    : map_.at(static_cast<std::uint32_t>(image_x),
                                              static_cast<std::uint32_t>(grid_y));
  cursor_status_ = QStringLiteral("pixel=(%1,%2) world=(%3,%4) occupancy=%5")
                       .arg(image_x)
                       .arg(grid_y)
                       .arg(world.x, 0, 'f', 3)
                       .arg(world.y, 0, 'f', 3)
                       .arg(occupancy);
  emit status_changed(cursor_status_);
  update();
}

bool OccupancyViewer::screen_to_world(const QPointF &position,
                                      GridWorldPoint *world) const {
  if (!world || map_.empty() || zoom_ <= 0.0) return false;
  const QPointF image_position = (position - pan_) / zoom_;
  const int image_x = static_cast<int>(std::floor(image_position.x()));
  const int image_y = static_cast<int>(std::floor(image_position.y()));
  if (image_x < 0 || image_y < 0 || image_x >= static_cast<int>(map_.width()) ||
      image_y >= static_cast<int>(map_.height())) {
    return false;
  }
  const int grid_y = static_cast<int>(map_.height()) - 1 - image_y;
  const auto converted = map_.pixel_to_world(image_x, grid_y);
  *world = converted;
  return true;
}

QPointF OccupancyViewer::world_to_screen(const GridWorldPoint &world) const {
  const double grid_x = (world.x - map_.origin_x()) / map_.resolution();
  const double grid_y = (world.y - map_.origin_y()) / map_.resolution();
  const double image_y = static_cast<double>(map_.height()) - 1.0 - grid_y;
  return pan_ + QPointF(grid_x, image_y) * zoom_;
}

void OccupancyViewer::finish_forbidden_polygon() {
  if (forbidden_polygon_world_.size() >= 3U) {
    if (forbidden_polygon_world_.size() >= 2U &&
        forbidden_polygon_world_.back() == forbidden_polygon_world_[
                                                   forbidden_polygon_world_.size() - 2U]) {
      forbidden_polygon_world_.removeLast();
    }
    if (forbidden_polygon_world_.size() >= 3U) {
      emit forbidden_polygon_requested(forbidden_polygon_world_);
    }
  }
  forbidden_polygon_world_.clear();
  update();
}

}  // namespace agt_map_studio
