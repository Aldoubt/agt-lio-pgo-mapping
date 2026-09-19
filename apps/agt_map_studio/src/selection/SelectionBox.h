#pragma once

#include <QPoint>
#include <QRect>

namespace agt_map_studio {

class SelectionBox {
public:
  SelectionBox() = default;
  SelectionBox(const QPoint &start, const QPoint &end) : start_(start), end_(end) {}

  void set_start(const QPoint &point) { start_ = point; }
  void set_end(const QPoint &point) { end_ = point; }
  QRect rect() const { return QRect(start_, end_).normalized(); }
  bool is_valid() const { return rect().width() > 1 && rect().height() > 1; }
  bool contains(const QPoint &point) const { return rect().contains(point); }
  QPoint start() const { return start_; }
  QPoint end() const { return end_; }

private:
  QPoint start_;
  QPoint end_;
};

}  // namespace agt_map_studio
