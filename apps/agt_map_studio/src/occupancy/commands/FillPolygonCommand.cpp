#include "occupancy/commands/FillPolygonCommand.hpp"

#include "occupancy/RefinementModel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace agt_map_studio {

const char *FillPolygonCommand::type_for_value(std::int8_t value) {
  if (value == GridMap::kFree) return "fill_free_polygon";
  if (value == GridMap::kOccupied) return "fill_occupied_polygon";
  return "fill_unknown_polygon";
}

bool FillPolygonCommand::point_in_polygon(const std::vector<GridWorldPoint> &polygon,
                                          double x, double y) {
  bool inside = false;
  const std::size_t count = polygon.size();
  for (std::size_t i = 0, j = count - 1; i < count; j = i++) {
    const auto &a = polygon[i];
    const auto &b = polygon[j];
    const bool crosses = (a.y > y) != (b.y > y);
    if (!crosses) continue;
    const double x_at = (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x;
    if (x < x_at) inside = !inside;
  }
  return inside;
}

std::unique_ptr<FillPolygonCommand> FillPolygonCommand::create(
    const RefinementModel &model, const std::vector<GridWorldPoint> &polygon,
    std::int8_t value) {
  if (!model.has_map() || polygon.size() < 3U) return nullptr;
  const auto &map = model.base_map();
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  for (const auto &point : polygon) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  }
  const auto x0 = std::max(0, static_cast<int>(std::floor((min_x - map.origin_x()) / map.resolution())));
  const auto y0 = std::max(0, static_cast<int>(std::floor((min_y - map.origin_y()) / map.resolution())));
  const auto x1 = std::min(static_cast<int>(map.width()) - 1,
                           static_cast<int>(std::floor((max_x - map.origin_x()) / map.resolution())));
  const auto y1 = std::min(static_cast<int>(map.height()) - 1,
                           static_cast<int>(std::floor((max_y - map.origin_y()) / map.resolution())));
  if (x0 > x1 || y0 > y1) return nullptr;
  RefinementOperation operation;
  operation.id = model.next_operation_id();
  operation.type = type_for_value(value);
  operation.timestamp = RefinementModel::timestamp_now();
  operation.geometry = polygon;
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const GridWorldPoint center = map.pixel_to_world(x, y);
      if (!point_in_polygon(polygon, center.x, center.y)) continue;
      const std::size_t index = static_cast<std::size_t>(y) * map.width() + x;
      const auto before = model.effective_at_index(index);
      if (before != value) operation.changes.push_back({index, before, value});
    }
  }
  if (operation.changes.empty()) return nullptr;
  return std::make_unique<FillPolygonCommand>(std::move(operation));
}

void FillPolygonCommand::redo(RefinementModel &model) {
  model.apply_cell_changes(operation_.changes, true);
}

void FillPolygonCommand::undo(RefinementModel &model) {
  model.apply_cell_changes(operation_.changes, false);
}

}  // namespace agt_map_studio
