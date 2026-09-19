#include "occupancy/commands/DrawObstacleCommand.hpp"

#include "occupancy/RefinementModel.hpp"

#include <algorithm>
#include <cmath>

namespace agt_map_studio {
namespace {

double segment_distance(double px, double py, GridWorldPoint start,
                        GridWorldPoint end) {
  const double dx = end.x - start.x;
  const double dy = end.y - start.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 1.0e-12) {
    const double ex = px - start.x;
    const double ey = py - start.y;
    return std::sqrt(ex * ex + ey * ey);
  }
  const double t = std::clamp(((px - start.x) * dx + (py - start.y) * dy) /
                                  length_squared,
                              0.0, 1.0);
  const double ex = start.x + t * dx - px;
  const double ey = start.y + t * dy - py;
  return std::sqrt(ex * ex + ey * ey);
}

}  // namespace

std::unique_ptr<DrawObstacleCommand> DrawObstacleCommand::create(
    const RefinementModel &model, GridWorldPoint start, GridWorldPoint end,
    double width_m) {
  if (!model.has_map() || !(width_m > 0.0)) return nullptr;
  const auto &map = model.base_map();
  const double half_width = width_m * 0.5;
  const double min_x = std::min(start.x, end.x) - half_width;
  const double max_x = std::max(start.x, end.x) + half_width;
  const double min_y = std::min(start.y, end.y) - half_width;
  const double max_y = std::max(start.y, end.y) + half_width;
  const auto x0 = std::max(0, static_cast<int>(std::floor(
                             (min_x - map.origin_x()) / map.resolution())));
  const auto y0 = std::max(0, static_cast<int>(std::floor(
                             (min_y - map.origin_y()) / map.resolution())));
  const auto x1 = std::min(static_cast<int>(map.width()) - 1,
                           static_cast<int>(std::floor(
                               (max_x - map.origin_x()) / map.resolution())));
  const auto y1 = std::min(static_cast<int>(map.height()) - 1,
                           static_cast<int>(std::floor(
                               (max_y - map.origin_y()) / map.resolution())));
  if (x0 > x1 || y0 > y1) return nullptr;
  RefinementOperation operation;
  operation.id = model.next_operation_id();
  operation.type = "draw_obstacle";
  operation.timestamp = RefinementModel::timestamp_now();
  operation.geometry = {start, end};
  operation.width_m = width_m;
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const auto center = map.pixel_to_world(x, y);
      if (segment_distance(center.x, center.y, start, end) > half_width) continue;
      const std::size_t index = static_cast<std::size_t>(y) * map.width() + x;
      const auto before = model.effective_at_index(index);
      if (before == GridMap::kFree) {
        operation.changes.push_back({index, before, GridMap::kOccupied});
      }
    }
  }
  if (operation.changes.empty()) return nullptr;
  return std::make_unique<DrawObstacleCommand>(std::move(operation));
}

void DrawObstacleCommand::redo(RefinementModel &model) {
  model.apply_cell_changes(operation_.changes, true);
}

void DrawObstacleCommand::undo(RefinementModel &model) {
  model.apply_cell_changes(operation_.changes, false);
}

}  // namespace agt_map_studio
