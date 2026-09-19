#include "occupancy/commands/EraseRectangleCommand.hpp"

#include "occupancy/RefinementModel.hpp"

#include <algorithm>
#include <cmath>

namespace agt_map_studio {

std::unique_ptr<EraseRectangleCommand> EraseRectangleCommand::create(
    const RefinementModel &model, GridWorldPoint first, GridWorldPoint second) {
  if (!model.has_map()) return nullptr;
  const double min_x = std::min(first.x, second.x);
  const double max_x = std::max(first.x, second.x);
  const double min_y = std::min(first.y, second.y);
  const double max_y = std::max(first.y, second.y);
  const auto &map = model.base_map();
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
  operation.type = "erase_rectangle";
  operation.timestamp = RefinementModel::timestamp_now();
  operation.geometry = {{min_x, min_y}, {max_x, max_y}};
  for (int y = y0; y <= y1; ++y) {
    for (int x = x0; x <= x1; ++x) {
      const std::size_t index = static_cast<std::size_t>(y) * map.width() + x;
      const auto before = model.effective_at_index(index);
      if (before == GridMap::kOccupied) {
        operation.changes.push_back({index, before, GridMap::kFree});
      }
    }
  }
  if (operation.changes.empty()) return nullptr;
  return std::make_unique<EraseRectangleCommand>(std::move(operation));
}

void EraseRectangleCommand::redo(RefinementModel &model) {
  model.apply_cell_changes(operation_.changes, true);
}

void EraseRectangleCommand::undo(RefinementModel &model) {
  model.apply_cell_changes(operation_.changes, false);
}

}  // namespace agt_map_studio
