#include "occupancy/commands/ForbiddenPolygonCommand.hpp"

#include "occupancy/RefinementModel.hpp"

namespace agt_map_studio {

std::unique_ptr<ForbiddenPolygonCommand> ForbiddenPolygonCommand::create(
    const RefinementModel &model, std::vector<GridWorldPoint> polygon) {
  if (!model.has_map() || polygon.size() < 3U) return nullptr;
  RefinementOperation operation;
  operation.id = model.next_operation_id();
  operation.type = "forbidden_polygon";
  operation.timestamp = RefinementModel::timestamp_now();
  operation.geometry = std::move(polygon);
  return std::make_unique<ForbiddenPolygonCommand>(std::move(operation));
}

void ForbiddenPolygonCommand::redo(RefinementModel &model) {
  model.add_forbidden_zone(operation_.id, operation_.geometry);
}

void ForbiddenPolygonCommand::undo(RefinementModel &model) {
  model.remove_forbidden_zone(operation_.id);
}

}  // namespace agt_map_studio
