#pragma once

#include "occupancy/commands/GridCommand.hpp"

namespace agt_map_studio {

class ForbiddenPolygonCommand : public GridCommand {
public:
  static std::unique_ptr<ForbiddenPolygonCommand> create(
      const RefinementModel &model, std::vector<GridWorldPoint> polygon);
  explicit ForbiddenPolygonCommand(RefinementOperation operation)
      : operation_(std::move(operation)) {}

  void redo(RefinementModel &model) override;
  void undo(RefinementModel &model) override;
  const RefinementOperation &operation() const override { return operation_; }

private:
  RefinementOperation operation_;
};

}  // namespace agt_map_studio
