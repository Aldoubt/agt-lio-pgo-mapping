#pragma once

#include "occupancy/commands/GridCommand.hpp"

namespace agt_map_studio {

class DrawObstacleCommand : public GridCommand {
public:
  static std::unique_ptr<DrawObstacleCommand> create(
      const RefinementModel &model, GridWorldPoint start, GridWorldPoint end,
      double width_m);
  explicit DrawObstacleCommand(RefinementOperation operation)
      : operation_(std::move(operation)) {}

  void redo(RefinementModel &model) override;
  void undo(RefinementModel &model) override;
  const RefinementOperation &operation() const override { return operation_; }

private:
  RefinementOperation operation_;
};

}  // namespace agt_map_studio
