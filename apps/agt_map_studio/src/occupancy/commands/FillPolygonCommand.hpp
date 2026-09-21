#pragma once

#include "occupancy/commands/GridCommand.hpp"

namespace agt_map_studio {

// Sets every cell inside a world-frame polygon to `value` (free / occupied /
// unknown). Serialises 1:1 to a `patch_nav_map` edit with `polygon_m`.
class FillPolygonCommand : public GridCommand {
public:
  static std::unique_ptr<FillPolygonCommand> create(
      const RefinementModel &model, const std::vector<GridWorldPoint> &polygon,
      std::int8_t value);
  explicit FillPolygonCommand(RefinementOperation operation)
      : operation_(std::move(operation)) {}

  static const char *type_for_value(std::int8_t value);
  static bool point_in_polygon(const std::vector<GridWorldPoint> &polygon, double x, double y);

  void redo(RefinementModel &model) override;
  void undo(RefinementModel &model) override;
  const RefinementOperation &operation() const override { return operation_; }

private:
  RefinementOperation operation_;
};

}  // namespace agt_map_studio
