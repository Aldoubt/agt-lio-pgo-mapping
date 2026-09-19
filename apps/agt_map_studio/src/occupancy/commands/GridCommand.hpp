#pragma once

#include "occupancy/GridMap.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace agt_map_studio {

class RefinementModel;

struct CellChange {
  std::size_t index = 0;
  std::int8_t before = GridMap::kUnknown;
  std::int8_t after = GridMap::kUnknown;
};

struct RefinementOperation {
  std::size_t id = 0;
  std::string type;
  std::string timestamp;
  std::vector<GridWorldPoint> geometry;
  double width_m = 0.0;
  std::vector<CellChange> changes;
  bool undone = false;
};

class GridCommand {
public:
  virtual ~GridCommand() = default;
  virtual void redo(RefinementModel &model) = 0;
  virtual void undo(RefinementModel &model) = 0;
  virtual const RefinementOperation &operation() const = 0;
};

}  // namespace agt_map_studio
