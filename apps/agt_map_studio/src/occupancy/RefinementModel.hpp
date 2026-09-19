#pragma once

#include "occupancy/GridMap.hpp"
#include "occupancy/MapYamlLoader.hpp"
#include "occupancy/commands/GridCommand.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace agt_map_studio {

struct ForbiddenZone {
  std::size_t id = 0;
  std::vector<GridWorldPoint> polygon;
};

class RefinementModel {
public:
  void set_base_map(GridMap map, MapYamlMetadata metadata);
  void clear();
  bool has_map() const { return !base_map_.empty(); }
  const GridMap &base_map() const { return base_map_; }
  const MapYamlMetadata &metadata() const { return metadata_; }
  std::int8_t effective_at(std::uint32_t pixel_x, std::uint32_t pixel_y) const;
  std::int8_t effective_at_index(std::size_t index) const;
  const std::vector<ForbiddenZone> &forbidden_zones() const {
    return forbidden_zones_;
  }
  const std::vector<RefinementOperation> &history() const { return history_; }
  std::size_t active_override_count() const { return cell_overrides_.size(); }

  bool execute(std::unique_ptr<GridCommand> command, std::string *error);
  bool undo();
  bool redo();

  void apply_cell_changes(const std::vector<CellChange> &changes, bool after);
  void add_forbidden_zone(std::size_t id,
                          const std::vector<GridWorldPoint> &polygon);
  void remove_forbidden_zone(std::size_t id);

  bool save_refinement_yaml(const std::string &path, std::string *error) const;
  bool load_refinement_yaml(const std::string &path, std::string *error);
  bool export_navigation_map(const std::string &output_dir,
                             std::string *error) const;

  std::size_t next_operation_id() const { return next_operation_id_; }
  static std::string timestamp_now();

private:
  void set_history_undone(std::size_t id, bool undone);
  void rebuild_overrides_from_history();

  GridMap base_map_;
  MapYamlMetadata metadata_;
  std::unordered_map<std::size_t, std::int8_t> cell_overrides_;
  std::vector<ForbiddenZone> forbidden_zones_;
  std::vector<RefinementOperation> history_;
  std::vector<std::unique_ptr<GridCommand>> undo_stack_;
  std::vector<std::unique_ptr<GridCommand>> redo_stack_;
  std::size_t next_operation_id_ = 1;
};

}  // namespace agt_map_studio
