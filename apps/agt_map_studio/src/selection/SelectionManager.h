#pragma once

#include "io/PCDLoader.hpp"

#include <Eigen/Core>
#include <QString>

#include <cstddef>
#include <string>
#include <vector>

namespace agt_map_studio {

enum class PointStatus { VISIBLE = 0, SELECTED = 1, DELETED = 2 };

struct AxisAlignedBoundingBox {
  Eigen::Vector3f min = Eigen::Vector3f::Zero();
  Eigen::Vector3f max = Eigen::Vector3f::Zero();
  bool valid = false;
};

// Reproducible geometry that produced a selection. It is serialised into
// agt_map_refinement_core `refinement.yaml` rules so the same deletion can be
// replayed outside the GUI.
struct SelectionGeometry {
  // remove_box | remove_polygon | remove_height_band | remove_sphere
  std::string rule_type = "remove_box";
  AxisAlignedBoundingBox box;                   // remove_box / evidence AABB
  std::vector<Eigen::Vector2d> polygon_xy;      // remove_polygon (map XY)
  double z_min = 0.0;                           // polygon z_range / height band
  double z_max = 0.0;
  bool has_z_range = false;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  // remove_sphere
  double radius = 0.0;
};

struct EditOperation {
  std::size_t id = 0;
  std::string type;
  AxisAlignedBoundingBox box;
  SelectionGeometry geometry;
  std::size_t point_count = 0;
  std::string timestamp;
  bool undone = false;
};

class SelectionManager {
public:
  void reset(std::size_t point_count);
  void select_points(const std::vector<std::size_t> &indices,
                     const AxisAlignedBoundingBox &box);
  void select_points(const std::vector<std::size_t> &indices,
                     const SelectionGeometry &geometry);
  void invert_selection();
  // Replace the rule geometry describing the current selection (used after
  // invert_selection, when the viewer recomputes the AABB of selected points).
  void set_selection_geometry(const SelectionGeometry &geometry) { selection_geometry_ = geometry; }
  void clear_selection();
  bool delete_selected();
  bool undo();
  bool redo();

  const std::vector<PointStatus> &statuses() const { return statuses_; }
  const std::vector<EditOperation> &history() const { return history_; }
  const AxisAlignedBoundingBox &selection_box() const { return selection_geometry_.box; }
  const SelectionGeometry &selection_geometry() const { return selection_geometry_; }
  std::size_t total_count() const { return statuses_.size(); }
  std::size_t visible_count() const;
  std::size_t selected_count() const;
  std::size_t deleted_count() const;
  bool has_active_deletes() const;

  // Hidden-point flags are pure view state (not exported).
  void set_hide_deleted(bool hide) { hide_deleted_ = hide; }
  bool hide_deleted() const { return hide_deleted_; }
  void set_isolate_selected(bool isolate) { isolate_selected_ = isolate; }
  bool isolate_selected() const { return isolate_selected_; }

  // Stable fingerprint of the active (not undone) deletions, for freshness.
  QString active_fingerprint() const;

  bool export_clean_map(const LoadedPointCloud &cloud, const QString &output_dir,
                        const QString &source_path, QString *error) const;
  // Write agt_map_refinement_core rules (version 1) for the active deletions.
  bool write_refinement_rules(const QString &path, const QString &source_path,
                              QString *error) const;

private:
  struct DeleteCommand {
    std::vector<std::size_t> indices;
    std::vector<PointStatus> before_status;
    SelectionGeometry geometry;
    std::size_t operation_id = 0;
  };

  static std::string timestamp_now();
  void rebuild_selection_from_statuses();
  EditOperation *operation(std::size_t id);

  std::vector<PointStatus> statuses_;
  std::vector<std::size_t> selected_indices_;
  SelectionGeometry selection_geometry_;
  std::vector<EditOperation> history_;
  std::vector<DeleteCommand> undo_stack_;
  std::vector<DeleteCommand> redo_stack_;
  std::size_t next_operation_id_ = 1;
  bool hide_deleted_ = false;
  bool isolate_selected_ = false;
};

}  // namespace agt_map_studio
