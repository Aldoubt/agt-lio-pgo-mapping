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

struct EditOperation {
  std::size_t id = 0;
  std::string type;
  AxisAlignedBoundingBox box;
  std::size_t point_count = 0;
  std::string timestamp;
  bool undone = false;
};

class SelectionManager {
public:
  void reset(std::size_t point_count);
  void select_points(const std::vector<std::size_t> &indices,
                     const AxisAlignedBoundingBox &box);
  bool delete_selected();
  bool undo();
  bool redo();

  const std::vector<PointStatus> &statuses() const { return statuses_; }
  const std::vector<EditOperation> &history() const { return history_; }
  const AxisAlignedBoundingBox &selection_box() const { return selection_box_; }
  std::size_t total_count() const { return statuses_.size(); }
  std::size_t visible_count() const;
  std::size_t selected_count() const;
  std::size_t deleted_count() const;

  bool export_clean_map(const LoadedPointCloud &cloud, const QString &output_dir,
                        const QString &source_path, QString *error) const;

private:
  struct DeleteBoxCommand {
    std::vector<std::size_t> indices;
    std::vector<PointStatus> before_status;
    AxisAlignedBoundingBox box;
    std::size_t operation_id = 0;
  };

  static std::string timestamp_now();
  void clear_selection();
  void rebuild_selection_from_statuses();
  EditOperation *operation(std::size_t id);

  std::vector<PointStatus> statuses_;
  std::vector<std::size_t> selected_indices_;
  AxisAlignedBoundingBox selection_box_;
  std::vector<EditOperation> history_;
  std::vector<DeleteBoxCommand> undo_stack_;
  std::vector<DeleteBoxCommand> redo_stack_;
  std::size_t next_operation_id_ = 1;
};

}  // namespace agt_map_studio
