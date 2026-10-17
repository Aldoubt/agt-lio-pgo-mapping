#pragma once

#include "confidence/SpatialConfidenceModel.hpp"

#include <agt_spatial_map_core/spatial_evidence.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace agt_map_studio {

struct ConfidenceOverrideIntent {
  agt_spatial_map_core::ManualOverrideMode mode =
      agt_spatial_map_core::ManualOverrideMode::AUTO;
  bool has_manual_value = false;
  float manual_value = 0.0F;
  agt_spatial_map_core::ManualOverrideAudit audit;

  bool operator==(const ConfidenceOverrideIntent &other) const {
    return mode == other.mode && has_manual_value == other.has_manual_value &&
           (!has_manual_value || manual_value == other.manual_value) &&
           audit == other.audit;
  }
};

// Independent UI intent/command stack. Never mutates SpatialConfidenceModel,
// raw LoadedPointCloud, auto_confidence, geometry_score or source PCD. All
// preview final/stable semantics are delegated to agt_spatial_map_core.
class SpatialConfidenceEditor {
public:
  void set_model(const SpatialConfidenceModel *model);
  const SpatialConfidenceModel *model() const { return model_; }
  const ConfidenceOverrideIntent *intent(std::size_t voxel_index) const;
  agt_spatial_map_core::ManualOverrideMode effective_mode(std::size_t index) const;
  float preview_final(std::size_t index) const;
  bool stable_preview(std::size_t index) const;
  std::size_t stable_preview_count() const { return stable_count_; }
  std::size_t override_count() const { return current_.size(); }

  // Apply an audited non-AUTO intent to voxel *indices* from the confidence
  // selection manager only. For AUTO call restore_auto(). Returns false for
  // errors (with message) or a no-op (empty message); never partially applies.
  bool apply(const std::vector<std::size_t> &voxel_indices,
             const ConfidenceOverrideIntent &intent, std::string *error = nullptr);
  bool restore_auto(const std::vector<std::size_t> &voxel_indices,
                    std::string *error = nullptr);
  bool undo();
  bool redo();
  bool can_undo() const { return !undo_stack_.empty(); }
  bool can_redo() const { return !redo_stack_.empty(); }
  bool dirty() const;
  void mark_saved();
  std::vector<std::pair<agt_spatial_map_core::VoxelKey, ConfidenceOverrideIntent>>
  sorted_intents() const;

private:
  using Key = agt_spatial_map_core::VoxelKey;
  using Map = std::unordered_map<Key, ConfidenceOverrideIntent,
                                 agt_spatial_map_core::VoxelKeyHash>;
  struct Change {
    std::size_t index = 0;
    std::optional<ConfidenceOverrideIntent> before;
    std::optional<ConfidenceOverrideIntent> after;
  };
  using Command = std::vector<Change>;

  bool change(const std::vector<std::size_t> &indices,
              const std::optional<ConfidenceOverrideIntent> &after,
              std::string *error);
  bool stable_with(std::size_t index,
                   const std::optional<ConfidenceOverrideIntent> &intent) const;
  void run(const Command &command, bool forward);

  const SpatialConfidenceModel *model_ = nullptr;  // owned by MainWindow
  Map current_;
  Map saved_;
  std::vector<Command> undo_stack_;
  std::vector<Command> redo_stack_;
  std::size_t stable_count_ = 0;
};

}  // namespace agt_map_studio
