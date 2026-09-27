#include "confidence/SpatialConfidenceEditor.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace agt_map_studio {
namespace core = agt_spatial_map_core;

void SpatialConfidenceEditor::set_model(const SpatialConfidenceModel *model) {
  model_ = model && !model->empty() ? model : nullptr;
  current_.clear();
  saved_.clear();
  undo_stack_.clear();
  redo_stack_.clear();
  stable_count_ = 0;
  if (!model_) return;
  for (const auto &v : model_->voxels()) {
    if (!v.has_override_entry) continue;
    current_.emplace(v.key, ConfidenceOverrideIntent{
        v.override_mode, v.has_manual_value, v.manual_value, v.audit});
  }
  saved_ = current_;
  for (std::size_t i = 0; i < model_->voxels().size(); ++i) {
    if (stable_preview(i)) ++stable_count_;
  }
}

const ConfidenceOverrideIntent *SpatialConfidenceEditor::intent(std::size_t index) const {
  if (!model_) return nullptr;
  const auto &key = model_->voxels().at(index).key;
  const auto found = current_.find(key);
  return found == current_.end() ? nullptr : &found->second;
}

core::ManualOverrideMode SpatialConfidenceEditor::effective_mode(std::size_t index) const {
  const auto *override = intent(index);
  return override ? override->mode : core::ManualOverrideMode::AUTO;
}

float SpatialConfidenceEditor::preview_final(std::size_t index) const {
  if (!model_) throw std::invalid_argument("confidence editor has no model");
  const auto &voxel = model_->voxels().at(index);
  const auto *override = intent(index);
  return core::manual_final_confidence(
      voxel.auto_confidence, override ? override->mode : core::ManualOverrideMode::AUTO,
      override && override->has_manual_value, override ? override->manual_value : 0.0F,
      model_->info().force_low_value);
}

bool SpatialConfidenceEditor::stable_preview(std::size_t index) const {
  if (!model_) return false;
  return core::stable_preview_selected(preview_final(index), effective_mode(index),
                                       model_->info().stable_threshold);
}

bool SpatialConfidenceEditor::stable_with(
    std::size_t index, const std::optional<ConfidenceOverrideIntent> &override) const {
  const auto &v = model_->voxels().at(index);
  const auto mode = override ? override->mode : core::ManualOverrideMode::AUTO;
  const float final = core::manual_final_confidence(
      v.auto_confidence, mode, override && override->has_manual_value,
      override ? override->manual_value : 0.0F, model_->info().force_low_value);
  return core::stable_preview_selected(final, mode, model_->info().stable_threshold);
}

void SpatialConfidenceEditor::run(const Command &command, bool forward) {
  for (const auto &change : command) {
    const auto &key = model_->voxels()[change.index].key;
    const auto found = current_.find(key);
    const auto old_intent = found == current_.end()
        ? std::optional<ConfidenceOverrideIntent>{}
        : std::optional<ConfidenceOverrideIntent>{found->second};
    const auto &next = forward ? change.after : change.before;
    const bool was_stable = stable_with(change.index, old_intent);
    const bool now_stable = stable_with(change.index, next);
    if (was_stable && !now_stable) --stable_count_;
    if (!was_stable && now_stable) ++stable_count_;
    if (next) current_.insert_or_assign(key, *next);
    else current_.erase(key);
  }
}

bool SpatialConfidenceEditor::change(
    const std::vector<std::size_t> &indices,
    const std::optional<ConfidenceOverrideIntent> &after,
    std::string *error) {
  if (error) error->clear();
  if (!model_) {
    if (error) *error = "Open a verified confidence derivative first";
    return false;
  }
  std::vector<std::size_t> unique = indices;
  std::sort(unique.begin(), unique.end());
  unique.erase(std::unique(unique.begin(), unique.end()), unique.end());
  Command command;
  command.reserve(unique.size());
  for (const auto index : unique) {
    if (index >= model_->voxels().size()) {
      if (error) *error = "Voxel selection index is outside the confidence model";
      return false;  // no partial mutation on malformed selection
    }
    const auto found = current_.find(model_->voxels()[index].key);
    std::optional<ConfidenceOverrideIntent> before;
    if (found != current_.end()) before = found->second;
    if (!(before == after)) command.push_back({index, std::move(before), after});
  }
  if (command.empty()) return false;
  undo_stack_.push_back(std::move(command));
  run(undo_stack_.back(), true);
  redo_stack_.clear();
  return true;
}

bool SpatialConfidenceEditor::apply(
    const std::vector<std::size_t> &indices,
    const ConfidenceOverrideIntent &override, std::string *error) {
  if (error) error->clear();
  try {
    if (override.mode == core::ManualOverrideMode::AUTO) {
      throw std::invalid_argument("Use Restore Auto rather than applying an AUTO override");
    }
    if (override.audit.reason.empty()) {
      throw std::invalid_argument("A reason tag is required for human override intent");
    }
    core::validate_manual_override_audit(override.audit);
    if (override.has_manual_value && override.mode != core::ManualOverrideMode::FORCE_LOW) {
      throw std::invalid_argument("Explicit value is valid only for FORCE_LOW");
    }
    // Validate value and mode using the authoritative core, not a copied UI
    // formula. Actual voxel auto evidence is not recomputed here.
    core::manual_final_confidence(0.5F, override.mode,
                                  override.has_manual_value, override.manual_value, .05F);
    return change(indices, override, error);
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
}

bool SpatialConfidenceEditor::restore_auto(
    const std::vector<std::size_t> &indices, std::string *error) {
  return change(indices, std::nullopt, error);
}

bool SpatialConfidenceEditor::undo() {
  if (undo_stack_.empty()) return false;
  Command command = std::move(undo_stack_.back());
  undo_stack_.pop_back();
  run(command, false);
  redo_stack_.push_back(std::move(command));
  return true;
}

bool SpatialConfidenceEditor::redo() {
  if (redo_stack_.empty()) return false;
  Command command = std::move(redo_stack_.back());
  redo_stack_.pop_back();
  run(command, true);
  undo_stack_.push_back(std::move(command));
  return true;
}

bool SpatialConfidenceEditor::dirty() const { return current_ != saved_; }

void SpatialConfidenceEditor::mark_saved() { saved_ = current_; }

std::vector<std::pair<core::VoxelKey, ConfidenceOverrideIntent>>
SpatialConfidenceEditor::sorted_intents() const {
  std::vector<std::pair<Key, ConfidenceOverrideIntent>> sorted(current_.begin(), current_.end());
  std::sort(sorted.begin(), sorted.end(), [](const auto &a, const auto &b) {
    return a.first < b.first;
  });
  return sorted;
}

}  // namespace agt_map_studio
