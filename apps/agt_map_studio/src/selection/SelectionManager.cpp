#include "selection/SelectionManager.h"

#include <Eigen/Geometry>

#include <QDir>
#include <QFileInfo>

#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace agt_map_studio {
namespace {

}  // namespace

void SelectionManager::reset(std::size_t point_count) {
  statuses_.assign(point_count, PointStatus::VISIBLE);
  selected_indices_.clear();
  selection_box_ = AxisAlignedBoundingBox();
  history_.clear();
  undo_stack_.clear();
  redo_stack_.clear();
  next_operation_id_ = 1;
}

void SelectionManager::select_points(const std::vector<std::size_t> &indices,
                                     const AxisAlignedBoundingBox &box) {
  for (auto &status : statuses_) {
    if (status == PointStatus::SELECTED) status = PointStatus::VISIBLE;
  }
  selected_indices_.clear();
  for (const auto index : indices) {
    if (index < statuses_.size() && statuses_[index] != PointStatus::DELETED) {
      statuses_[index] = PointStatus::SELECTED;
      selected_indices_.push_back(index);
    }
  }
  selection_box_ = selected_indices_.empty() ? AxisAlignedBoundingBox() : box;
}

bool SelectionManager::delete_selected() {
  if (selected_indices_.empty()) return false;
  DeleteBoxCommand command;
  command.indices = selected_indices_;
  command.before_status.reserve(command.indices.size());
  command.box = selection_box_;
  command.operation_id = next_operation_id_++;
  for (const auto index : command.indices) {
    command.before_status.push_back(statuses_[index]);
    statuses_[index] = PointStatus::DELETED;
  }
  history_.push_back({command.operation_id, "delete_box", command.box,
                      command.indices.size(), timestamp_now(), false});
  undo_stack_.push_back(std::move(command));
  redo_stack_.clear();
  clear_selection();
  return true;
}

bool SelectionManager::undo() {
  if (undo_stack_.empty()) return false;
  DeleteBoxCommand command = std::move(undo_stack_.back());
  undo_stack_.pop_back();
  for (std::size_t i = 0; i < command.indices.size(); ++i) {
    statuses_[command.indices[i]] = command.before_status[i];
  }
  if (auto *entry = operation(command.operation_id)) entry->undone = true;
  redo_stack_.push_back(std::move(command));
  rebuild_selection_from_statuses();
  return true;
}

bool SelectionManager::redo() {
  if (redo_stack_.empty()) return false;
  DeleteBoxCommand command = std::move(redo_stack_.back());
  redo_stack_.pop_back();
  for (const auto index : command.indices) statuses_[index] = PointStatus::DELETED;
  if (auto *entry = operation(command.operation_id)) entry->undone = false;
  undo_stack_.push_back(std::move(command));
  clear_selection();
  return true;
}

std::size_t SelectionManager::visible_count() const {
  return static_cast<std::size_t>(std::count_if(
      statuses_.begin(), statuses_.end(),
      [](PointStatus status) { return status != PointStatus::DELETED; }));
}

std::size_t SelectionManager::selected_count() const {
  return static_cast<std::size_t>(std::count(statuses_.begin(), statuses_.end(),
                                              PointStatus::SELECTED));
}

std::size_t SelectionManager::deleted_count() const {
  return static_cast<std::size_t>(std::count(statuses_.begin(), statuses_.end(),
                                              PointStatus::DELETED));
}

std::string SelectionManager::timestamp_now() {
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
  const auto fraction = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - seconds).count();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&time, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y-%m-%dT%H:%M:%S") << '.'
         << std::setfill('0') << std::setw(3) << fraction;
  return stream.str();
}

void SelectionManager::clear_selection() {
  for (auto &status : statuses_) {
    if (status == PointStatus::SELECTED) status = PointStatus::VISIBLE;
  }
  selected_indices_.clear();
  selection_box_ = AxisAlignedBoundingBox();
}

void SelectionManager::rebuild_selection_from_statuses() {
  selected_indices_.clear();
  for (std::size_t i = 0; i < statuses_.size(); ++i) {
    if (statuses_[i] == PointStatus::SELECTED) selected_indices_.push_back(i);
  }
  if (selected_indices_.empty()) selection_box_ = AxisAlignedBoundingBox();
}

EditOperation *SelectionManager::operation(std::size_t id) {
  for (auto &entry : history_) {
    if (entry.id == id) return &entry;
  }
  return nullptr;
}

bool SelectionManager::export_clean_map(const LoadedPointCloud &cloud,
                                        const QString &output_dir,
                                        const QString &source_path,
                                        QString *error) const {
  if (!cloud.source || cloud.source_indices.size() != cloud.point_count() ||
      statuses_.size() != cloud.point_count()) {
    if (error) *error = QStringLiteral("Cloud/state layer size mismatch");
    return false;
  }
  QDir directory;
  if (!directory.mkpath(output_dir)) {
    if (error) *error = QStringLiteral("Cannot create export directory: %1").arg(output_dir);
    return false;
  }

  pcl::PCLPointCloud2 output = *cloud.source;
  output.width = static_cast<std::uint32_t>(visible_count());
  output.height = 1;
  output.row_step = output.width * output.point_step;
  output.data.clear();
  output.data.reserve(static_cast<std::size_t>(output.row_step));
  for (std::size_t i = 0; i < statuses_.size(); ++i) {
    if (statuses_[i] == PointStatus::DELETED) continue;
    const std::size_t source_index = cloud.source_indices[i];
    const std::size_t row = cloud.source->width == 0 ? 0 : source_index / cloud.source->width;
    const std::size_t col = cloud.source->width == 0 ? 0 : source_index % cloud.source->width;
    const std::size_t row_step = cloud.source->row_step == 0
                                     ? cloud.source->width * cloud.source->point_step
                                     : cloud.source->row_step;
    const std::size_t offset = row * row_step + col * cloud.source->point_step;
    if (offset + cloud.source->point_step > cloud.source->data.size()) {
      if (error) *error = QStringLiteral("Source record offset is outside PCD data");
      return false;
    }
    output.data.insert(output.data.end(), cloud.source->data.begin() + offset,
                       cloud.source->data.begin() + offset + cloud.source->point_step);
  }
  if (pcl::io::savePCDFile((QDir(output_dir).filePath("map.pcd")).toStdString(),
                           output, Eigen::Vector4f::Zero(),
                           Eigen::Quaternionf::Identity(), true) != 0) {
    if (error) *error = QStringLiteral("Could not write clean map.pcd");
    return false;
  }

  try {
    YAML::Emitter history_emitter;
    history_emitter << YAML::BeginMap << YAML::Key << "version" << YAML::Value << 1
                    << YAML::Key << "operations" << YAML::Value << YAML::BeginSeq;
    for (const auto &entry : history_) {
      history_emitter << YAML::BeginMap << YAML::Key << "id" << YAML::Value << entry.id
                      << YAML::Key << "type" << YAML::Value << entry.type
                      << YAML::Key << "point_count" << YAML::Value << entry.point_count
                      << YAML::Key << "undone" << YAML::Value << entry.undone
                      << YAML::Key << "bbox" << YAML::Value << YAML::BeginMap
                      << YAML::Key << "min" << YAML::Value << YAML::Flow << YAML::BeginSeq
                      << entry.box.min.x() << entry.box.min.y() << entry.box.min.z()
                      << YAML::EndSeq << YAML::Key << "max" << YAML::Value << YAML::Flow
                      << YAML::BeginSeq << entry.box.max.x() << entry.box.max.y()
                      << entry.box.max.z() << YAML::EndSeq << YAML::EndMap
                      << YAML::Key << "timestamp" << YAML::Value << entry.timestamp
                      << YAML::EndMap;
    }
    history_emitter << YAML::EndSeq << YAML::EndMap;
    std::ofstream history_stream(QDir(output_dir).filePath("edit_history.yaml").toStdString());
    history_stream << history_emitter.c_str() << '\n';

    YAML::Emitter metadata_emitter;
    metadata_emitter << YAML::BeginMap << YAML::Key << "version" << YAML::Value << 1
                     << YAML::Key << "source_pcd" << YAML::Value << source_path.toStdString()
                     << YAML::Key << "original_points" << YAML::Value << cloud.point_count()
                     << YAML::Key << "visible_points" << YAML::Value << visible_count()
                     << YAML::Key << "deleted_points" << YAML::Value << deleted_count()
                     << YAML::Key << "fields" << YAML::Value << YAML::BeginSeq;
    for (const auto &field : cloud.source->fields) metadata_emitter << field.name;
    metadata_emitter << YAML::EndSeq << YAML::Key << "exported_at" << YAML::Value
                     << timestamp_now() << YAML::EndMap;
    std::ofstream metadata_stream(QDir(output_dir).filePath("metadata.yaml").toStdString());
    metadata_stream << metadata_emitter.c_str() << '\n';
    if (!history_stream.good() || !metadata_stream.good()) {
      if (error) *error = QStringLiteral("Failed while writing export metadata");
      return false;
    }
  } catch (const std::exception &exception) {
    if (error) *error = QStringLiteral("Metadata export failed: %1").arg(exception.what());
    return false;
  }
  return true;
}

}  // namespace agt_map_studio
