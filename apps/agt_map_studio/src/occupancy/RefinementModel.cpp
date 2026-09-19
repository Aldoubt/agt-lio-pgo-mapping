#include "occupancy/RefinementModel.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <algorithm>
#include <cstdint>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace agt_map_studio {

void RefinementModel::set_base_map(GridMap map, MapYamlMetadata metadata) {
  base_map_ = std::move(map);
  metadata_ = metadata;
  cell_overrides_.clear();
  forbidden_zones_.clear();
  history_.clear();
  undo_stack_.clear();
  redo_stack_.clear();
  next_operation_id_ = 1;
}

void RefinementModel::clear() {
  base_map_.clear();
  cell_overrides_.clear();
  forbidden_zones_.clear();
  history_.clear();
  undo_stack_.clear();
  redo_stack_.clear();
  next_operation_id_ = 1;
}

std::int8_t RefinementModel::effective_at(std::uint32_t pixel_x,
                                          std::uint32_t pixel_y) const {
  if (pixel_x >= base_map_.width() || pixel_y >= base_map_.height()) {
    return GridMap::kUnknown;
  }
  return effective_at_index(static_cast<std::size_t>(pixel_y) * base_map_.width() +
                            pixel_x);
}

std::int8_t RefinementModel::effective_at_index(std::size_t index) const {
  const auto override = cell_overrides_.find(index);
  if (override != cell_overrides_.end()) return override->second;
  if (index >= base_map_.cells().size()) return GridMap::kUnknown;
  return base_map_.cells()[index];
}

bool RefinementModel::execute(std::unique_ptr<GridCommand> command,
                              std::string *error) {
  if (!command || !has_map()) {
    if (error) *error = "a base occupancy map and command are required";
    return false;
  }
  command->redo(*this);
  history_.push_back(command->operation());
  next_operation_id_ = std::max(next_operation_id_, command->operation().id + 1U);
  undo_stack_.push_back(std::move(command));
  redo_stack_.clear();
  return true;
}

bool RefinementModel::undo() {
  if (undo_stack_.empty()) return false;
  auto command = std::move(undo_stack_.back());
  undo_stack_.pop_back();
  command->undo(*this);
  set_history_undone(command->operation().id, true);
  redo_stack_.push_back(std::move(command));
  return true;
}

bool RefinementModel::redo() {
  if (redo_stack_.empty()) return false;
  auto command = std::move(redo_stack_.back());
  redo_stack_.pop_back();
  command->redo(*this);
  set_history_undone(command->operation().id, false);
  undo_stack_.push_back(std::move(command));
  return true;
}

void RefinementModel::apply_cell_changes(const std::vector<CellChange> &changes,
                                         bool after) {
  for (const auto &change : changes) {
    if (change.index >= base_map_.cells().size()) continue;
    const std::int8_t value = after ? change.after : change.before;
    if (value == base_map_.cells()[change.index]) {
      cell_overrides_.erase(change.index);
    } else {
      cell_overrides_[change.index] = value;
    }
  }
}

void RefinementModel::add_forbidden_zone(
    std::size_t id, const std::vector<GridWorldPoint> &polygon) {
  forbidden_zones_.push_back({id, polygon});
}

void RefinementModel::remove_forbidden_zone(std::size_t id) {
  forbidden_zones_.erase(
      std::remove_if(forbidden_zones_.begin(), forbidden_zones_.end(),
                     [id](const ForbiddenZone &zone) { return zone.id == id; }),
      forbidden_zones_.end());
}

void RefinementModel::set_history_undone(std::size_t id, bool undone) {
  for (auto &entry : history_) {
    if (entry.id == id) entry.undone = undone;
  }
}

std::string RefinementModel::timestamp_now() {
  const auto now = std::chrono::system_clock::now();
  const auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
  const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                                now - seconds)
                                .count();
  const std::time_t time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&time, &local_time);
  std::ostringstream stream;
  stream << std::put_time(&local_time, "%Y-%m-%dT%H:%M:%S") << '.'
         << std::setfill('0') << std::setw(3) << milliseconds;
  return stream.str();
}

bool RefinementModel::save_refinement_yaml(const std::string &path,
                                           std::string *error) const {
  if (!has_map()) {
    if (error) *error = "no base occupancy map loaded";
    return false;
  }
  try {
    const std::filesystem::path file_path(path);
    if (!file_path.parent_path().empty()) {
      std::filesystem::create_directories(file_path.parent_path());
    }
    YAML::Emitter emitter;
    emitter << YAML::BeginMap << YAML::Key << "version" << YAML::Value << 1
            << YAML::Key << "base_map" << YAML::Value << YAML::BeginMap
            << YAML::Key << "yaml" << YAML::Value << base_map_.yaml_path()
            << YAML::EndMap << YAML::Key << "operations" << YAML::Value
            << YAML::BeginSeq;
    for (const auto &entry : history_) {
      emitter << YAML::BeginMap << YAML::Key << "id" << YAML::Value << entry.id
              << YAML::Key << "type" << YAML::Value << entry.type
              << YAML::Key << "timestamp" << YAML::Value << entry.timestamp
              << YAML::Key << "undone" << YAML::Value << entry.undone
              << YAML::Key << "geometry" << YAML::Value << YAML::BeginMap;
      if (entry.type == "erase_rectangle" && entry.geometry.size() >= 2U) {
        emitter << YAML::Key << "min" << YAML::Value << YAML::Flow << YAML::BeginSeq
                << entry.geometry[0].x << entry.geometry[0].y << YAML::EndSeq
                << YAML::Key << "max" << YAML::Value << YAML::Flow << YAML::BeginSeq
                << entry.geometry[1].x << entry.geometry[1].y << YAML::EndSeq;
      } else if (entry.type == "draw_obstacle" && entry.geometry.size() >= 2U) {
        emitter << YAML::Key << "start" << YAML::Value << YAML::Flow << YAML::BeginSeq
                << entry.geometry[0].x << entry.geometry[0].y << YAML::EndSeq
                << YAML::Key << "end" << YAML::Value << YAML::Flow << YAML::BeginSeq
                << entry.geometry[1].x << entry.geometry[1].y << YAML::EndSeq
                << YAML::Key << "width_m" << YAML::Value << entry.width_m;
      } else {
        emitter << YAML::Key << "polygon" << YAML::Value << YAML::BeginSeq;
        for (const auto &point : entry.geometry) {
          emitter << YAML::Flow << YAML::BeginSeq << point.x << point.y
                  << YAML::EndSeq;
        }
        emitter << YAML::EndSeq;
      }
      emitter << YAML::EndMap << YAML::Key << "changes" << YAML::Value
              << YAML::BeginSeq;
      for (const auto &change : entry.changes) {
        const auto x = static_cast<std::uint32_t>(change.index % base_map_.width());
        const auto y = static_cast<std::uint32_t>(change.index / base_map_.width());
        emitter << YAML::BeginMap << YAML::Key << "pixel" << YAML::Value << YAML::Flow
                << YAML::BeginSeq << x << y << YAML::EndSeq << YAML::Key << "before"
                << YAML::Value << static_cast<int>(change.before) << YAML::Key << "after"
                << YAML::Value << static_cast<int>(change.after) << YAML::EndMap;
      }
      emitter << YAML::EndSeq << YAML::EndMap;
    }
    emitter << YAML::EndSeq << YAML::EndMap;
    std::ofstream stream(file_path);
    if (!stream) {
      if (error) *error = "cannot write refinement file: " + path;
      return false;
    }
    stream << emitter.c_str() << '\n';
    if (!stream.good()) {
      if (error) *error = "failed writing refinement file: " + path;
      return false;
    }
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
  return true;
}

bool RefinementModel::load_refinement_yaml(const std::string &path,
                                           std::string *error) {
  if (!has_map()) {
    if (error) *error = "load a base map before loading refinement";
    return false;
  }
  try {
    const YAML::Node root = YAML::LoadFile(path);
    const YAML::Node operations = root["operations"];
    if (!operations || !operations.IsSequence()) {
      if (error) *error = "refinement operations must be a sequence";
      return false;
    }
    cell_overrides_.clear();
    forbidden_zones_.clear();
    history_.clear();
    undo_stack_.clear();
    redo_stack_.clear();
    next_operation_id_ = 1;
    for (const auto &node : operations) {
      RefinementOperation operation;
      operation.id = node["id"].as<std::size_t>();
      operation.type = node["type"].as<std::string>();
      operation.timestamp = node["timestamp"].as<std::string>("");
      operation.undone = node["undone"].as<bool>(false);
      const YAML::Node geometry = node["geometry"];
      if (operation.type == "erase_rectangle") {
        for (const char *key : {"min", "max"}) {
          const auto point = geometry[key];
          operation.geometry.push_back({point[0].as<double>(), point[1].as<double>()});
        }
      } else if (operation.type == "draw_obstacle") {
        for (const char *key : {"start", "end"}) {
          const auto point = geometry[key];
          operation.geometry.push_back({point[0].as<double>(), point[1].as<double>()});
        }
        operation.width_m = geometry["width_m"].as<double>();
      } else if (operation.type == "forbidden_polygon") {
        for (const auto &point : geometry["polygon"]) {
          operation.geometry.push_back({point[0].as<double>(), point[1].as<double>()});
        }
      }
      for (const auto &change : node["changes"]) {
        const auto pixel = change["pixel"];
        const std::size_t index = static_cast<std::size_t>(pixel[1].as<std::uint32_t>()) *
                                  base_map_.width() + pixel[0].as<std::uint32_t>();
        operation.changes.push_back(
            {index, static_cast<std::int8_t>(change["before"].as<int>()),
             static_cast<std::int8_t>(change["after"].as<int>())});
      }
      if (!operation.undone) {
        apply_cell_changes(operation.changes, true);
        if (operation.type == "forbidden_polygon") {
          add_forbidden_zone(operation.id, operation.geometry);
        }
      }
      history_.push_back(operation);
      next_operation_id_ = std::max(next_operation_id_, operation.id + 1U);
    }
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
  return true;
}

bool RefinementModel::export_navigation_map(const std::string &output_dir,
                                            std::string *error) const {
  if (!has_map()) {
    if (error) *error = "no base occupancy map loaded";
    return false;
  }
  try {
    std::filesystem::create_directories(output_dir);
    const std::filesystem::path directory(output_dir);
    std::ofstream pgm(directory / "map.pgm", std::ios::binary);
    if (!pgm) {
      if (error) *error = "cannot write navigation map PGM";
      return false;
    }
    pgm << "P5\n" << base_map_.width() << ' ' << base_map_.height() << "\n255\n";
    for (std::int64_t image_y = static_cast<std::int64_t>(base_map_.height()) - 1;
         image_y >= 0; --image_y) {
      for (std::uint32_t x = 0; x < base_map_.width(); ++x) {
        const auto value = effective_at(x, static_cast<std::uint32_t>(image_y));
        unsigned char pixel = value == GridMap::kOccupied ? 0U
                             : value == GridMap::kFree ? 254U : 205U;
        if (metadata_.negate && value != GridMap::kUnknown) pixel = 255U - pixel;
        pgm.write(reinterpret_cast<const char *>(&pixel), 1);
      }
    }
    YAML::Emitter map_yaml;
    map_yaml << YAML::BeginMap << YAML::Key << "image" << YAML::Value << "map.pgm"
             << YAML::Key << "resolution" << YAML::Value << base_map_.resolution()
             << YAML::Key << "origin" << YAML::Value << YAML::Flow << YAML::BeginSeq
             << base_map_.origin_x() << base_map_.origin_y() << 0.0 << YAML::EndSeq
             << YAML::Key << "occupied_thresh" << YAML::Value << metadata_.occupied_thresh
             << YAML::Key << "free_thresh" << YAML::Value << metadata_.free_thresh
             << YAML::Key << "negate" << YAML::Value << (metadata_.negate ? 1 : 0)
             << YAML::Key << "mode" << YAML::Value << metadata_.mode << YAML::EndMap;
    std::ofstream yaml_stream(directory / "map.yaml");
    yaml_stream << map_yaml.c_str() << '\n';
    std::string refinement_error;
    if (!save_refinement_yaml((directory / "map_refinement.yaml").string(),
                              &refinement_error)) {
      if (error) *error = refinement_error;
      return false;
    }
    YAML::Emitter metadata;
    metadata << YAML::BeginMap << YAML::Key << "base_map_yaml" << YAML::Value
             << base_map_.yaml_path() << YAML::Key << "width" << YAML::Value
             << base_map_.width() << YAML::Key << "height" << YAML::Value
             << base_map_.height() << YAML::Key << "active_overrides" << YAML::Value
             << cell_overrides_.size() << YAML::Key << "forbidden_zones" << YAML::Value
             << forbidden_zones_.size() << YAML::Key << "operation_count" << YAML::Value
             << history_.size() << YAML::EndMap;
    std::ofstream metadata_stream(directory / "metadata.yaml");
    metadata_stream << metadata.c_str() << '\n';
    if (!pgm.good() || !yaml_stream.good() || !metadata_stream.good()) {
      if (error) *error = "failed writing navigation map asset";
      return false;
    }
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
  return true;
}

}  // namespace agt_map_studio
