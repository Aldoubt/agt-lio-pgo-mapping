#include "agt_pcd2grid_exporter/ParameterLoader.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>

namespace agt_pcd2grid_exporter {

bool ParameterLoader::load(const std::string &path,
                           ProjectionParameters *parameters,
                           std::string *error) {
  if (!parameters) {
    if (error) *error = "parameters must not be null";
    return false;
  }
  try {
    const YAML::Node root = YAML::LoadFile(path);
    *parameters = ProjectionParameters();
    parameters->resolution = root["resolution"].as<float>(parameters->resolution);
    const YAML::Node origin = root["origin"];
    if (origin) {
      parameters->origin_auto = origin["auto"].as<bool>(true);
      parameters->origin_x = origin["x"].as<float>(0.0F);
      parameters->origin_y = origin["y"].as<float>(0.0F);
    }
    const YAML::Node z_filter = root["z_filter"];
    if (z_filter) {
      parameters->z_min = z_filter["min"].as<float>(parameters->z_min);
      parameters->z_max = z_filter["max"].as<float>(parameters->z_max);
    }
    parameters->occupied_threshold = root["occupied_threshold"].as<std::uint32_t>(3U);
    const std::string empty_cell = root["empty_cell"].as<std::string>("unknown");
    if (empty_cell == "free") {
      parameters->empty_cell = EmptyCellPolicy::Free;
    } else if (empty_cell == "unknown") {
      parameters->empty_cell = EmptyCellPolicy::Unknown;
    } else {
      if (error) *error = "empty_cell must be 'unknown' or 'free'";
      return false;
    }
    if (!(parameters->resolution > 0.0F) ||
        parameters->z_min > parameters->z_max ||
        parameters->occupied_threshold == 0U) {
      if (error) *error = "projection.yaml contains invalid values";
      return false;
    }
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
  return true;
}

bool ParameterLoader::write(const std::string &path,
                            const ProjectionParameters &parameters,
                            std::string *error) {
  try {
    YAML::Emitter emitter;
    emitter << YAML::BeginMap << YAML::Key << "resolution" << YAML::Value
            << parameters.resolution << YAML::Key << "origin" << YAML::Value
            << YAML::BeginMap << YAML::Key << "auto" << YAML::Value
            << parameters.origin_auto << YAML::Key << "x" << YAML::Value
            << parameters.origin_x << YAML::Key << "y" << YAML::Value
            << parameters.origin_y << YAML::EndMap << YAML::Key << "z_filter"
            << YAML::Value << YAML::BeginMap << YAML::Key << "min" << YAML::Value
            << parameters.z_min << YAML::Key << "max" << YAML::Value
            << parameters.z_max << YAML::EndMap << YAML::Key
            << "occupied_threshold" << YAML::Value << parameters.occupied_threshold
            << YAML::Key << "empty_cell" << YAML::Value
            << (parameters.empty_cell == EmptyCellPolicy::Free ? "free" : "unknown")
            << YAML::EndMap;
    std::ofstream stream(path);
    if (!stream) {
      if (error) *error = "cannot write projection.yaml: " + path;
      return false;
    }
    stream << emitter.c_str() << '\n';
    if (!stream.good()) {
      if (error) *error = "failed writing projection.yaml: " + path;
      return false;
    }
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
  return true;
}

}  // namespace agt_pcd2grid_exporter
