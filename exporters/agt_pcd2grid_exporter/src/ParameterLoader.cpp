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
    const std::string mode = root["projection_mode"].as<std::string>("local_ground");
    if (mode == "local_ground") {
      parameters->projection_mode = ProjectionMode::LocalGround;
    } else if (mode == "fixed_height") {
      parameters->projection_mode = ProjectionMode::FixedHeight;
    } else {
      if (error) *error = "projection_mode must be 'local_ground' or 'fixed_height'";
      return false;
    }
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
    const YAML::Node ground = root["ground"];
    if (ground) {
      parameters->ground_cell_size =
          ground["cell_size"].as<float>(parameters->ground_cell_size);
      parameters->ground_percentile =
          ground["percentile"].as<float>(parameters->ground_percentile);
      parameters->ground_neighbor_radius =
          ground["neighbor_radius_cells"].as<std::uint32_t>(parameters->ground_neighbor_radius);
      parameters->ground_free_tolerance =
          ground["free_tolerance"].as<float>(parameters->ground_free_tolerance);
    }
    const YAML::Node obstacle = root["obstacle_height"];
    if (obstacle) {
      parameters->obstacle_min_height =
          obstacle["min"].as<float>(parameters->obstacle_min_height);
      parameters->obstacle_max_height =
          obstacle["max"].as<float>(parameters->obstacle_max_height);
    }
    parameters->occupied_threshold =
        root["occupied_threshold"].as<std::uint32_t>(parameters->occupied_threshold);
    parameters->free_threshold =
        root["free_threshold"].as<std::uint32_t>(parameters->free_threshold);
    parameters->free_space_radius_cells = root["free_space_radius_cells"].as<std::uint32_t>(
        parameters->free_space_radius_cells);
    const YAML::Node postprocess = root["postprocess"];
    if (postprocess) {
      parameters->closing_radius_cells = postprocess["closing_radius_cells"].as<std::uint32_t>(
          parameters->closing_radius_cells);
      parameters->min_component_cells = postprocess["min_component_cells"].as<std::uint32_t>(
          parameters->min_component_cells);
    }
    const YAML::Node temporal = root["temporal_filter"];
    if (temporal) {
      parameters->temporal_filter_enabled =
          temporal["enabled"].as<bool>(parameters->temporal_filter_enabled);
      parameters->temporal_voxel_size =
          temporal["voxel_size"].as<float>(parameters->temporal_voxel_size);
      parameters->temporal_min_observations = temporal["min_observations"].as<std::uint32_t>(
          parameters->temporal_min_observations);
      parameters->temporal_min_keyframe_span = temporal["min_keyframe_span"].as<std::uint32_t>(
          parameters->temporal_min_keyframe_span);
    }
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
        parameters->occupied_threshold == 0U || parameters->free_threshold == 0U ||
        !(parameters->ground_cell_size > 0.0F) ||
        parameters->ground_percentile < 0.0F || parameters->ground_percentile > 1.0F ||
        parameters->ground_free_tolerance < 0.0F ||
        parameters->obstacle_min_height < 0.0F ||
        parameters->obstacle_min_height > parameters->obstacle_max_height ||
        !(parameters->temporal_voxel_size > 0.0F) ||
        parameters->temporal_min_observations == 0U) {
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
            << parameters.resolution << YAML::Key << "projection_mode" << YAML::Value
            << (parameters.projection_mode == ProjectionMode::LocalGround
                    ? "local_ground" : "fixed_height")
            << YAML::Key << "origin" << YAML::Value
            << YAML::BeginMap << YAML::Key << "auto" << YAML::Value
            << parameters.origin_auto << YAML::Key << "x" << YAML::Value
            << parameters.origin_x << YAML::Key << "y" << YAML::Value
            << parameters.origin_y << YAML::EndMap << YAML::Key << "z_filter"
            << YAML::Value << YAML::BeginMap << YAML::Key << "min" << YAML::Value
            << parameters.z_min << YAML::Key << "max" << YAML::Value
            << parameters.z_max << YAML::EndMap << YAML::Key << "ground" << YAML::Value
            << YAML::BeginMap << YAML::Key << "cell_size" << YAML::Value
            << parameters.ground_cell_size << YAML::Key << "percentile" << YAML::Value
            << parameters.ground_percentile << YAML::Key << "neighbor_radius_cells" << YAML::Value
            << parameters.ground_neighbor_radius << YAML::Key << "free_tolerance" << YAML::Value
            << parameters.ground_free_tolerance << YAML::EndMap << YAML::Key
            << "obstacle_height" << YAML::Value << YAML::BeginMap << YAML::Key << "min"
            << YAML::Value << parameters.obstacle_min_height << YAML::Key << "max"
            << YAML::Value << parameters.obstacle_max_height << YAML::EndMap << YAML::Key
            << "occupied_threshold" << YAML::Value << parameters.occupied_threshold
            << YAML::Key << "free_threshold" << YAML::Value << parameters.free_threshold
            << YAML::Key << "free_space_radius_cells" << YAML::Value
            << parameters.free_space_radius_cells
            << YAML::Key << "postprocess" << YAML::Value << YAML::BeginMap << YAML::Key
            << "closing_radius_cells" << YAML::Value << parameters.closing_radius_cells
            << YAML::Key << "min_component_cells" << YAML::Value
            << parameters.min_component_cells << YAML::EndMap << YAML::Key
            << "temporal_filter" << YAML::Value << YAML::BeginMap << YAML::Key << "enabled"
            << YAML::Value << parameters.temporal_filter_enabled << YAML::Key << "voxel_size"
            << YAML::Value << parameters.temporal_voxel_size << YAML::Key << "min_observations"
            << YAML::Value << parameters.temporal_min_observations << YAML::Key
            << "min_keyframe_span" << YAML::Value << parameters.temporal_min_keyframe_span
            << YAML::EndMap
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
