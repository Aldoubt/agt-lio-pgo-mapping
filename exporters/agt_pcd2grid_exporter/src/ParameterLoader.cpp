#include "agt_pcd2grid_exporter/ParameterLoader.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <fstream>

namespace agt_pcd2grid_exporter {

namespace {

const char *mode_name(ProjectionMode mode) {
  switch (mode) {
    case ProjectionMode::FixedHeight:
      return "fixed_height";
    case ProjectionMode::Traversability:
      return "traversability";
    case ProjectionMode::LocalGround:
    default:
      return "local_ground";
  }
}

bool load_triplet(const YAML::Node &node, std::array<float, 3> *value) {
  if (!node) return true;
  if (!node.IsSequence() || node.size() != 3U) return false;
  for (std::size_t i = 0; i < 3U; ++i) (*value)[i] = node[i].as<float>();
  return true;
}

bool load_robot(const YAML::Node &robot, RobotModel *model, std::string *error) {
  if (!robot) return true;
  const YAML::Node transform = robot["base_from_body"];
  if (transform) {
    if (!load_triplet(transform["xyz"], &model->base_from_body_xyz) ||
        !load_triplet(transform["rpy"], &model->base_from_body_rpy)) {
      if (error) *error = "robot.base_from_body.xyz/rpy must be 3-element sequences";
      return false;
    }
  }
  const YAML::Node footprint = robot["footprint"];
  if (footprint) {
    if (!footprint.IsSequence() || footprint.size() < 3U) {
      if (error) *error = "robot.footprint must be a sequence of at least three [x, y] points";
      return false;
    }
    model->footprint.clear();
    for (const auto &vertex : footprint) {
      if (!vertex.IsSequence() || vertex.size() != 2U) {
        if (error) *error = "robot.footprint vertices must be [x, y]";
        return false;
      }
      model->footprint.emplace_back(vertex[0].as<float>(), vertex[1].as<float>());
    }
  }
  return true;
}

void load_traversability(const YAML::Node &node, TraversabilityParameters *tp) {
  if (!node) return;
  if (const YAML::Node self_filter = node["self_filter"]) {
    tp->self_filter_enabled = self_filter["enabled"].as<bool>(tp->self_filter_enabled);
    tp->self_filter_margin = self_filter["margin"].as<float>(tp->self_filter_margin);
    tp->self_filter_min_height = self_filter["min_height"].as<float>(tp->self_filter_min_height);
    tp->self_filter_max_height = self_filter["max_height"].as<float>(tp->self_filter_max_height);
  }
  if (const YAML::Node ground = node["ground"]) {
    tp->ground_cell_size = ground["cell_size"].as<float>(tp->ground_cell_size);
    tp->ground_percentile = ground["percentile"].as<float>(tp->ground_percentile);
    tp->ground_max_step = ground["max_step"].as<float>(tp->ground_max_step);
    tp->ground_fill_max_distance =
        ground["fill_max_distance"].as<float>(tp->ground_fill_max_distance);
    tp->ground_tolerance = ground["tolerance"].as<float>(tp->ground_tolerance);
    tp->ground_reference_extension =
        ground["reference_extension"].as<float>(tp->ground_reference_extension);
  }
  if (const YAML::Node raycast = node["raycast"]) {
    tp->raycast_enabled = raycast["enabled"].as<bool>(tp->raycast_enabled);
    tp->raycast_max_range = raycast["max_range"].as<float>(tp->raycast_max_range);
    tp->carve_max_height = raycast["carve_max_height"].as<float>(tp->carve_max_height);
    tp->carve_end_margin_cells =
        raycast["end_margin_cells"].as<std::uint32_t>(tp->carve_end_margin_cells);
    tp->free_min_passes = raycast["free_min_passes"].as<std::uint32_t>(tp->free_min_passes);
  }
  if (const YAML::Node evidence = node["obstacle_evidence"]) {
    tp->obstacle_support_radius_cells = evidence["support_radius_cells"].as<std::uint32_t>(
        tp->obstacle_support_radius_cells);
    tp->free_requires_no_hits =
        evidence["free_requires_no_hits"].as<bool>(tp->free_requires_no_hits);
    tp->obstacle_min_observation_range =
        evidence["min_observation_range"].as<float>(tp->obstacle_min_observation_range);
  }
  if (const YAML::Node sweep = node["footprint_sweep"]) {
    tp->sweep_enabled = sweep["enabled"].as<bool>(tp->sweep_enabled);
    tp->sweep_padding = sweep["padding"].as<float>(tp->sweep_padding);
  }
  tp->hole_fill_max_area = node["hole_fill_max_area"].as<float>(tp->hole_fill_max_area);
  tp->max_keyframe_tilt_deg =
      node["max_keyframe_tilt_deg"].as<float>(tp->max_keyframe_tilt_deg);
  tp->debug_layers = node["debug_layers"].as<bool>(tp->debug_layers);
}

bool traversability_valid(const ProjectionParameters &parameters) {
  const auto &tp = parameters.traversability;
  const auto finite = [](float value) { return std::isfinite(value); };
  for (const float value : parameters.robot.base_from_body_xyz) {
    if (!finite(value)) return false;
  }
  for (const float value : parameters.robot.base_from_body_rpy) {
    if (!finite(value)) return false;
  }
  for (const auto &[x, y] : parameters.robot.footprint) {
    if (!finite(x) || !finite(y)) return false;
  }
  return parameters.robot.footprint.size() >= 3U && tp.self_filter_margin >= 0.0F &&
         tp.self_filter_min_height < tp.self_filter_max_height &&
         tp.ground_cell_size >= parameters.resolution && tp.ground_percentile >= 0.0F &&
         tp.ground_percentile <= 1.0F && tp.ground_max_step >= 0.0F &&
         tp.ground_fill_max_distance >= 0.0F && tp.ground_tolerance >= 0.0F &&
         tp.ground_reference_extension >= 0.0F && finite(tp.ground_reference_extension) &&
         tp.raycast_max_range > 0.0F && tp.carve_max_height > -tp.ground_tolerance &&
         tp.free_min_passes >= 1U && tp.obstacle_support_radius_cells <= 5U &&
         tp.obstacle_min_observation_range >= 0.0F &&
         finite(tp.obstacle_min_observation_range) &&
         finite(tp.sweep_padding) && tp.sweep_padding <= 0.5F && tp.hole_fill_max_area >= 0.0F &&
         tp.max_keyframe_tilt_deg > 0.0F && tp.max_keyframe_tilt_deg <= 180.0F &&
         finite(tp.ground_cell_size) && finite(tp.ground_max_step) &&
         finite(tp.ground_fill_max_distance) && finite(tp.raycast_max_range) &&
         finite(tp.carve_max_height) && finite(tp.hole_fill_max_area);
}

}  // namespace

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
    } else if (mode == "traversability") {
      parameters->projection_mode = ProjectionMode::Traversability;
    } else {
      if (error) {
        *error = "projection_mode must be 'local_ground', 'fixed_height' or 'traversability'";
      }
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
    if (!load_robot(root["robot"], &parameters->robot, error)) return false;
    load_traversability(root["traversability"], &parameters->traversability);
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
    if (parameters->projection_mode == ProjectionMode::Traversability &&
        !traversability_valid(*parameters)) {
      if (error) *error = "projection.yaml contains invalid robot/traversability values";
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
            << mode_name(parameters.projection_mode)
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
            << (parameters.empty_cell == EmptyCellPolicy::Free ? "free" : "unknown");
    if (parameters.projection_mode == ProjectionMode::Traversability) {
      const auto &robot = parameters.robot;
      const auto &tp = parameters.traversability;
      emitter << YAML::Key << "robot" << YAML::Value << YAML::BeginMap
              << YAML::Key << "base_from_body" << YAML::Value << YAML::BeginMap
              << YAML::Key << "xyz" << YAML::Value << YAML::Flow << YAML::BeginSeq
              << robot.base_from_body_xyz[0] << robot.base_from_body_xyz[1]
              << robot.base_from_body_xyz[2] << YAML::EndSeq
              << YAML::Key << "rpy" << YAML::Value << YAML::Flow << YAML::BeginSeq
              << robot.base_from_body_rpy[0] << robot.base_from_body_rpy[1]
              << robot.base_from_body_rpy[2] << YAML::EndSeq << YAML::EndMap
              << YAML::Key << "footprint" << YAML::Value << YAML::Flow << YAML::BeginSeq;
      for (const auto &[x, y] : robot.footprint) {
        emitter << YAML::Flow << YAML::BeginSeq << x << y << YAML::EndSeq;
      }
      emitter << YAML::EndSeq << YAML::EndMap;
      emitter << YAML::Key << "traversability" << YAML::Value << YAML::BeginMap
              << YAML::Key << "self_filter" << YAML::Value << YAML::BeginMap
              << YAML::Key << "enabled" << YAML::Value << tp.self_filter_enabled
              << YAML::Key << "margin" << YAML::Value << tp.self_filter_margin
              << YAML::Key << "min_height" << YAML::Value << tp.self_filter_min_height
              << YAML::Key << "max_height" << YAML::Value << tp.self_filter_max_height
              << YAML::EndMap
              << YAML::Key << "ground" << YAML::Value << YAML::BeginMap
              << YAML::Key << "cell_size" << YAML::Value << tp.ground_cell_size
              << YAML::Key << "percentile" << YAML::Value << tp.ground_percentile
              << YAML::Key << "max_step" << YAML::Value << tp.ground_max_step
              << YAML::Key << "fill_max_distance" << YAML::Value << tp.ground_fill_max_distance
              << YAML::Key << "tolerance" << YAML::Value << tp.ground_tolerance
              << YAML::Key << "reference_extension" << YAML::Value
              << tp.ground_reference_extension
              << YAML::EndMap
              << YAML::Key << "raycast" << YAML::Value << YAML::BeginMap
              << YAML::Key << "enabled" << YAML::Value << tp.raycast_enabled
              << YAML::Key << "max_range" << YAML::Value << tp.raycast_max_range
              << YAML::Key << "carve_max_height" << YAML::Value << tp.carve_max_height
              << YAML::Key << "end_margin_cells" << YAML::Value << tp.carve_end_margin_cells
              << YAML::Key << "free_min_passes" << YAML::Value << tp.free_min_passes
              << YAML::EndMap
              << YAML::Key << "obstacle_evidence" << YAML::Value << YAML::BeginMap
              << YAML::Key << "support_radius_cells" << YAML::Value
              << tp.obstacle_support_radius_cells
              << YAML::Key << "free_requires_no_hits" << YAML::Value << tp.free_requires_no_hits
              << YAML::Key << "min_observation_range" << YAML::Value
              << tp.obstacle_min_observation_range
              << YAML::EndMap
              << YAML::Key << "footprint_sweep" << YAML::Value << YAML::BeginMap
              << YAML::Key << "enabled" << YAML::Value << tp.sweep_enabled
              << YAML::Key << "padding" << YAML::Value << tp.sweep_padding
              << YAML::EndMap
              << YAML::Key << "hole_fill_max_area" << YAML::Value << tp.hole_fill_max_area
              << YAML::Key << "max_keyframe_tilt_deg" << YAML::Value << tp.max_keyframe_tilt_deg
              << YAML::Key << "debug_layers" << YAML::Value << tp.debug_layers
              << YAML::EndMap;
    }
    emitter << YAML::EndMap;
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
