#include "agt_pcd2grid_exporter/OccupancyGridWriter.hpp"

#include "agt_pcd2grid_exporter/ParameterLoader.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <sstream>

namespace agt_pcd2grid_exporter {

bool OccupancyGridWriter::write_navigation_map(
    const OccupancyGrid &grid, const ProjectionParameters &parameters,
    const ProjectionStats &stats, const std::string &output_dir,
    const std::string &source_pcd, std::string *error) {
  if (grid.width == 0U || grid.height == 0U ||
      grid.hit_count.size() != static_cast<std::size_t>(grid.width) * grid.height) {
    if (error) *error = "occupancy grid is empty or inconsistent";
    return false;
  }
  try {
    std::filesystem::create_directories(output_dir);
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
  const std::filesystem::path directory(output_dir);
  const auto pgm_path = directory / "map.pgm";
  std::ofstream pgm(pgm_path, std::ios::binary);
  if (!pgm) {
    if (error) *error = "cannot write " + pgm_path.string();
    return false;
  }
  pgm << "P5\n" << grid.width << ' ' << grid.height << "\n255\n";
  for (std::int64_t y = static_cast<std::int64_t>(grid.height) - 1; y >= 0; --y) {
    for (std::uint32_t x = 0; x < grid.width; ++x) {
      const std::size_t index = static_cast<std::size_t>(y) * grid.width + x;
      const std::int8_t value = grid.value(index, parameters);
      const unsigned char pixel = value == 100 ? 0U : (value == 0 ? 254U : 205U);
      pgm.write(reinterpret_cast<const char *>(&pixel), 1);
    }
  }
  if (!pgm.good()) {
    if (error) *error = "failed writing " + pgm_path.string();
    return false;
  }

  try {
    YAML::Emitter map_yaml;
    map_yaml << YAML::BeginMap << YAML::Key << "image" << YAML::Value << "map.pgm"
             << YAML::Key << "resolution" << YAML::Value << grid.resolution
             << YAML::Key << "origin" << YAML::Value << YAML::Flow << YAML::BeginSeq
             << grid.origin_x << grid.origin_y << 0.0F << YAML::EndSeq
             << YAML::Key << "occupied_thresh" << YAML::Value << 0.65
             << YAML::Key << "free_thresh" << YAML::Value << 0.196
             << YAML::Key << "negate" << YAML::Value << 0
             << YAML::Key << "mode" << YAML::Value << "trinary" << YAML::EndMap;
    std::ofstream map_stream(directory / "map.yaml");
    map_stream << map_yaml.c_str() << '\n';

    std::string yaml_error;
    if (!ParameterLoader::write((directory / "projection.yaml").string(), parameters,
                                &yaml_error)) {
      if (error) *error = yaml_error;
      return false;
    }

    YAML::Emitter metadata;
    metadata << YAML::BeginMap << YAML::Key << "source_pcd" << YAML::Value << source_pcd
             << YAML::Key << "input_points" << YAML::Value << stats.input_points
             << YAML::Key << "accepted_points" << YAML::Value << stats.accepted_points
             << YAML::Key << "z_filtered_points" << YAML::Value << stats.z_filtered_points
             << YAML::Key << "ground_points" << YAML::Value << stats.ground_points
             << YAML::Key << "obstacle_points" << YAML::Value << stats.obstacle_points
             << YAML::Key << "width" << YAML::Value << grid.width
             << YAML::Key << "height" << YAML::Value << grid.height
             << YAML::Key << "occupied_cells" << YAML::Value << stats.occupied_cells
             << YAML::Key << "free_cells" << YAML::Value << stats.free_cells
             << YAML::Key << "unknown_cells" << YAML::Value << stats.unknown_cells
             << YAML::Key << "empty_cells" << YAML::Value << stats.empty_cells
             << YAML::Key << "removed_small_component_cells" << YAML::Value
             << stats.removed_small_component_cells << YAML::Key << "closing_added_cells"
             << YAML::Value << stats.closing_added_cells << YAML::Key << "free_expanded_cells"
             << YAML::Value << stats.free_expanded_cells << YAML::Key
             << "temporal_input_points" << YAML::Value << stats.temporal_input_points
             << YAML::Key << "temporal_retained_points" << YAML::Value
             << stats.temporal_retained_points << YAML::Key << "temporal_removed_points"
             << YAML::Value << stats.temporal_removed_points
             << YAML::Key << "coordinate_convention" << YAML::Value
             << "origin is lower-left; PGM rows are vertically flipped" << YAML::EndMap;
    std::ofstream metadata_stream(directory / "metadata.yaml");
    metadata_stream << metadata.c_str() << '\n';
    if (!map_stream.good() || !metadata_stream.good()) {
      if (error) *error = "failed writing navigation map metadata";
      return false;
    }
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
  return true;
}

}  // namespace agt_pcd2grid_exporter
