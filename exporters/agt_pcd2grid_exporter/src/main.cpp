#include "agt_pcd2grid_exporter/OccupancyGridWriter.hpp"
#include "agt_pcd2grid_exporter/PCDProjector.hpp"
#include "agt_pcd2grid_exporter/ParameterLoader.hpp"
#include "agt_pcd2grid_exporter/TemporalPersistenceFilter.hpp"

#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <iostream>
#include <string>

namespace {

void usage(const char *program) {
  std::cerr << "Usage: " << program
            << " (--pcd input.pcd | --package mapping_package)"
               " --output navigation_map [--config projection.yaml]\n";
}

}  // namespace

int main(int argc, char **argv) {
  std::string pcd_path;
  std::string package_path;
  std::string output_dir;
  std::string config_path;
  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    if ((argument == "--pcd" || argument == "--package" || argument == "--output" ||
         argument == "--config") &&
        i + 1 < argc) {
      const std::string value(argv[++i]);
      if (argument == "--pcd") pcd_path = value;
      else if (argument == "--package") package_path = value;
      else if (argument == "--output") output_dir = value;
      else config_path = value;
    } else if (argument == "--help" || argument == "-h") {
      usage(argv[0]);
      return 0;
    } else {
      usage(argv[0]);
      return 2;
    }
  }
  if ((pcd_path.empty() == package_path.empty()) || output_dir.empty()) {
    usage(argv[0]);
    return 2;
  }

  agt_pcd2grid_exporter::ProjectionParameters parameters;
  std::string error;
  if (!config_path.empty() &&
      !agt_pcd2grid_exporter::ParameterLoader::load(config_path, &parameters, &error)) {
    std::cerr << "Parameter error: " << error << '\n';
    return 1;
  }
  pcl::PCLPointCloud2 cloud;
  agt_pcd2grid_exporter::TemporalFilterStats temporal_stats;
  std::string source_path = pcd_path;
  if (!package_path.empty()) {
    source_path = package_path;
    if (parameters.temporal_filter_enabled) {
      if (!agt_pcd2grid_exporter::TemporalPersistenceFilter::filter_package(
              package_path, parameters, &cloud, &temporal_stats, &error)) {
        std::cerr << "Temporal filter error: " << error << '\n';
        return 1;
      }
    } else {
      pcd_path = (std::filesystem::path(package_path) / "map.pcd").string();
    }
  }
  if (cloud.data.empty() && pcl::io::loadPCDFile(pcd_path, cloud) != 0) {
      std::cerr << "Could not load PCD: " << pcd_path << '\n';
      return 1;
  }
  agt_pcd2grid_exporter::OccupancyGrid grid;
  agt_pcd2grid_exporter::ProjectionStats stats;
  if (!agt_pcd2grid_exporter::PCDProjector::project(
          cloud, parameters, &grid, &stats, &error)) {
    std::cerr << "Projection error: " << error << '\n';
    return 1;
  }
  stats.temporal_input_points = temporal_stats.input_points;
  stats.temporal_retained_points = temporal_stats.retained_points;
  stats.temporal_removed_points = temporal_stats.removed_points;
  if (!agt_pcd2grid_exporter::OccupancyGridWriter::write_navigation_map(
          grid, parameters, stats, output_dir, source_path, &error)) {
    std::cerr << "Write error: " << error << '\n';
    return 1;
  }
  std::cout << "Generated " << output_dir << " (" << grid.width << "x"
            << grid.height << ", accepted points=" << stats.accepted_points
            << ", occupied cells=" << stats.occupied_cells
            << ", free cells=" << stats.free_cells;
  if (temporal_stats.input_points > 0U) {
    std::cout << ", temporal points=" << temporal_stats.retained_points << "/"
              << temporal_stats.input_points;
  }
  std::cout << ")\n";
  return 0;
}
