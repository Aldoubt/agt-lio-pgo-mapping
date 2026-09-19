#include "agt_pcd2grid_exporter/OccupancyGridWriter.hpp"
#include "agt_pcd2grid_exporter/PCDProjector.hpp"
#include "agt_pcd2grid_exporter/ParameterLoader.hpp"

#include <pcl/io/pcd_io.h>

#include <iostream>
#include <string>

namespace {

void usage(const char *program) {
  std::cerr << "Usage: " << program
            << " --pcd input.pcd --output navigation_map [--config projection.yaml]\n";
}

}  // namespace

int main(int argc, char **argv) {
  std::string pcd_path;
  std::string output_dir;
  std::string config_path;
  for (int i = 1; i < argc; ++i) {
    const std::string argument(argv[i]);
    if ((argument == "--pcd" || argument == "--output" || argument == "--config") &&
        i + 1 < argc) {
      const std::string value(argv[++i]);
      if (argument == "--pcd") pcd_path = value;
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
  if (pcd_path.empty() || output_dir.empty()) {
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
  if (pcl::io::loadPCDFile(pcd_path, cloud) != 0) {
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
  if (!agt_pcd2grid_exporter::OccupancyGridWriter::write_navigation_map(
          grid, parameters, stats, output_dir, pcd_path, &error)) {
    std::cerr << "Write error: " << error << '\n';
    return 1;
  }
  std::cout << "Generated " << output_dir << " (" << grid.width << "x"
            << grid.height << ", accepted points=" << stats.accepted_points
            << ", occupied cells=" << stats.occupied_cells << ")\n";
  return 0;
}
