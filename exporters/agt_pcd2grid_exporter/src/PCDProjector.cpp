#include "agt_pcd2grid_exporter/PCDProjector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

namespace agt_pcd2grid_exporter {
namespace {

const pcl::PCLPointField *find_field(const pcl::PCLPointCloud2 &cloud,
                                     const char *name) {
  for (const auto &field : cloud.fields) {
    if (field.name == name) return &field;
  }
  return nullptr;
}

std::size_t datatype_size(std::uint8_t datatype) {
  switch (datatype) {
    case pcl::PCLPointField::INT8:
    case pcl::PCLPointField::UINT8: return 1U;
    case pcl::PCLPointField::INT16:
    case pcl::PCLPointField::UINT16: return 2U;
    case pcl::PCLPointField::INT32:
    case pcl::PCLPointField::UINT32:
    case pcl::PCLPointField::FLOAT32: return 4U;
    case pcl::PCLPointField::FLOAT64: return 8U;
    default: return 0U;
  }
}

bool read_float(const std::uint8_t *address, const pcl::PCLPointField &field,
                float *value) {
  if (!address || !value) return false;
  if (field.datatype == pcl::PCLPointField::FLOAT32) {
    std::memcpy(value, address, sizeof(float));
    return true;
  }
  if (field.datatype == pcl::PCLPointField::FLOAT64) {
    double converted = 0.0;
    std::memcpy(&converted, address, sizeof(double));
    *value = static_cast<float>(converted);
    return true;
  }
  return false;
}

}  // namespace

bool PCDProjector::project(const pcl::PCLPointCloud2 &cloud,
                           const ProjectionParameters &parameters,
                           OccupancyGrid *grid, ProjectionStats *stats,
                           std::string *error) {
  if (!grid || !stats) {
    if (error) *error = "grid and stats must not be null";
    return false;
  }
  *grid = OccupancyGrid();
  *stats = ProjectionStats();
  if (!(parameters.resolution > 0.0F) ||
      !(parameters.z_min <= parameters.z_max) ||
      parameters.occupied_threshold == 0U) {
    if (error) *error = "resolution, z range or occupied_threshold is invalid";
    return false;
  }

  const std::size_t width = cloud.width;
  const std::size_t height = cloud.height == 0U ? 1U : cloud.height;
  const std::size_t point_count = width * height;
  const std::size_t point_step = cloud.point_step;
  const std::size_t row_step = cloud.row_step == 0U ? width * point_step
                                                     : cloud.row_step;
  if (point_count == 0U || point_step == 0U || cloud.data.empty()) {
    if (error) *error = "PCD contains no point data";
    return false;
  }

  const auto *x_field = find_field(cloud, "x");
  const auto *y_field = find_field(cloud, "y");
  const auto *z_field = find_field(cloud, "z");
  if (!x_field || !y_field || !z_field) {
    if (error) *error = "PCD must contain x, y and z fields";
    return false;
  }
  const std::size_t x_size = datatype_size(x_field->datatype);
  const std::size_t y_size = datatype_size(y_field->datatype);
  const std::size_t z_size = datatype_size(z_field->datatype);
  if (!x_size || !y_size || !z_size ||
      x_field->offset + x_size > cloud.point_step ||
      y_field->offset + y_size > cloud.point_step ||
      z_field->offset + z_size > cloud.point_step) {
    if (error) *error = "XYZ field exceeds PCD point record";
    return false;
  }

  std::vector<std::pair<float, float>> accepted_xy;
  accepted_xy.reserve(point_count);
  bool have_bounds = false;
  for (std::size_t row = 0; row < height; ++row) {
    for (std::size_t col = 0; col < width; ++col) {
      ++stats->input_points;
      const std::size_t offset = row * row_step + col * point_step;
      if (offset + point_step > cloud.data.size()) {
        if (error) *error = "PCD row/point stride exceeds data buffer";
        return false;
      }
      const auto *base = cloud.data.data() + offset;
      float x = 0.0F;
      float y = 0.0F;
      float z = 0.0F;
      if (!read_float(base + x_field->offset, *x_field, &x) ||
          !read_float(base + y_field->offset, *y_field, &y) ||
          !read_float(base + z_field->offset, *z_field, &z) ||
          !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++stats->z_filtered_points;
        continue;
      }
      if (z < parameters.z_min || z > parameters.z_max) {
        ++stats->z_filtered_points;
        continue;
      }
      accepted_xy.emplace_back(x, y);
      ++stats->accepted_points;
      if (!have_bounds) {
        stats->accepted_x_min = stats->accepted_x_max = x;
        stats->accepted_y_min = stats->accepted_y_max = y;
        have_bounds = true;
      } else {
        stats->accepted_x_min = std::min(stats->accepted_x_min, x);
        stats->accepted_x_max = std::max(stats->accepted_x_max, x);
        stats->accepted_y_min = std::min(stats->accepted_y_min, y);
        stats->accepted_y_max = std::max(stats->accepted_y_max, y);
      }
    }
  }

  if (!have_bounds) {
    if (error) *error = "no finite points remain after z_filter";
    return false;
  }
  if (parameters.origin_auto) {
    grid->origin_x = std::floor(stats->accepted_x_min / parameters.resolution) *
                     parameters.resolution;
    grid->origin_y = std::floor(stats->accepted_y_min / parameters.resolution) *
                     parameters.resolution;
  } else {
    grid->origin_x = parameters.origin_x;
    grid->origin_y = parameters.origin_y;
  }
  grid->resolution = parameters.resolution;
  const double width_cells = std::floor(
      (static_cast<double>(stats->accepted_x_max) - grid->origin_x) /
      parameters.resolution) + 1.0;
  const double height_cells = std::floor(
      (static_cast<double>(stats->accepted_y_max) - grid->origin_y) /
      parameters.resolution) + 1.0;
  if (width_cells < 1.0 || height_cells < 1.0 ||
      width_cells > std::numeric_limits<std::uint32_t>::max() ||
      height_cells > std::numeric_limits<std::uint32_t>::max()) {
    if (error) *error = "computed grid dimensions are invalid";
    return false;
  }
  grid->width = static_cast<std::uint32_t>(width_cells);
  grid->height = static_cast<std::uint32_t>(height_cells);
  const std::uint64_t cell_count = static_cast<std::uint64_t>(grid->width) *
                                   static_cast<std::uint64_t>(grid->height);
  if (cell_count > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max())) {
    if (error) *error = "computed grid is too large";
    return false;
  }
  grid->hit_count.assign(static_cast<std::size_t>(cell_count), 0U);
  for (const auto &[x, y] : accepted_xy) {
    const auto gx = static_cast<long long>(std::floor(
        (static_cast<double>(x) - grid->origin_x) / parameters.resolution));
    const auto gy = static_cast<long long>(std::floor(
        (static_cast<double>(y) - grid->origin_y) / parameters.resolution));
    if (gx < 0 || gy < 0 || gx >= grid->width || gy >= grid->height) continue;
    ++grid->hit_count[static_cast<std::size_t>(gy) * grid->width +
                       static_cast<std::size_t>(gx)];
  }
  for (const auto hits : grid->hit_count) {
    if (hits >= parameters.occupied_threshold) ++stats->occupied_cells;
    else ++stats->empty_cells;
  }
  return true;
}

}  // namespace agt_pcd2grid_exporter
