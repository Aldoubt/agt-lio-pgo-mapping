#include "agt_pcd2grid_exporter/PCDProjector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <queue>
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

struct Point3 {
  float x;
  float y;
  float z;
};

std::size_t remove_small_components(std::vector<std::uint8_t> *mask,
                                    std::uint32_t width, std::uint32_t height,
                                    std::uint32_t minimum_size) {
  if (!mask || minimum_size <= 1U) return 0U;
  std::vector<std::uint8_t> visited(mask->size(), 0U);
  std::size_t removed = 0U;
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      const std::size_t start = static_cast<std::size_t>(y) * width + x;
      if (!(*mask)[start] || visited[start]) continue;
      std::vector<std::size_t> component;
      std::queue<std::pair<std::uint32_t, std::uint32_t>> pending;
      pending.push({x, y});
      visited[start] = 1U;
      while (!pending.empty()) {
        const auto [cx, cy] = pending.front();
        pending.pop();
        component.push_back(static_cast<std::size_t>(cy) * width + cx);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) continue;
            const auto nx = static_cast<std::int64_t>(cx) + dx;
            const auto ny = static_cast<std::int64_t>(cy) + dy;
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
            const std::size_t index = static_cast<std::size_t>(ny) * width +
                                      static_cast<std::size_t>(nx);
            if ((*mask)[index] && !visited[index]) {
              visited[index] = 1U;
              pending.push({static_cast<std::uint32_t>(nx),
                            static_cast<std::uint32_t>(ny)});
            }
          }
        }
      }
      if (component.size() < minimum_size) {
        removed += component.size();
        for (const auto index : component) (*mask)[index] = 0U;
      }
    }
  }
  return removed;
}

std::vector<std::uint8_t> close_mask(const std::vector<std::uint8_t> &source,
                                     std::uint32_t width, std::uint32_t height,
                                     std::uint32_t radius) {
  if (radius == 0U) return source;
  std::vector<std::uint8_t> dilated(source.size(), 0U);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      bool value = false;
      for (int dy = -static_cast<int>(radius); dy <= static_cast<int>(radius) && !value; ++dy) {
        for (int dx = -static_cast<int>(radius); dx <= static_cast<int>(radius); ++dx) {
          const auto nx = static_cast<std::int64_t>(x) + dx;
          const auto ny = static_cast<std::int64_t>(y) + dy;
          if (nx >= 0 && ny >= 0 && nx < width && ny < height &&
              source[static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx)]) {
            value = true;
            break;
          }
        }
      }
      dilated[static_cast<std::size_t>(y) * width + x] = value ? 1U : 0U;
    }
  }
  std::vector<std::uint8_t> closed(source.size(), 0U);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      bool value = true;
      for (int dy = -static_cast<int>(radius); dy <= static_cast<int>(radius) && value; ++dy) {
        for (int dx = -static_cast<int>(radius); dx <= static_cast<int>(radius); ++dx) {
          const auto nx = static_cast<std::int64_t>(x) + dx;
          const auto ny = static_cast<std::int64_t>(y) + dy;
          if (nx < 0 || ny < 0 || nx >= width || ny >= height ||
              !dilated[static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx)]) {
            value = false;
            break;
          }
        }
      }
      closed[static_cast<std::size_t>(y) * width + x] = value ? 1U : 0U;
    }
  }
  return closed;
}

std::vector<std::uint8_t> dilate_mask(const std::vector<std::uint8_t> &source,
                                      std::uint32_t width, std::uint32_t height,
                                      std::uint32_t radius) {
  if (radius == 0U) return source;
  std::vector<std::uint8_t> result(source.size(), 0U);
  for (std::uint32_t y = 0; y < height; ++y) {
    for (std::uint32_t x = 0; x < width; ++x) {
      bool value = false;
      for (int dy = -static_cast<int>(radius); dy <= static_cast<int>(radius) && !value; ++dy) {
        for (int dx = -static_cast<int>(radius); dx <= static_cast<int>(radius); ++dx) {
          const auto nx = static_cast<std::int64_t>(x) + dx;
          const auto ny = static_cast<std::int64_t>(y) + dy;
          if (nx >= 0 && ny >= 0 && nx < width && ny < height &&
              source[static_cast<std::size_t>(ny) * width + static_cast<std::size_t>(nx)]) {
            value = true;
            break;
          }
        }
      }
      result[static_cast<std::size_t>(y) * width + x] = value ? 1U : 0U;
    }
  }
  return result;
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
      parameters.occupied_threshold == 0U || parameters.free_threshold == 0U ||
      !(parameters.ground_cell_size > 0.0F) ||
      parameters.ground_percentile < 0.0F || parameters.ground_percentile > 1.0F ||
      parameters.ground_free_tolerance < 0.0F ||
      parameters.obstacle_min_height < 0.0F ||
      parameters.obstacle_min_height > parameters.obstacle_max_height) {
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

  std::vector<Point3> finite_points;
  finite_points.reserve(point_count);
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
      finite_points.push_back({x, y, z});
    }
  }

  if (finite_points.empty()) {
    if (error) *error = "no finite XYZ points remain";
    return false;
  }

  std::vector<std::pair<float, float>> obstacle_xy;
  std::vector<std::pair<float, float>> ground_xy;
  obstacle_xy.reserve(finite_points.size());
  ground_xy.reserve(finite_points.size());
  if (parameters.projection_mode == ProjectionMode::FixedHeight) {
    for (const auto &point : finite_points) {
      if (point.z < parameters.z_min || point.z > parameters.z_max) {
        ++stats->z_filtered_points;
        continue;
      }
      obstacle_xy.emplace_back(point.x, point.y);
      ++stats->obstacle_points;
    }
  } else {
    float min_x = finite_points.front().x;
    float max_x = min_x;
    float min_y = finite_points.front().y;
    float max_y = min_y;
    for (const auto &point : finite_points) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }
    const std::uint32_t ground_width = static_cast<std::uint32_t>(
        std::floor((max_x - min_x) / parameters.ground_cell_size) + 1.0F);
    const std::uint32_t ground_height = static_cast<std::uint32_t>(
        std::floor((max_y - min_y) / parameters.ground_cell_size) + 1.0F);
    const std::size_t ground_size = static_cast<std::size_t>(ground_width) * ground_height;
    std::vector<std::vector<float>> samples(ground_size);
    const auto ground_index = [&](const Point3 &point) {
      const auto gx = static_cast<std::uint32_t>(
          std::floor((point.x - min_x) / parameters.ground_cell_size));
      const auto gy = static_cast<std::uint32_t>(
          std::floor((point.y - min_y) / parameters.ground_cell_size));
      return static_cast<std::size_t>(gy) * ground_width + gx;
    };
    for (const auto &point : finite_points) samples[ground_index(point)].push_back(point.z);
    const float missing = std::numeric_limits<float>::quiet_NaN();
    std::vector<float> estimates(ground_size, missing);
    for (std::size_t index = 0; index < samples.size(); ++index) {
      auto &values = samples[index];
      if (values.empty()) continue;
      const std::size_t selected = static_cast<std::size_t>(
          parameters.ground_percentile * static_cast<float>(values.size() - 1U));
      std::nth_element(values.begin(), values.begin() + selected, values.end());
      estimates[index] = values[selected];
    }
    std::vector<float> smoothed(ground_size, missing);
    const int radius = static_cast<int>(parameters.ground_neighbor_radius);
    for (std::uint32_t gy = 0; gy < ground_height; ++gy) {
      for (std::uint32_t gx = 0; gx < ground_width; ++gx) {
        std::vector<float> neighbors;
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            const auto nx = static_cast<std::int64_t>(gx) + dx;
            const auto ny = static_cast<std::int64_t>(gy) + dy;
            if (nx < 0 || ny < 0 || nx >= ground_width || ny >= ground_height) continue;
            const float value = estimates[static_cast<std::size_t>(ny) * ground_width +
                                          static_cast<std::size_t>(nx)];
            if (std::isfinite(value)) neighbors.push_back(value);
          }
        }
        if (neighbors.empty()) continue;
        // Lower quartile rejects obstacle-only tiles while remaining less
        // sensitive to a single low outlier than a neighborhood minimum.
        const std::size_t selected = (neighbors.size() - 1U) / 4U;
        std::nth_element(neighbors.begin(), neighbors.begin() + selected, neighbors.end());
        smoothed[static_cast<std::size_t>(gy) * ground_width + gx] = neighbors[selected];
      }
    }
    for (const auto &point : finite_points) {
      const float ground = smoothed[ground_index(point)];
      if (!std::isfinite(ground)) {
        ++stats->z_filtered_points;
        continue;
      }
      const float relative_height = point.z - ground;
      if (std::abs(relative_height) <= parameters.ground_free_tolerance) {
        ground_xy.emplace_back(point.x, point.y);
        ++stats->ground_points;
      } else if (relative_height >= parameters.obstacle_min_height &&
                 relative_height <= parameters.obstacle_max_height) {
        obstacle_xy.emplace_back(point.x, point.y);
        ++stats->obstacle_points;
      } else {
        ++stats->z_filtered_points;
      }
    }
  }

  std::vector<std::pair<float, float>> accepted_xy = ground_xy;
  accepted_xy.insert(accepted_xy.end(), obstacle_xy.begin(), obstacle_xy.end());
  stats->accepted_points = accepted_xy.size();
  if (accepted_xy.empty()) {
    if (error) *error = "no points remain after projection classification";
    return false;
  }
  stats->accepted_x_min = stats->accepted_x_max = accepted_xy.front().first;
  stats->accepted_y_min = stats->accepted_y_max = accepted_xy.front().second;
  for (const auto &[x, y] : accepted_xy) {
    stats->accepted_x_min = std::min(stats->accepted_x_min, x);
    stats->accepted_x_max = std::max(stats->accepted_x_max, x);
    stats->accepted_y_min = std::min(stats->accepted_y_min, y);
    stats->accepted_y_max = std::max(stats->accepted_y_max, y);
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
  grid->free_count.assign(static_cast<std::size_t>(cell_count), 0U);
  const auto rasterize = [&](const std::vector<std::pair<float, float>> &points,
                             std::vector<std::uint32_t> *counts) {
    for (const auto &[x, y] : points) {
    const auto gx = static_cast<long long>(std::floor(
        (static_cast<double>(x) - grid->origin_x) / parameters.resolution));
    const auto gy = static_cast<long long>(std::floor(
        (static_cast<double>(y) - grid->origin_y) / parameters.resolution));
    if (gx < 0 || gy < 0 || gx >= grid->width || gy >= grid->height) continue;
      ++(*counts)[static_cast<std::size_t>(gy) * grid->width +
                  static_cast<std::size_t>(gx)];
    }
  };
  rasterize(obstacle_xy, &grid->hit_count);
  rasterize(ground_xy, &grid->free_count);

  std::vector<std::uint8_t> occupied(grid->size(), 0U);
  for (std::size_t index = 0; index < occupied.size(); ++index) {
    occupied[index] = grid->hit_count[index] >= parameters.occupied_threshold ? 1U : 0U;
  }
  stats->removed_small_component_cells += remove_small_components(
      &occupied, grid->width, grid->height, parameters.min_component_cells);
  const auto before_closing = occupied;
  occupied = close_mask(occupied, grid->width, grid->height,
                        parameters.closing_radius_cells);
  for (std::size_t index = 0; index < occupied.size(); ++index) {
    if (occupied[index] && !before_closing[index]) ++stats->closing_added_cells;
  }
  stats->removed_small_component_cells += remove_small_components(
      &occupied, grid->width, grid->height, parameters.min_component_cells);

  std::vector<std::uint8_t> free_space(grid->size(), 0U);
  for (std::size_t index = 0; index < free_space.size(); ++index) {
    free_space[index] = grid->free_count[index] >= parameters.free_threshold ? 1U : 0U;
  }
  const auto raw_free_space = free_space;
  free_space = dilate_mask(free_space, grid->width, grid->height,
                           parameters.free_space_radius_cells);
  for (std::size_t index = 0; index < free_space.size(); ++index) {
    if (free_space[index] && !raw_free_space[index]) ++stats->free_expanded_cells;
  }

  grid->occupancy.assign(grid->size(), -1);
  for (std::size_t index = 0; index < grid->size(); ++index) {
    if (occupied[index]) {
      grid->occupancy[index] = 100;
      ++stats->occupied_cells;
    } else if (free_space[index] ||
               parameters.empty_cell == EmptyCellPolicy::Free) {
      grid->occupancy[index] = 0;
      ++stats->free_cells;
    } else {
      ++stats->unknown_cells;
    }
  }
  stats->empty_cells = stats->free_cells + stats->unknown_cells;
  return true;
}

}  // namespace agt_pcd2grid_exporter
