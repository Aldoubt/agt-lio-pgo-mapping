#include "occupancy/GridMap.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace agt_map_studio {

void GridMap::clear() {
  width_ = 0;
  height_ = 0;
  cells_.clear();
  yaml_path_.clear();
  image_path_.clear();
}

bool GridMap::set_geometry(std::uint32_t width, std::uint32_t height,
                           float resolution, double origin_x, double origin_y,
                           std::string *error) {
  if (width == 0U || height == 0U || !(resolution > 0.0F)) {
    if (error) *error = "grid width, height and resolution must be positive";
    return false;
  }
  const std::uint64_t count = static_cast<std::uint64_t>(width) * height;
  if (count > std::numeric_limits<std::size_t>::max()) {
    if (error) *error = "grid is too large";
    return false;
  }
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  cells_.assign(static_cast<std::size_t>(count), kUnknown);
  return true;
}

void GridMap::set_source_paths(std::string yaml_path, std::string image_path) {
  yaml_path_ = std::move(yaml_path);
  image_path_ = std::move(image_path);
}

std::int8_t GridMap::at(std::uint32_t pixel_x, std::uint32_t pixel_y) const {
  if (pixel_x >= width_ || pixel_y >= height_) return kUnknown;
  return cells_[static_cast<std::size_t>(pixel_y) * width_ + pixel_x];
}

GridWorldPoint GridMap::pixel_to_world(int pixel_x, int pixel_y) const {
  return {origin_x_ + static_cast<double>(pixel_x) * resolution_,
          origin_y_ + static_cast<double>(pixel_y) * resolution_};
}

bool GridMap::world_to_pixel(double world_x, double world_y, int *pixel_x,
                             int *pixel_y) const {
  if (!pixel_x || !pixel_y || empty()) return false;
  const double x = std::floor((world_x - origin_x_) / resolution_);
  const double y = std::floor((world_y - origin_y_) / resolution_);
  if (x < 0.0 || y < 0.0 || x >= width_ || y >= height_ ||
      x > std::numeric_limits<int>::max() || y > std::numeric_limits<int>::max()) {
    return false;
  }
  *pixel_x = static_cast<int>(x);
  *pixel_y = static_cast<int>(y);
  return true;
}

}  // namespace agt_map_studio
