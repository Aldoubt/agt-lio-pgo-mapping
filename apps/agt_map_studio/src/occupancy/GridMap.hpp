#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace agt_map_studio {

struct GridWorldPoint {
  double x = 0.0;
  double y = 0.0;
};

class GridMap {
public:
  static constexpr std::int8_t kUnknown = -1;
  static constexpr std::int8_t kFree = 0;
  static constexpr std::int8_t kOccupied = 100;

  void clear();
  bool empty() const { return cells_.empty(); }
  std::uint32_t width() const { return width_; }
  std::uint32_t height() const { return height_; }
  float resolution() const { return resolution_; }
  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }
  const std::vector<std::int8_t> &cells() const { return cells_; }
  std::vector<std::int8_t> &cells() { return cells_; }
  const std::string &yaml_path() const { return yaml_path_; }
  const std::string &image_path() const { return image_path_; }

  bool set_geometry(std::uint32_t width, std::uint32_t height, float resolution,
                    double origin_x, double origin_y, std::string *error);
  void set_source_paths(std::string yaml_path, std::string image_path);
  std::int8_t at(std::uint32_t pixel_x, std::uint32_t pixel_y) const;
  GridWorldPoint pixel_to_world(int pixel_x, int pixel_y) const;
  bool world_to_pixel(double world_x, double world_y, int *pixel_x,
                      int *pixel_y) const;

private:
  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  float resolution_ = 0.05F;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  std::vector<std::int8_t> cells_;
  std::string yaml_path_;
  std::string image_path_;
};

}  // namespace agt_map_studio
