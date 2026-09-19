#include "occupancy/MapYamlLoader.hpp"

#include <yaml-cpp/yaml.h>

#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <vector>

namespace agt_map_studio {
namespace {

bool read_token(std::istream &stream, std::string *token) {
  token->clear();
  char character = 0;
  while (stream.get(character)) {
    if (std::isspace(static_cast<unsigned char>(character))) continue;
    if (character == '#') {
      stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      continue;
    }
    token->push_back(character);
    break;
  }
  while (stream.get(character)) {
    if (std::isspace(static_cast<unsigned char>(character))) break;
    if (character == '#') {
      stream.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
      break;
    }
    token->push_back(character);
  }
  return !token->empty();
}

bool load_pgm(const std::string &path, std::uint32_t *width,
              std::uint32_t *height, std::uint32_t *max_value,
              std::vector<std::uint32_t> *pixels, std::string *error) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    if (error) *error = "cannot open PGM: " + path;
    return false;
  }
  std::string magic;
  std::string token;
  if (!read_token(stream, &magic) || (magic != "P2" && magic != "P5") ||
      !read_token(stream, &token)) {
    if (error) *error = "invalid PGM header: " + path;
    return false;
  }
  const auto parsed_width = std::stoul(token);
  if (!read_token(stream, &token)) {
    if (error) *error = "invalid PGM height";
    return false;
  }
  const auto parsed_height = std::stoul(token);
  if (!read_token(stream, &token)) {
    if (error) *error = "invalid PGM max value";
    return false;
  }
  const auto parsed_max = std::stoul(token);
  if (parsed_width == 0U || parsed_height == 0U || parsed_max == 0U ||
      parsed_max > 65535U ||
      static_cast<std::uint64_t>(parsed_width) * parsed_height >
          std::numeric_limits<std::size_t>::max() ||
      parsed_width > std::numeric_limits<std::uint32_t>::max() ||
      parsed_height > std::numeric_limits<std::uint32_t>::max()) {
    if (error) *error = "invalid PGM dimensions or max value";
    return false;
  }
  *width = static_cast<std::uint32_t>(parsed_width);
  *height = static_cast<std::uint32_t>(parsed_height);
  *max_value = static_cast<std::uint32_t>(parsed_max);
  pixels->resize(static_cast<std::size_t>(parsed_width) * parsed_height);
  if (magic == "P2") {
    for (auto &pixel : *pixels) {
      if (!read_token(stream, &token)) {
        if (error) *error = "truncated P2 image";
        return false;
      }
      pixel = std::stoul(token);
      if (pixel > *max_value) {
        if (error) *error = "P2 pixel exceeds max value";
        return false;
      }
    }
    return true;
  }
  const std::size_t bytes_per_pixel = *max_value > 255U ? 2U : 1U;
  std::vector<unsigned char> bytes(pixels->size() * bytes_per_pixel);
  stream.read(reinterpret_cast<char *>(bytes.data()),
              static_cast<std::streamsize>(bytes.size()));
  if (stream.gcount() != static_cast<std::streamsize>(bytes.size())) {
    if (error) *error = "truncated P5 image";
    return false;
  }
  for (std::size_t i = 0; i < pixels->size(); ++i) {
    if (bytes_per_pixel == 1U) {
      (*pixels)[i] = bytes[i];
    } else {
      (*pixels)[i] = (static_cast<std::uint32_t>(bytes[i * 2U]) << 8U) |
                     bytes[i * 2U + 1U];
    }
  }
  return true;
}

}  // namespace

bool MapYamlLoader::load(const std::string &yaml_path, GridMap *map,
                         MapYamlMetadata *metadata, std::string *error) {
  if (!map || !metadata) {
    if (error) *error = "map and metadata must not be null";
    return false;
  }
  try {
    const YAML::Node root = YAML::LoadFile(yaml_path);
    const std::string image = root["image"].as<std::string>();
    const float resolution = root["resolution"].as<float>();
    const YAML::Node origin = root["origin"];
    if (!origin || origin.size() < 2U) {
      if (error) *error = "map.yaml origin must contain x and y";
      return false;
    }
    const double origin_x = origin[0].as<double>();
    const double origin_y = origin[1].as<double>();
    metadata->occupied_thresh = root["occupied_thresh"].as<double>(0.65);
    metadata->free_thresh = root["free_thresh"].as<double>(0.196);
    metadata->negate = root["negate"].as<int>(0) != 0;
    metadata->mode = root["mode"].as<std::string>("trinary");
    if (!(resolution > 0.0F) || metadata->occupied_thresh < 0.0 ||
        metadata->occupied_thresh > 1.0 || metadata->free_thresh < 0.0 ||
        metadata->free_thresh > 1.0 ||
        metadata->free_thresh >= metadata->occupied_thresh) {
      if (error) *error = "map.yaml contains invalid resolution or thresholds";
      return false;
    }
    const std::filesystem::path yaml_file(yaml_path);
    const std::filesystem::path image_path = yaml_file.parent_path() / image;
    std::uint32_t width = 0U;
    std::uint32_t height = 0U;
    std::uint32_t max_value = 0U;
    std::vector<std::uint32_t> pixels;
    if (!load_pgm(image_path.string(), &width, &height, &max_value, &pixels,
                  error)) {
      return false;
    }
    if (!map->set_geometry(width, height, resolution, origin_x, origin_y, error)) {
      return false;
    }
    auto &cells = map->cells();
    for (std::uint32_t image_y = 0; image_y < height; ++image_y) {
      const std::uint32_t grid_y = height - 1U - image_y;
      for (std::uint32_t x = 0; x < width; ++x) {
        const std::uint32_t pixel = pixels[static_cast<std::size_t>(image_y) * width + x];
        const double normalized = static_cast<double>(metadata->negate
                                                          ? pixel
                                                          : max_value - pixel) /
                                  max_value;
        std::int8_t value = GridMap::kUnknown;
        if (normalized > metadata->occupied_thresh) value = GridMap::kOccupied;
        else if (normalized < metadata->free_thresh) value = GridMap::kFree;
        cells[static_cast<std::size_t>(grid_y) * width + x] = value;
      }
    }
    map->set_source_paths(yaml_path, image_path.string());
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
  return true;
}

}  // namespace agt_map_studio
