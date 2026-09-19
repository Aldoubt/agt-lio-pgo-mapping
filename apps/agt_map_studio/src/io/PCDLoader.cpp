#include "io/PCDLoader.hpp"

#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace agt_map_studio {
namespace {

const pcl::PCLPointField *find_field(const pcl::PCLPointCloud2 &cloud,
                                     const char *name) {
  for (const auto &field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

std::uint32_t datatype_size(std::uint8_t datatype) {
  switch (datatype) {
    case pcl::PCLPointField::INT8:
    case pcl::PCLPointField::UINT8:
      return 1U;
    case pcl::PCLPointField::INT16:
    case pcl::PCLPointField::UINT16:
      return 2U;
    case pcl::PCLPointField::INT32:
    case pcl::PCLPointField::UINT32:
    case pcl::PCLPointField::FLOAT32:
      return 4U;
    case pcl::PCLPointField::FLOAT64:
      return 8U;
    default:
      return 0U;
  }
}

bool read_numeric(const std::uint8_t *address, std::uint8_t datatype,
                  std::uint32_t size, float *value) {
  if (!address || !value) {
    return false;
  }
  switch (datatype) {
    case pcl::PCLPointField::INT8: {
      std::int8_t v;
      std::memcpy(&v, address, std::min<std::uint32_t>(size, sizeof(v)));
      *value = static_cast<float>(v);
      return size >= sizeof(v);
    }
    case pcl::PCLPointField::UINT8: {
      std::uint8_t v;
      std::memcpy(&v, address, std::min<std::uint32_t>(size, sizeof(v)));
      *value = static_cast<float>(v);
      return size >= sizeof(v);
    }
    case pcl::PCLPointField::INT16: {
      std::int16_t v;
      if (size < sizeof(v)) return false;
      std::memcpy(&v, address, sizeof(v));
      *value = static_cast<float>(v);
      return true;
    }
    case pcl::PCLPointField::UINT16: {
      std::uint16_t v;
      if (size < sizeof(v)) return false;
      std::memcpy(&v, address, sizeof(v));
      *value = static_cast<float>(v);
      return true;
    }
    case pcl::PCLPointField::INT32: {
      std::int32_t v;
      if (size < sizeof(v)) return false;
      std::memcpy(&v, address, sizeof(v));
      *value = static_cast<float>(v);
      return true;
    }
    case pcl::PCLPointField::UINT32: {
      std::uint32_t v;
      if (size < sizeof(v)) return false;
      std::memcpy(&v, address, sizeof(v));
      *value = static_cast<float>(v);
      return true;
    }
    case pcl::PCLPointField::FLOAT32: {
      float v;
      if (size < sizeof(v)) return false;
      std::memcpy(&v, address, sizeof(v));
      *value = v;
      return true;
    }
    case pcl::PCLPointField::FLOAT64: {
      double v;
      if (size < sizeof(v)) return false;
      std::memcpy(&v, address, sizeof(v));
      *value = static_cast<float>(v);
      return true;
    }
    default:
      return false;
  }
}

}  // namespace

bool PCDLoader::load(const std::string &path, LoadedPointCloud *result,
                     std::string *error) {
  if (!result) {
    if (error) *error = "result must not be null";
    return false;
  }
  result->source.reset();
  result->xyz.clear();
  result->intensity.clear();
  result->source_indices.clear();
  result->valid_point_count = 0;

  auto cloud = std::make_shared<pcl::PCLPointCloud2>();
  if (pcl::io::loadPCDFile(path, *cloud) != 0) {
    if (error) *error = "PCL could not load PCD: " + path;
    return false;
  }

  const auto *x_field = find_field(*cloud, "x");
  const auto *y_field = find_field(*cloud, "y");
  const auto *z_field = find_field(*cloud, "z");
  const auto *intensity_field = find_field(*cloud, "intensity");
  if (!x_field || !y_field || !z_field) {
    if (error) *error = "PCD must contain x, y and z fields";
    return false;
  }

  const std::size_t width = cloud->width;
  const std::size_t height = cloud->height == 0 ? 1U : cloud->height;
  const std::size_t point_count = width * height;
  const std::size_t point_step = cloud->point_step;
  const std::size_t row_step = cloud->row_step == 0 ? width * point_step
                                                      : cloud->row_step;
  if (point_count == 0 || point_step == 0 || cloud->data.empty()) {
    if (error) *error = "PCD contains no point data";
    return false;
  }

  result->source = cloud;
  result->has_intensity = intensity_field != nullptr;
  result->xyz.reserve(point_count * 3U);
  result->source_indices.reserve(point_count);
  if (result->has_intensity) result->intensity.reserve(point_count);

  const float inf = std::numeric_limits<float>::infinity();
  result->min_bound = Eigen::Vector3f(inf, inf, inf);
  result->max_bound = Eigen::Vector3f(-inf, -inf, -inf);

  for (std::size_t row = 0; row < height; ++row) {
    for (std::size_t col = 0; col < width; ++col) {
      const std::size_t offset = row * row_step + col * point_step;
      if (offset + point_step > cloud->data.size()) {
        if (error) *error = "PCD row/point stride exceeds data buffer";
        result->source.reset();
        result->xyz.clear();
        result->intensity.clear();
        result->source_indices.clear();
        return false;
      }
      const auto *base = cloud->data.data() + offset;
      float x = 0.0F, y = 0.0F, z = 0.0F;
      if (!read_numeric(base + x_field->offset, x_field->datatype,
                        datatype_size(x_field->datatype), &x) ||
          !read_numeric(base + y_field->offset, y_field->datatype,
                        datatype_size(y_field->datatype), &y) ||
          !read_numeric(base + z_field->offset, z_field->datatype,
                        datatype_size(z_field->datatype), &z)) {
        continue;
      }
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }
      result->xyz.insert(result->xyz.end(), {x, y, z});
      result->source_indices.push_back(row * width + col);
      if (result->has_intensity) {
        float intensity = 0.0F;
        if (!read_numeric(base + intensity_field->offset,
                          intensity_field->datatype,
                          datatype_size(intensity_field->datatype),
                          &intensity)) {
          intensity = 0.0F;
        }
        result->intensity.push_back(intensity);
      }
      const Eigen::Vector3f point(x, y, z);
      result->min_bound = result->min_bound.cwiseMin(point);
      result->max_bound = result->max_bound.cwiseMax(point);
      ++result->valid_point_count;
    }
  }

  if (result->valid_point_count == 0) {
    if (error) *error = "PCD contains no finite XYZ points";
    result->source.reset();
    return false;
  }
  return true;
}

}  // namespace agt_map_studio
