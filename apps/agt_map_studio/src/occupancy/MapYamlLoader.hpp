#pragma once

#include "occupancy/GridMap.hpp"

#include <string>

namespace agt_map_studio {

struct MapYamlMetadata {
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
  bool negate = false;
  std::string mode = "trinary";
};

class MapYamlLoader {
public:
  static bool load(const std::string &yaml_path, GridMap *map,
                   MapYamlMetadata *metadata, std::string *error);
};

}  // namespace agt_map_studio
