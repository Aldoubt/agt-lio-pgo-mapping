#pragma once

#include "agt_pcd2grid_exporter/ProjectionParameters.hpp"

#include <string>

namespace agt_pcd2grid_exporter {

class ParameterLoader {
public:
  static bool load(const std::string &path, ProjectionParameters *parameters,
                   std::string *error);
  static bool write(const std::string &path,
                    const ProjectionParameters &parameters, std::string *error);
};

}  // namespace agt_pcd2grid_exporter
