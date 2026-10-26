#pragma once

#include "io/PCDLoader.hpp"

#include <QString>
#include <QVector3D>

#include <vector>

namespace agt_map_studio {

struct StudyResultMarkerCloud {
  LoadedPointCloud cloud;
  std::vector<QVector3D> colors;
};

// Load empirical result markers at each query's resolved map-frame position.
// Filters use the stored contract values (or "ALL") and never infer row IDs.
bool load_study_result_markers(const QString &study_manifest_path,
                               const QString &classification_filter,
                               const QString &scene_filter,
                               const QString &frames_filter,
                               StudyResultMarkerCloud *result,
                               QString *error = nullptr);

}  // namespace agt_map_studio
