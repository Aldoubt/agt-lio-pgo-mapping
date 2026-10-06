#include "ui/StudyResultMarkers.hpp"

#include <QDir>
#include <QFileInfo>

#include <yaml-cpp/yaml.h>

#include <array>
#include <cmath>
#include <unordered_map>

namespace agt_map_studio {
namespace {

QVector3D classification_color(const std::string &classification) {
  if (classification == "CORRECT") return {0.10F, 0.82F, 0.20F};
  if (classification == "FALSE_ACCEPT") return {0.98F, 0.12F, 0.12F};
  if (classification == "REJECTED") return {0.98F, 0.75F, 0.08F};
  if (classification == "TIMEOUT") return {0.76F, 0.25F, 0.92F};
  if (classification == "NO_DATA") return {0.62F, 0.62F, 0.62F};
  return {};
}

bool supported_classification(const std::string &classification) {
  return classification == "CORRECT" || classification == "FALSE_ACCEPT" ||
         classification == "REJECTED" || classification == "TIMEOUT" ||
         classification == "NO_DATA";
}

bool matches_filter(const QString &filter, const std::string &value) {
  return filter == QStringLiteral("ALL") || filter == QString::fromStdString(value);
}

}  // namespace

bool load_study_result_markers(const QString &study_manifest_path,
                               const QString &classification_filter,
                               const QString &scene_filter,
                               const QString &frames_filter,
                               StudyResultMarkerCloud *result,
                               QString *error) {
  if (!result) {
    if (error) *error = QStringLiteral("result output is null");
    return false;
  }
  *result = {};
  try {
    const YAML::Node study = YAML::LoadFile(study_manifest_path.toStdString());
    if (!study["query_set_ref"] || !study["query_set_ref"]["path"] ||
        study["query_set_ref"]["kind"].as<std::string>("") != "filesystem") {
      if (error) *error = QStringLiteral("Study has no supported Query Set reference");
      return false;
    }
    const QString study_dir = QFileInfo(study_manifest_path).absolutePath();
    const QString query_set_path = QDir(study_dir).absoluteFilePath(
        QString::fromStdString(study["query_set_ref"]["path"].as<std::string>()));
    const YAML::Node query_set = YAML::LoadFile(query_set_path.toStdString());
    std::unordered_map<std::string, std::array<float, 3>> locations;
    if (query_set["queries"] && query_set["queries"].IsSequence()) {
      for (const auto &query : query_set["queries"]) {
        if (!query["query_id"] || !query["resolved_position_m"] ||
            !query["resolved_position_m"].IsSequence() ||
            query["resolved_position_m"].size() != 3) continue;
        std::array<float, 3> xyz{};
        bool finite = true;
        for (std::size_t axis = 0; axis < xyz.size(); ++axis) {
          xyz[axis] = query["resolved_position_m"][axis].as<float>();
          finite = finite && std::isfinite(xyz[axis]);
        }
        if (finite) locations[query["query_id"].as<std::string>()] = xyz;
      }
    }
    if (study["results"] && study["results"].IsSequence()) {
      for (const auto &row : study["results"]) {
        const std::string query_id = row["query_id"].as<std::string>("");
        const std::string classification = row["classification"].as<std::string>("NO_DATA");
        const std::string scene = row["scene"].as<std::string>("UNKNOWN");
        const int frames = row["query_accumulation_frames"].as<int>(0);
        if (!supported_classification(classification) ||
            !matches_filter(classification_filter, classification) ||
            !matches_filter(scene_filter, scene) ||
            (frames_filter != QStringLiteral("ALL") &&
             frames_filter != QString::number(frames))) continue;
        const auto location = locations.find(query_id);
        if (location == locations.end()) continue;
        result->cloud.xyz.insert(result->cloud.xyz.end(), location->second.begin(), location->second.end());
        result->colors.push_back(classification_color(classification));
      }
    }
    result->cloud.valid_point_count = result->cloud.point_count();
    return true;
  } catch (const std::exception &exception) {
    if (error) *error = QString::fromUtf8(exception.what());
    *result = {};
    return false;
  }
}

}  // namespace agt_map_studio
