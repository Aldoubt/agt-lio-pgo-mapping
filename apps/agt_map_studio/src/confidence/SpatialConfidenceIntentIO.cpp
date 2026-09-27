#include "confidence/SpatialConfidenceIntentIO.hpp"

#include <QSaveFile>
#include <QString>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace agt_map_studio {
namespace {
namespace fs = std::filesystem;

bool inside(const fs::path &candidate, const fs::path &base) {
  auto current = candidate.begin();
  for (auto part = base.begin(); part != base.end(); ++part, ++current) {
    if (current == candidate.end() || *current != *part) return false;
  }
  return true;
}

fs::path validated_target(const fs::path &path, const SpatialConfidenceModel &model,
                          bool allow_replace) {
  if (path.empty() || path.filename().empty() || path.filename() == "." ||
      path.filename() == ".." || model.empty()) {
    throw std::invalid_argument("choose a separate override-intent YAML file for a loaded derivative");
  }
  const auto &info = model.info();
  if (info.source_package.empty() || info.derivative_dir.empty() ||
      !std::isfinite(info.voxel_size) || info.voxel_size <= 0.0F) {
    throw std::invalid_argument("confidence source provenance/voxel_size is missing");
  }
  const auto parent = path.has_parent_path() ? path.parent_path() : fs::path(".");
  if (!fs::is_directory(parent)) {
    throw std::invalid_argument("intent YAML parent directory must already exist");
  }
  const auto resolved = fs::canonical(parent) / path.filename();
  if (inside(resolved, fs::canonical(info.source_package)) ||
      inside(resolved, fs::canonical(info.derivative_dir))) {
    throw std::invalid_argument("intent YAML must be outside the PGO package and source derivative");
  }
  if (fs::is_symlink(fs::symlink_status(resolved)) ||
      (fs::exists(resolved) && (!fs::is_regular_file(resolved) || !allow_replace))) {
    throw std::invalid_argument("intent target exists or is unsafe; choose a new file or explicitly confirm replacement");
  }
  return resolved;
}

}  // namespace

bool SpatialConfidenceIntentIO::save(const fs::path &path,
                                      const SpatialConfidenceModel &model,
                                      const SpatialConfidenceEditor &editor,
                                      bool allow_replace, std::string *error) {
  if (error) error->clear();
  try {
    if (editor.model() != &model) {
      throw std::invalid_argument("confidence editor is not bound to the selected derivative");
    }
    const auto target = validated_target(path, model, allow_replace);
    YAML::Emitter yaml;
    yaml << YAML::BeginMap
         << YAML::Key << "schema_version" << YAML::Value << 1
         << YAML::Key << "coordinate_system" << YAML::Value << "voxel_index"
         << YAML::Key << "voxel_size" << YAML::Value << model.info().voxel_size
         << YAML::Key << "overrides" << YAML::Value << YAML::BeginSeq;
    for (const auto &[key, intent] : editor.sorted_intents()) {
      if (!model.find(key)) throw std::invalid_argument("intent targets an unobserved voxel");
      agt_spatial_map_core::validate_manual_override_audit(intent.audit);
      if (intent.has_manual_value &&
          intent.mode != agt_spatial_map_core::ManualOverrideMode::FORCE_LOW) {
        throw std::invalid_argument("only FORCE_LOW can have an explicit value");
      }
      // Use core's v1 modes/final-value validation without editing evidence.
      agt_spatial_map_core::manual_final_confidence(
          0.5F, intent.mode, intent.has_manual_value, intent.manual_value,
          model.info().force_low_value);
      yaml << YAML::BeginMap << YAML::Key << "key" << YAML::Value << YAML::Flow
           << YAML::BeginSeq << key.x << key.y << key.z << YAML::EndSeq
           << YAML::Key << "mode" << YAML::Value
           << agt_spatial_map_core::override_mode_name(intent.mode);
      if (intent.has_manual_value) yaml << YAML::Key << "value" << YAML::Value << intent.manual_value;
      if (!intent.audit.reason.empty()) {
        yaml << YAML::Key << "reason" << YAML::Value << intent.audit.reason;
      }
      if (!intent.audit.edited_at.empty()) {
        yaml << YAML::Key << "edited_at" << YAML::Value << YAML::DoubleQuoted
               << intent.audit.edited_at;
      }
      if (!intent.audit.editor.empty()) {
        yaml << YAML::Key << "editor" << YAML::Value << intent.audit.editor;
      }
      yaml << YAML::EndMap;
    }
    yaml << YAML::EndSeq << YAML::EndMap;
    if (!yaml.good()) throw std::runtime_error("cannot serialize manual override intent YAML");
    QSaveFile file(QString::fromStdString(target.string()));
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      throw std::runtime_error("cannot create separate intent YAML: " + file.errorString().toStdString());
    }
    const std::string text = std::string(yaml.c_str()) + '\n';
    if (file.write(text.data(), static_cast<qint64>(text.size())) !=
            static_cast<qint64>(text.size()) || !file.commit()) {
      throw std::runtime_error("atomic intent save failed: " + file.errorString().toStdString());
    }
    return true;
  } catch (const std::exception &exception) {
    if (error) *error = exception.what();
    return false;
  }
}

}  // namespace agt_map_studio
