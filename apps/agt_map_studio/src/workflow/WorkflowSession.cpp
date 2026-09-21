#include "workflow/WorkflowSession.hpp"

#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>

#include <yaml-cpp/yaml.h>

#include <exception>
#include <fstream>

namespace agt_map_studio {

QStringList ConverterParameters::to_arguments(const QString &poses_path) const {
  QStringList arguments;
  arguments << QStringLiteral("--resolution") << QString::number(resolution, 'f', 4)
            << QStringLiteral("--margin") << QString::number(margin, 'f', 3)
            << QStringLiteral("--min-points") << QString::number(min_points)
            << QStringLiteral("--max-step") << QString::number(max_step, 'f', 3)
            << QStringLiteral("--max-slope-deg") << QString::number(max_slope_deg, 'f', 2);
  if (use_trajectory && !poses_path.isEmpty()) {
    arguments << QStringLiteral("--trajectory-poses") << poses_path
              << QStringLiteral("--trajectory-front-m") << QString::number(trajectory_front_m, 'f', 3)
              << QStringLiteral("--trajectory-rear-m") << QString::number(trajectory_rear_m, 'f', 3)
              << QStringLiteral("--trajectory-half-width-m")
              << QString::number(trajectory_half_width_m, 'f', 3);
  }
  return arguments;
}

QString ConverterParameters::fingerprint() const {
  return to_arguments(use_trajectory ? QStringLiteral("poses") : QString()).join('|');
}

QString WorkflowSession::sha256_file(const QString &path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) return QString();
  QCryptographicHash hash(QCryptographicHash::Sha256);
  if (!hash.addData(&file)) return QString();
  return QString::fromLatin1(hash.result().toHex());
}

QString WorkflowSession::now_iso8601() {
  return QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
}

void WorkflowSession::reset(const QString &source_pcd, const QString &source_package_dir) {
  source_pcd_ = source_pcd;
  source_package_dir_ = source_package_dir;
  source_pcd_sha256_ = sha256_file(source_pcd);
  refinement_fingerprint_.clear();
  patch_fingerprint_.clear();
  records_.clear();
  if (work_dir_.isEmpty() && !source_pcd.isEmpty()) {
    work_dir_ = QDir(QFileInfo(source_pcd).absolutePath()).filePath(QStringLiteral("studio_work"));
  }
}

QString WorkflowSession::source_poses_path() const {
  if (source_package_dir_.isEmpty()) return QString();
  const QString poses = QDir(source_package_dir_).filePath(QStringLiteral("poses.txt"));
  return QFileInfo::exists(poses) ? poses : QString();
}

QString WorkflowSession::session_file() const {
  return QDir(work_dir_).filePath(QStringLiteral("studio_session.yaml"));
}
QString WorkflowSession::log_path() const {
  return QDir(work_dir_).filePath(QStringLiteral("studio_tools.log"));
}
QString WorkflowSession::refinement_rules_path() const {
  return QDir(work_dir_).filePath(QStringLiteral("refinement.yaml"));
}
QString WorkflowSession::navigation_patch_path() const {
  return QDir(work_dir_).filePath(QStringLiteral("navigation_patch.yaml"));
}
QString WorkflowSession::keepout_zones_path() const {
  return QDir(work_dir_).filePath(QStringLiteral("keepout_zones.yaml"));
}
QString WorkflowSession::pipeline_config_path() const {
  return QDir(work_dir_).filePath(QStringLiteral("pipeline.yaml"));
}
QString WorkflowSession::hmi_edit_metadata_path() const {
  return QDir(work_dir_).filePath(QStringLiteral("hmi_edit_metadata.yaml"));
}

QString WorkflowSession::expected_input_fingerprint(Stage stage) const {
  switch (stage) {
    case Refine:
      return source_pcd_sha256_ + '|' + refinement_fingerprint_;
    case Relocalization:
      return effective_pcd_sha256();
    case Navigation:
      return effective_pcd_sha256() + '|' + converter_.fingerprint();
    case Patch: {
      const auto &navigation = record(Navigation);
      return navigation.output_sha256 + '|' + patch_fingerprint_;
    }
    case Publish: {
      return effective_pcd_sha256() + '|' + record(Relocalization).output_sha256 + '|' +
             record(Patch).input_fingerprint + '|' + record(Navigation).output_sha256;
    }
  }
  return QString();
}

StageState WorkflowSession::state(Stage stage) const {
  const auto found = records_.find(static_cast<int>(stage));
  if (found == records_.end() || found->second.path.isEmpty()) return StageState::Missing;
  if (!QFileInfo::exists(found->second.path)) return StageState::Missing;
  return found->second.input_fingerprint == expected_input_fingerprint(stage)
             ? StageState::Fresh
             : StageState::Stale;
}

const StageRecord &WorkflowSession::record(Stage stage) const {
  static const StageRecord empty;
  const auto found = records_.find(static_cast<int>(stage));
  return found == records_.end() ? empty : found->second;
}

void WorkflowSession::mark_done(Stage stage, const QString &path, const QString &output_sha256,
                                const QString &note) {
  StageRecord record;
  record.path = path;
  record.input_fingerprint = expected_input_fingerprint(stage);
  record.output_sha256 = output_sha256;
  record.updated_at = now_iso8601();
  record.note = note;
  records_[static_cast<int>(stage)] = record;
}

void WorkflowSession::clear(Stage stage) { records_.erase(static_cast<int>(stage)); }

QString WorkflowSession::effective_pcd() const {
  if (has_3d_edits() && state(Refine) == StageState::Fresh) {
    return QDir(record(Refine).path).filePath(QStringLiteral("map.pcd"));
  }
  return source_pcd_;
}

QString WorkflowSession::effective_pcd_sha256() const {
  if (has_3d_edits() && state(Refine) == StageState::Fresh) return record(Refine).output_sha256;
  return source_pcd_sha256_;
}

QString WorkflowSession::effective_poses_path() const {
  if (has_3d_edits() && state(Refine) == StageState::Fresh) {
    const QString poses = QDir(record(Refine).path).filePath(QStringLiteral("poses.txt"));
    if (QFileInfo::exists(poses)) return poses;
  }
  return source_poses_path();
}

QString WorkflowSession::effective_navigation_dir() const {
  if (has_2d_edits()) {
    return state(Patch) == StageState::Fresh ? record(Patch).path : QString();
  }
  return state(Navigation) == StageState::Fresh ? record(Navigation).path : QString();
}

QString WorkflowSession::stage_name(Stage stage) {
  switch (stage) {
    case Refine: return QStringLiteral("3D refinement");
    case Relocalization: return QStringLiteral("relocalization assets");
    case Navigation: return QStringLiteral("navigation layers");
    case Patch: return QStringLiteral("2D patch");
    case Publish: return QStringLiteral("map package");
  }
  return QString();
}

QStringList WorkflowSession::blocking_reasons_for_publish() const {
  QStringList reasons;
  if (empty()) {
    reasons << QStringLiteral("no source PCD loaded");
    return reasons;
  }
  if (has_3d_edits() && state(Refine) != StageState::Fresh) {
    reasons << QStringLiteral("3D edits changed: run Refine again");
  }
  if (state(Relocalization) != StageState::Fresh) {
    reasons << QStringLiteral("relocalization assets missing or built from another PCD");
  }
  if (state(Navigation) != StageState::Fresh) {
    reasons << QStringLiteral("navigation layers missing or built from another PCD/parameters");
  }
  if (has_2d_edits() && state(Patch) != StageState::Fresh) {
    reasons << QStringLiteral("2D edits changed: apply the navigation patch again");
  }
  if (publish_target_.map_id.trimmed().isEmpty() || publish_target_.map_version.trimmed().isEmpty()) {
    reasons << QStringLiteral("map_id and map_version are required");
  }
  return reasons;
}

namespace {

const char *state_text(StageState state) {
  switch (state) {
    case StageState::Fresh: return "fresh";
    case StageState::Stale: return "stale";
    default: return "missing";
  }
}

}  // namespace

bool WorkflowSession::save(QString *error) const {
  if (work_dir_.isEmpty()) {
    if (error) *error = QStringLiteral("session work directory is not set");
    return false;
  }
  try {
    QDir().mkpath(work_dir_);
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "schema_version" << YAML::Value << 1;
    out << YAML::Key << "generator" << YAML::Value << "agt_map_studio";
    out << YAML::Key << "saved_at" << YAML::Value << now_iso8601().toStdString();
    out << YAML::Key << "source" << YAML::Value << YAML::BeginMap
        << YAML::Key << "pcd" << YAML::Value << source_pcd_.toStdString()
        << YAML::Key << "package_dir" << YAML::Value << source_package_dir_.toStdString()
        << YAML::Key << "pcd_sha256" << YAML::Value << source_pcd_sha256_.toStdString()
        << YAML::EndMap;
    out << YAML::Key << "edits" << YAML::Value << YAML::BeginMap
        << YAML::Key << "refinement_fingerprint" << YAML::Value << refinement_fingerprint_.toStdString()
        << YAML::Key << "patch_fingerprint" << YAML::Value << patch_fingerprint_.toStdString()
        << YAML::EndMap;
    out << YAML::Key << "converter" << YAML::Value << YAML::BeginMap
        << YAML::Key << "resolution" << YAML::Value << converter_.resolution
        << YAML::Key << "margin" << YAML::Value << converter_.margin
        << YAML::Key << "min_points" << YAML::Value << converter_.min_points
        << YAML::Key << "max_step" << YAML::Value << converter_.max_step
        << YAML::Key << "max_slope_deg" << YAML::Value << converter_.max_slope_deg
        << YAML::Key << "trajectory_front_m" << YAML::Value << converter_.trajectory_front_m
        << YAML::Key << "trajectory_rear_m" << YAML::Value << converter_.trajectory_rear_m
        << YAML::Key << "trajectory_half_width_m" << YAML::Value << converter_.trajectory_half_width_m
        << YAML::Key << "use_trajectory" << YAML::Value << converter_.use_trajectory
        << YAML::EndMap;
    out << YAML::Key << "publish" << YAML::Value << YAML::BeginMap
        << YAML::Key << "map_root" << YAML::Value << publish_target_.map_root.toStdString()
        << YAML::Key << "map_id" << YAML::Value << publish_target_.map_id.toStdString()
        << YAML::Key << "map_version" << YAML::Value << publish_target_.map_version.toStdString()
        << YAML::Key << "activate" << YAML::Value << publish_target_.activate
        << YAML::EndMap;
    out << YAML::Key << "stages" << YAML::Value << YAML::BeginMap;
    for (const auto stage : {Refine, Relocalization, Navigation, Patch, Publish}) {
      const auto &record = this->record(stage);
      out << YAML::Key << stage_name(stage).toStdString() << YAML::Value << YAML::BeginMap
          << YAML::Key << "stage" << YAML::Value << static_cast<int>(stage)
          << YAML::Key << "path" << YAML::Value << record.path.toStdString()
          << YAML::Key << "input_fingerprint" << YAML::Value << record.input_fingerprint.toStdString()
          << YAML::Key << "output_sha256" << YAML::Value << record.output_sha256.toStdString()
          << YAML::Key << "updated_at" << YAML::Value << record.updated_at.toStdString()
          << YAML::Key << "note" << YAML::Value << record.note.toStdString()
          << YAML::Key << "state" << YAML::Value << state_text(state(stage))
          << YAML::EndMap;
    }
    out << YAML::EndMap << YAML::EndMap;
    std::ofstream stream(session_file().toStdString());
    stream << out.c_str() << '\n';
    if (!stream.good()) {
      if (error) *error = QStringLiteral("cannot write %1").arg(session_file());
      return false;
    }
  } catch (const std::exception &exception) {
    if (error) *error = QString::fromUtf8(exception.what());
    return false;
  }
  return true;
}

bool WorkflowSession::load(const QString &session_file, QString *error) {
  try {
    const YAML::Node root = YAML::LoadFile(session_file.toStdString());
    if (root["schema_version"].as<int>(0) != 1) {
      if (error) *error = QStringLiteral("unsupported studio session schema");
      return false;
    }
    work_dir_ = QFileInfo(session_file).absolutePath();
    const YAML::Node source = root["source"];
    source_pcd_ = QString::fromStdString(source["pcd"].as<std::string>(""));
    source_package_dir_ = QString::fromStdString(source["package_dir"].as<std::string>(""));
    source_pcd_sha256_ = QString::fromStdString(source["pcd_sha256"].as<std::string>(""));
    const YAML::Node edits = root["edits"];
    refinement_fingerprint_ = QString::fromStdString(edits["refinement_fingerprint"].as<std::string>(""));
    patch_fingerprint_ = QString::fromStdString(edits["patch_fingerprint"].as<std::string>(""));
    const YAML::Node converter = root["converter"];
    if (converter) {
      converter_.resolution = converter["resolution"].as<double>(converter_.resolution);
      converter_.margin = converter["margin"].as<double>(converter_.margin);
      converter_.min_points = converter["min_points"].as<int>(converter_.min_points);
      converter_.max_step = converter["max_step"].as<double>(converter_.max_step);
      converter_.max_slope_deg = converter["max_slope_deg"].as<double>(converter_.max_slope_deg);
      converter_.trajectory_front_m = converter["trajectory_front_m"].as<double>(converter_.trajectory_front_m);
      converter_.trajectory_rear_m = converter["trajectory_rear_m"].as<double>(converter_.trajectory_rear_m);
      converter_.trajectory_half_width_m =
          converter["trajectory_half_width_m"].as<double>(converter_.trajectory_half_width_m);
      converter_.use_trajectory = converter["use_trajectory"].as<bool>(converter_.use_trajectory);
    }
    const YAML::Node publish = root["publish"];
    if (publish) {
      publish_target_.map_root = QString::fromStdString(publish["map_root"].as<std::string>(""));
      publish_target_.map_id = QString::fromStdString(publish["map_id"].as<std::string>(""));
      publish_target_.map_version = QString::fromStdString(publish["map_version"].as<std::string>(""));
      publish_target_.activate = publish["activate"].as<bool>(false);
    }
    records_.clear();
    for (const auto &entry : root["stages"]) {
      const YAML::Node value = entry.second;
      StageRecord record;
      record.path = QString::fromStdString(value["path"].as<std::string>(""));
      record.input_fingerprint = QString::fromStdString(value["input_fingerprint"].as<std::string>(""));
      record.output_sha256 = QString::fromStdString(value["output_sha256"].as<std::string>(""));
      record.updated_at = QString::fromStdString(value["updated_at"].as<std::string>(""));
      record.note = QString::fromStdString(value["note"].as<std::string>(""));
      records_[value["stage"].as<int>()] = record;
    }
  } catch (const std::exception &exception) {
    if (error) *error = QString::fromUtf8(exception.what());
    return false;
  }
  return true;
}

}  // namespace agt_map_studio
