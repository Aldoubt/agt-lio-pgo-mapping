#pragma once

#include <QString>
#include <QStringList>

#include <map>
#include <string>

namespace agt_map_studio {

// Freshness of one derived artifact relative to the inputs it was built from.
enum class StageState { Missing, Fresh, Stale };

struct StageRecord {
  QString path;               // directory or file produced by the stage
  QString input_fingerprint;  // fingerprint of the inputs when the stage ran
  QString output_sha256;      // sha256 of the main output file when applicable
  QString updated_at;
  QString note;
};

// Navigation converter parameters mirrored from agt_map_manager pipeline
// defaults. Only these keys are forwarded to `pcd_to_nav_map`.
struct ConverterParameters {
  double resolution = 0.10;
  double margin = 1.0;
  int min_points = 2;
  double max_step = 0.22;
  double max_slope_deg = 20.0;
  double trajectory_front_m = 0.40;
  double trajectory_rear_m = 0.72;
  double trajectory_half_width_m = 0.46;
  bool use_trajectory = true;

  QStringList to_arguments(const QString &poses_path) const;
  QString fingerprint() const;
};

struct PublishTarget {
  QString map_root;
  QString map_id;
  QString map_version;
  bool activate = false;
};

// Durable studio session: which source is loaded, which derived artifacts
// exist and whether they are still consistent with their upstream inputs.
//
//   source.pcd -> refined package -> { relocalization/, navigation/ }
//              -> patched navigation/ -> published package
class WorkflowSession {
public:
  enum Stage { Refine, Relocalization, Navigation, Patch, Publish };

  static QString sha256_file(const QString &path);
  static QString now_iso8601();

  void reset(const QString &source_pcd, const QString &source_package_dir);
  bool empty() const { return source_pcd_.isEmpty(); }

  const QString &source_pcd() const { return source_pcd_; }
  const QString &source_package_dir() const { return source_package_dir_; }
  const QString &source_pcd_sha256() const { return source_pcd_sha256_; }
  bool source_is_mapping_package() const { return !source_package_dir_.isEmpty(); }
  QString source_poses_path() const;

  // Working directory that stores refinement.yaml, patches, logs, session file.
  void set_work_dir(const QString &dir) { work_dir_ = dir; }
  const QString &work_dir() const { return work_dir_; }
  QString session_file() const;
  QString log_path() const;
  QString refinement_rules_path() const;
  QString navigation_patch_path() const;
  QString keepout_zones_path() const;
  QString pipeline_config_path() const;
  QString hmi_edit_metadata_path() const;

  // Fingerprints of the current in-memory edits; owned by the caller.
  void set_refinement_fingerprint(const QString &value) { refinement_fingerprint_ = value; }
  void set_patch_fingerprint(const QString &value) { patch_fingerprint_ = value; }
  const QString &refinement_fingerprint() const { return refinement_fingerprint_; }
  const QString &patch_fingerprint() const { return patch_fingerprint_; }
  bool has_3d_edits() const { return !refinement_fingerprint_.isEmpty(); }
  bool has_2d_edits() const { return !patch_fingerprint_.isEmpty(); }

  ConverterParameters &converter() { return converter_; }
  const ConverterParameters &converter() const { return converter_; }
  PublishTarget &publish_target() { return publish_target_; }
  const PublishTarget &publish_target() const { return publish_target_; }

  // Expected inputs of each stage right now.
  QString expected_input_fingerprint(Stage stage) const;
  StageState state(Stage stage) const;
  const StageRecord &record(Stage stage) const;
  void mark_done(Stage stage, const QString &path, const QString &output_sha256 = QString(),
                 const QString &note = QString());
  void clear(Stage stage);

  // The PCD every downstream stage must consume: refined when fresh, else source.
  QString effective_pcd() const;
  QString effective_pcd_sha256() const;
  QString effective_poses_path() const;
  // The navigation directory the publish step must package.
  QString effective_navigation_dir() const;

  static QString stage_name(Stage stage);
  QStringList blocking_reasons_for_publish() const;

  bool save(QString *error) const;
  bool load(const QString &session_file, QString *error);

private:
  QString source_pcd_;
  QString source_package_dir_;
  QString source_pcd_sha256_;
  QString work_dir_;
  QString refinement_fingerprint_;
  QString patch_fingerprint_;
  ConverterParameters converter_;
  PublishTarget publish_target_;
  std::map<int, StageRecord> records_;
};

}  // namespace agt_map_studio
