#pragma once

#include "geometry/GeometryEvidenceModel.hpp"
#include "occupancy/GridMap.hpp"
#include "occupancy/OccupancyViewer.hpp"
#include "occupancy/RefinementModel.hpp"
#include "selection/SelectionManager.h"
#include "tools/ExternalToolRunner.hpp"
#include "viewer/PointCloudViewer.hpp"
#include "workflow/WorkflowSession.hpp"

#include <QMainWindow>
#include <QStackedWidget>
#include <QString>
#include <QVector>

#include <functional>
#include <vector>

class QAction;
class QCheckBox;
class QComboBox;
class QDockWidget;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QMenu;
class QPushButton;
class QSpinBox;
class QTableWidget;
class QToolBar;

namespace agt_map_studio {

class WorkflowPanel;

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(const QString &config_path, QWidget *parent = nullptr);

  bool open_pcd(const QString &path, QString *error = nullptr);
  // A mapping package directory (map.pcd + manifest.yaml ...) or a v3 map
  // package directory (localization/global_map.pcd + navigation/map.yaml).
  bool open_mapping_package(const QString &directory, QString *error = nullptr);
  bool open_spatial_confidence(const QString &directory, QString *error = nullptr);
  bool open_geometry_evidence(const QString &directory, QString *error = nullptr);
  // Save v1 intent YAML only; never modifies the opened derivative. The core
  // reviewed rebuild is a distinct asynchronous operation below.
  bool save_confidence_overrides(const QString &path, bool allow_replace = false,
                                 QString *error = nullptr);
  bool start_confidence_review(const QString &new_directory, QString *error = nullptr);
  bool open_occupancy_map(const QString &path, QString *error = nullptr);
  bool open_session(const QString &session_file, QString *error = nullptr);
  // Lightweight post-mapping review: keep the PCD as provenance, but load
  // only the generated 2D map into the UI.
  bool open_mapping_review(const QString &package_dir, const QString &map_yaml,
                           const QString &review_output, QString *error = nullptr);

protected:
  void closeEvent(QCloseEvent *event) override;

private slots:
  void open_pcd_dialog();
  void open_mapping_package_dialog();
  void open_spatial_confidence_dialog();
  void open_geometry_evidence_dialog();
  void inspect_confidence_voxel(std::size_t index);
  void apply_confidence_override();
  void restore_confidence_auto();
  void undo_confidence_override();
  void redo_confidence_override();
  void save_confidence_overrides_dialog();
  void rebuild_confidence_review_dialog();
  void open_occupancy_map_dialog();
  void open_session_dialog();
  void save_view_dialog();
  void export_clean_map_dialog();
  void generate_occupancy_preview_dialog();
  void save_refinement_dialog();
  void export_navigation_map_dialog();
  void confirm_mapping_review();
  void export_refinement_rules_dialog();
  void export_navigation_patch_dialog();
  void reset_camera();
  void undo_edit();
  void redo_edit();
  void set_mode_navigate();
  void set_mode_select();
  void set_mode_delete();
  void delete_selected();
  void select_height_band_dialog();
  void set_isometric_view();
  void set_front_view();
  void set_top_view();
  void show_3d_view();
  void show_2d_view();
  void set_occupancy_mode(OccupancyInteractionMode mode);
  void toggle_axis(bool checked);
  void toggle_background(bool checked);
  void show_controls();
  void show_workflow_help();
  void show_stats(const QString &text);
  void start_structure_editor();
  void build_keyframe_blocks();
  void load_keyframe_blocks();
  void run_relocalization_query();
  void load_relocalization_evidence();
  void choose_relocalization_evidence();
  void choose_topology_annotation();
  void load_structure_annotation();
  void pick_relocalization_point();
  void select_candidate_overlay(int row, int column);

  // Workflow steps (each is asynchronous; completion continues the queue).
  void run_refine();
  void run_relocalization();
  void run_navigation();
  void run_patch();
  void run_publish();
  void run_all_pending();
  void cancel_tool();

private:
  using StepFn = std::function<void()>;

  void create_actions();
  void create_workflow_dock();
  void create_confidence_dock();
  void create_relocalization_dock();
  void display_block_preview(const QString &directory);
  void start_relocalization_tool(const ToolInvocation &invocation,
                                 std::function<void(const ToolResult &)> on_done);
  void clear_confidence_view();
  void clear_geometry_view();
  void set_point_color_mode(PointColorMode mode);
  void refresh_confidence_editor_ui();
  void update_edit_state_label();
  bool confirm_discard_confidence_edits();
  bool validate_relocalization_output_root(QString *error) const;
  void load_config(const QString &path);
  void apply_erase_rectangle(double min_x, double min_y, double max_x, double max_y);
  void apply_obstacle_line(double start_x, double start_y, double end_x, double end_y,
                           double width_m);
  void apply_forbidden_polygon(const QVector<QPointF> &polygon);
  void apply_fill_polygon(const QVector<QPointF> &polygon, int value);
  void refresh_occupancy_view();
  void sync_edit_fingerprints();
  void refresh_workflow();
  void set_source(const QString &pcd_path, const QString &package_dir);
  QString stamp() const;
  bool ensure_work_dir(QString *error);
  bool tools_available(const QStringList &required, QString *missing) const;
  void run_tool(const ToolInvocation &invocation, std::function<void(const ToolResult &)> on_done);
  void continue_queue();
  void fail_queue(const QString &message);
  bool replay_2d_history(const std::vector<RefinementOperation> &history);
  bool load_navigation_dir_into_2d(const QString &directory, QString *error);
  bool write_pipeline_config(QString *error) const;
  void save_session_quietly();

  PointCloudViewer *viewer_ = nullptr;
  OccupancyViewer *occupancy_viewer_ = nullptr;
  QStackedWidget *view_stack_ = nullptr;
  QToolBar *toolbar_3d_ = nullptr;
  QToolBar *occupancy_toolbar_ = nullptr;
  QDockWidget *workflow_dock_ = nullptr;
  QDockWidget *confidence_dock_ = nullptr;
  QDockWidget *relocalization_dock_ = nullptr;
  QLabel *confidence_details_label_ = nullptr;
  QLabel *geometry_summary_label_ = nullptr;
  QLabel *confidence_status_label_ = nullptr;
  QLabel *confidence_edit_state_label_ = nullptr;
  QLabel *relocalization_status_label_ = nullptr;
  QLabel *relocalization_details_label_ = nullptr;
  QLineEdit *block_directory_edit_ = nullptr;
  QLineEdit *native_localizer_path_edit_ = nullptr;
  QLineEdit *topology_path_edit_ = nullptr;
  QLineEdit *evidence_root_edit_ = nullptr;
  QLineEdit *evidence_path_edit_ = nullptr;
  QLineEdit *query_x_edit_ = nullptr;
  QLineEdit *query_y_edit_ = nullptr;
  QLineEdit *row_id_edit_ = nullptr;
  QDoubleSpinBox *along_row_s_spin_ = nullptr;
  QComboBox *query_mode_combo_ = nullptr;
  QComboBox *query_frames_combo_ = nullptr;
  QSpinBox *block_keyframe_count_spin_ = nullptr;
  QSpinBox *query_keyframe_spin_ = nullptr;
  QSpinBox *candidate_top_k_spin_ = nullptr;
  QTableWidget *candidate_table_ = nullptr;
  QCheckBox *show_structure_layer_ = nullptr;
  QCheckBox *show_blocks_layer_ = nullptr;
  QCheckBox *show_query_layer_ = nullptr;
  QCheckBox *show_candidate_layer_ = nullptr;
  QString loaded_evidence_directory_;
  double picked_query_x_ = 0.0;
  double picked_query_y_ = 0.0;
  bool has_picked_query_point_ = false;
  QComboBox *confidence_override_mode_combo_ = nullptr;
  QComboBox *confidence_reason_combo_ = nullptr;
  QCheckBox *confidence_custom_low_check_ = nullptr;
  QDoubleSpinBox *confidence_low_value_spin_ = nullptr;
  QPushButton *confidence_apply_button_ = nullptr;
  QPushButton *confidence_restore_button_ = nullptr;
  QPushButton *confidence_undo_button_ = nullptr;
  QPushButton *confidence_redo_button_ = nullptr;
  QPushButton *confidence_save_button_ = nullptr;
  QPushButton *confidence_rebuild_button_ = nullptr;
  QAction *confidence_save_action_ = nullptr;
  QAction *confidence_rebuild_action_ = nullptr;
  WorkflowPanel *workflow_panel_ = nullptr;
  QComboBox *selection_tool_combo_ = nullptr;
  QCheckBox *z_window_check_ = nullptr;
  QDoubleSpinBox *z_min_spin_ = nullptr;
  QDoubleSpinBox *z_max_spin_ = nullptr;
  QDoubleSpinBox *sphere_radius_spin_ = nullptr;
  QAction *hide_deleted_action_ = nullptr;
  QAction *isolate_selection_action_ = nullptr;
  QAction *show_axis_action_ = nullptr;
  QAction *dark_background_action_ = nullptr;
  QAction *height_coloring_action_ = nullptr;
  QAction *solid_coloring_action_ = nullptr;
  QAction *show_3d_action_ = nullptr;
  QAction *show_2d_action_ = nullptr;
  QAction *stable_only_action_ = nullptr;
  QVector<QAction *> confidence_color_actions_;
  QVector<QAction *> geometry_color_actions_;
  QAction *mode_navigate_action_ = nullptr;
  QAction *mode_select_action_ = nullptr;
  QAction *mode_delete_action_ = nullptr;
  QAction *delete_points_action_ = nullptr;
  QLabel *edit_state_label_ = nullptr;
  QMenu *view_menu_ = nullptr;
  QMenu *tools_menu_ = nullptr;
  QAction *confirm_review_action_ = nullptr;

  RefinementModel refinement_model_;
  SelectionManager selection_manager_;
  SpatialConfidenceModel confidence_model_;
  GeometryEvidenceModel geometry_model_;
  SpatialConfidenceEditor confidence_editor_;
  SelectionManager confidence_selection_manager_;
  QString confidence_derivative_dir_;
  QString loaded_confidence_checksums_sha256_;
  QString saved_confidence_intent_path_;
  QString saved_confidence_intent_sha256_;
  QString pending_review_target_;
  QString pending_review_parent_;
  std::size_t pending_review_stable_count_ = 0;
  WorkflowSession session_;
  ExternalToolRunner tool_runner_;
  ExternalToolRunner confidence_review_runner_;
  std::function<void(const ToolResult &)> tool_callback_;
  std::vector<StepFn> step_queue_;
  bool queue_running_ = false;
  QString source_path_;
  QString default_map_root_;
  QString review_base_map_;
  QString review_output_;
  bool review_mode_ = false;
};

}  // namespace agt_map_studio
