#include "ui/MainWindow.hpp"

#include "io/PCDLoader.hpp"
#include "confidence/SpatialConfidenceLoader.hpp"
#include "confidence/SpatialConfidenceIntentIO.hpp"
#include "geometry/GeometryEvidenceLoader.hpp"
#include "occupancy/MapYamlLoader.hpp"
#include "occupancy/commands/DrawObstacleCommand.hpp"
#include "occupancy/commands/EraseRectangleCommand.hpp"
#include "occupancy/commands/FillPolygonCommand.hpp"
#include "occupancy/commands/ForbiddenPolygonCommand.hpp"
#include "ui/WorkflowPanel.hpp"

#include <agt_pcd2grid_exporter/OccupancyGridWriter.hpp>
#include <agt_pcd2grid_exporter/PCDProjector.hpp>
#include <agt_pcd2grid_exporter/ParameterLoader.hpp>
#include <agt_spatial_map_core/spatial_export.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <QAction>
#include <QActionGroup>
#include <QAbstractItemView>
#include <QApplication>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDesktopServices>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QImage>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPixmap>
#include <QProcess>
#include <QProcessEnvironment>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStatusBar>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QToolBar>
#include <QHeaderView>
#include <QUuid>
#include <QUrl>
#include <QVBoxLayout>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace agt_map_studio {

namespace {

constexpr const char *kRefinementPackage = "agt_map_refinement_core";
constexpr const char *kRefinementTool = "apply_map_refinement";
constexpr const char *kRelocPackage = "agt_global_relocalization_native";
constexpr const char *kRelocTool = "build_relocalization_assets";
constexpr const char *kConverterPackage = "agt_map_converter";
constexpr const char *kManagerPackage = "agt_map_manager";

bool yaml_has_value(const YAML::Node &node) {
  return node.IsDefined() && !node.IsNull();
}

QString yaml_number_or_unknown(const YAML::Node &node, int precision) {
  return yaml_has_value(node)
      ? QString::number(node.as<double>(), 'f', precision)
      : QStringLiteral("UNKNOWN");
}

QString yaml_boolean_or_unknown(const YAML::Node &node,
                                const QString &true_value,
                                const QString &false_value) {
  if (!yaml_has_value(node)) return QStringLiteral("UNKNOWN");
  return node.as<bool>() ? true_value : false_value;
}

QString gicp_state(const YAML::Node &attempted, const YAML::Node &converged) {
  if (!yaml_has_value(attempted)) return QStringLiteral("UNKNOWN");
  if (!attempted.as<bool>()) return QStringLiteral("NOT ATTEMPTED");
  if (!yaml_has_value(converged)) return QStringLiteral("UNKNOWN");
  return converged.as<bool>() ? QStringLiteral("CONVERGED")
                              : QStringLiteral("NOT CONVERGED");
}

std::filesystem::path normalized_path(const QString &path) {
  std::error_code error;
  const auto absolute = std::filesystem::absolute(path.toStdString(), error);
  if (error) return std::filesystem::path(path.toStdString()).lexically_normal();
  const auto canonical = std::filesystem::weakly_canonical(absolute, error);
  return error ? absolute.lexically_normal() : canonical;
}

bool path_is_same_or_within(const QString &candidate, const QString &root) {
  const auto child = normalized_path(candidate);
  const auto boundary = normalized_path(root);
  auto child_part = child.begin();
  for (auto boundary_part = boundary.begin(); boundary_part != boundary.end(); ++boundary_part, ++child_part) {
    if (child_part == child.end() || *child_part != *boundary_part) return false;
  }
  return true;
}

bool validate_mapping_parent(const QString &directory, QString *error) {
  // Same validator invoked by agt_spatial_map_export. No shell, no write,
  // fail closed on non-PGO / incomplete / invalid-checksum packages.
  QProcess process;
  process.setProgram(QStringLiteral("python3"));
  process.setArguments({QStringLiteral("-m"),
                        QStringLiteral("agt_mapping_artifacts.validation"), directory});
  process.start();
  if (!process.waitForStarted(5000)) {
    if (error) *error = QStringLiteral("Could not start mapping package validator: %1")
                            .arg(process.errorString());
    return false;
  }
  if (!process.waitForFinished(180000)) {
    process.kill();
    process.waitForFinished(5000);
    if (error) *error = QStringLiteral("Mapping package validation timed out");
    return false;
  }
  if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0) {
    if (error) *error = QStringLiteral("Mapping package / PGO / checksum validation failed: %1")
                            .arg(QString::fromUtf8(process.readAllStandardError()).left(1000));
    return false;
  }
  return true;
}

}  // namespace

MainWindow::MainWindow(const QString &config_path, QWidget *parent)
    : QMainWindow(parent), viewer_(new PointCloudViewer(this)),
      occupancy_viewer_(new OccupancyViewer(this)),
      view_stack_(new QStackedWidget(this)) {
  setWindowTitle(QStringLiteral("AGT Map Studio"));
  resize(1480, 900);
  view_stack_->addWidget(viewer_);
  view_stack_->addWidget(occupancy_viewer_);
  view_stack_->setCurrentWidget(viewer_);
  setCentralWidget(view_stack_);
  viewer_->set_selection_manager(&selection_manager_);
  viewer_->set_confidence_selection_manager(&confidence_selection_manager_);
  occupancy_viewer_->set_refinement_model(&refinement_model_);
  default_map_root_ = QProcessEnvironment::systemEnvironment().value(
      QStringLiteral("AGT_MAP_ROOT"), QDir::home().filePath(QStringLiteral("ros2_ws/maps")));
  create_actions();
  create_workflow_dock();
  create_confidence_dock();
  create_relocalization_dock();
  load_config(config_path);
  session_.publish_target().map_root = default_map_root_;
  workflow_panel_->set_publish_target(session_.publish_target());

  connect(viewer_, &PointCloudViewer::stats_changed, this, &MainWindow::show_stats);
  connect(viewer_, &PointCloudViewer::confidence_voxel_selected, this,
          &MainWindow::inspect_confidence_voxel);
  connect(viewer_, &PointCloudViewer::map_point_selected, this,
          [this](double x, double y, double) {
    picked_query_x_ = x;
    picked_query_y_ = y;
    has_picked_query_point_ = true;
    if (query_x_edit_) query_x_edit_->setText(QString::number(x, 'f', 3));
    if (query_y_edit_) query_y_edit_->setText(QString::number(y, 'f', 3));
    if (query_mode_combo_) query_mode_combo_->setCurrentIndex(1);
    if (relocalization_status_label_) {
      relocalization_status_label_->setText(
          QStringLiteral("Map click stored; query will snap to nearest keyframe and record distance."));
    }
  });
  connect(viewer_, &PointCloudViewer::delete_requested_outside_delete_mode, this, [this]() {
    statusBar()->showMessage(viewer_->showing_confidence()
        ? QStringLiteral("Confidence voxels cannot be deleted; use override intent in the editor")
        : QStringLiteral("Switch to Delete mode (toolbar or X) before pressing Delete"), 4000);
  });
  connect(occupancy_viewer_, &OccupancyViewer::status_changed, this, &MainWindow::show_stats);
  connect(occupancy_viewer_, &OccupancyViewer::erase_rectangle_requested, this,
          &MainWindow::apply_erase_rectangle);
  connect(occupancy_viewer_, &OccupancyViewer::obstacle_line_requested, this,
          &MainWindow::apply_obstacle_line);
  connect(occupancy_viewer_, &OccupancyViewer::forbidden_polygon_requested, this,
          &MainWindow::apply_forbidden_polygon);
  connect(occupancy_viewer_, &OccupancyViewer::fill_polygon_requested, this,
          &MainWindow::apply_fill_polygon);

  connect(&tool_runner_, &ExternalToolRunner::started, this, [this](const QString &label) {
    workflow_panel_->set_progress(QStringLiteral("Running: %1").arg(label), true);
    statusBar()->showMessage(QStringLiteral("Running %1 ...").arg(label));
    refresh_workflow();
  });
  connect(&tool_runner_, &ExternalToolRunner::output_appended, workflow_panel_,
          &WorkflowPanel::append_log);
  connect(&tool_runner_, &ExternalToolRunner::finished, this, [this](const ToolResult &result) {
    workflow_panel_->set_progress(result.ok ? QStringLiteral("Idle") : QStringLiteral("Failed"),
                                  false);
    auto callback = std::move(tool_callback_);
    tool_callback_ = nullptr;
    if (callback) callback(result);
    refresh_workflow();
  });
  connect(&confidence_review_runner_, &ExternalToolRunner::started, this,
          [this](const QString &) {
    statusBar()->showMessage(QStringLiteral("Core is rebuilding a separate reviewed derivative..."));
    refresh_confidence_editor_ui();
  });
  connect(&confidence_review_runner_, &ExternalToolRunner::output_appended,
          workflow_panel_, &WorkflowPanel::append_log);
  connect(&confidence_review_runner_, &ExternalToolRunner::finished, this,
          [this](const ToolResult &result) {
    const QString output = std::exchange(pending_review_target_, QString());
    const QString parent = std::exchange(pending_review_parent_, QString());
    const std::size_t expected = pending_review_stable_count_;
    pending_review_stable_count_ = 0;
    refresh_confidence_editor_ui();
    if (!result.ok) {
      const QString detail = result.error_summary.isEmpty()
          ? QStringLiteral("Core exited without a success result") : result.error_summary;
      statusBar()->showMessage(QStringLiteral("Core review failed; original files unchanged"), 10000);
      QMessageBox::critical(this, QStringLiteral("Reviewed derivative rebuild failed"),
          detail + QStringLiteral("\n\nNo production or navigation map was published."));
      return;
    }
    SpatialConfidenceModel checked;
    std::string validation_error;
    if (!SpatialConfidenceLoader::load(output.toStdString(), parent.toStdString(),
                                       &checked, &validation_error) ||
        checked.stable_preview_count() != expected) {
      statusBar()->showMessage(QStringLiteral("Core wrote derivative; verification needs attention: %1")
                                   .arg(output), 12000);
      QMessageBox::warning(this, QStringLiteral("Reviewed output verification failed"),
          QStringLiteral("Core reported success at %1, but Studio could not verify its "
                         "checksum-covered content / preview count (%2 expected): %3. "
                         "Do not use this derivative until investigated.")
              .arg(output).arg(static_cast<qulonglong>(expected))
              .arg(QString::fromStdString(validation_error)));
      return;
    }
    statusBar()->showMessage(QStringLiteral("Verified reviewed derivative: %1 (stable %2)")
                                 .arg(output).arg(static_cast<qulonglong>(expected)), 12000);
    QMessageBox::information(this, QStringLiteral("Reviewed derivative verified"),
        QStringLiteral("Core created and Studio re-verified a NEW reviewed derivative:\n%1\n"
                       "Stable voxels: %2\n\nThe source derivative, PGO package and "
                       "production navigation maps were not replaced or published. "
                       "Open the new derivative explicitly to inspect it.")
            .arg(output).arg(static_cast<qulonglong>(expected)));
  });
  statusBar()->showMessage(viewer_->stats_text());
  edit_state_label_ = new QLabel(this);
  statusBar()->addPermanentWidget(edit_state_label_);
  refresh_workflow();
}

void MainWindow::create_actions() {
  auto *open_action = new QAction(QStringLiteral("Open PCD..."), this);
  open_action->setShortcut(QKeySequence::Open);
  connect(open_action, &QAction::triggered, this, &MainWindow::open_pcd_dialog);
  auto *open_package_action = new QAction(QStringLiteral("Open Mapping / Map Package..."), this);
  open_package_action->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_O));
  connect(open_package_action, &QAction::triggered, this, &MainWindow::open_mapping_package_dialog);
  auto *open_confidence_action = new QAction(
      QStringLiteral("Open Spatial Confidence Derivative (source read-only)..."), this);
  connect(open_confidence_action, &QAction::triggered, this,
          &MainWindow::open_spatial_confidence_dialog);
  confidence_save_action_ = new QAction(
      QStringLiteral("Save Spatial Override Intent YAML..."), this);
  confidence_save_action_->setEnabled(false);
  connect(confidence_save_action_, &QAction::triggered,
          this, &MainWindow::save_confidence_overrides_dialog);
  confidence_rebuild_action_ = new QAction(
      QStringLiteral("Rebuild Reviewed Spatial Derivative (core, new directory)..."), this);
  confidence_rebuild_action_->setEnabled(false);
  connect(confidence_rebuild_action_, &QAction::triggered,
          this, &MainWindow::rebuild_confidence_review_dialog);
  auto *open_geometry_action = new QAction(
      QStringLiteral("Open Geometry Evidence Sidecar (read-only)..."), this);
  connect(open_geometry_action, &QAction::triggered, this,
          &MainWindow::open_geometry_evidence_dialog);
  auto *open_occupancy_action = new QAction(QStringLiteral("Open Occupancy Map (map.yaml)..."), this);
  connect(open_occupancy_action, &QAction::triggered, this, &MainWindow::open_occupancy_map_dialog);
  auto *open_session_action = new QAction(QStringLiteral("Open Studio Session..."), this);
  connect(open_session_action, &QAction::triggered, this, &MainWindow::open_session_dialog);
  auto *save_session_action = new QAction(QStringLiteral("Save Studio Session"), this);
  save_session_action->setShortcut(QKeySequence::Save);
  connect(save_session_action, &QAction::triggered, this, [this]() {
    QString error;
    if (!ensure_work_dir(&error) || !session_.save(&error)) {
      QMessageBox::critical(this, QStringLiteral("Save Session failed"), error);
      return;
    }
    statusBar()->showMessage(QStringLiteral("Saved session: %1").arg(session_.session_file()), 5000);
  });
  auto *save_view_action = new QAction(QStringLiteral("Save Camera View..."), this);
  connect(save_view_action, &QAction::triggered, this, &MainWindow::save_view_dialog);

  auto *export_rules_action = new QAction(QStringLiteral("Export 3D Refinement Rules (refinement.yaml)..."), this);
  connect(export_rules_action, &QAction::triggered, this, &MainWindow::export_refinement_rules_dialog);
  auto *export_clean_action = new QAction(QStringLiteral("Export Clean Map (preview)..."), this);
  connect(export_clean_action, &QAction::triggered, this, &MainWindow::export_clean_map_dialog);
  auto *export_patch_action = new QAction(QStringLiteral("Export 2D Patch (patch_nav_map YAML)..."), this);
  connect(export_patch_action, &QAction::triggered, this, &MainWindow::export_navigation_patch_dialog);
  auto *save_refinement_action = new QAction(QStringLiteral("Save 2D Refinement History..."), this);
  connect(save_refinement_action, &QAction::triggered, this, &MainWindow::save_refinement_dialog);
  auto *export_navigation_action = new QAction(QStringLiteral("Export Edited PGM (preview)..."), this);
  connect(export_navigation_action, &QAction::triggered, this, &MainWindow::export_navigation_map_dialog);
  confirm_review_action_ = new QAction(QStringLiteral("Confirm && Save 2D Map"), this);
  confirm_review_action_->setEnabled(false);
  connect(confirm_review_action_, &QAction::triggered, this, &MainWindow::confirm_mapping_review);

  auto *quit_action = new QAction(QStringLiteral("Quit"), this);
  quit_action->setShortcut(QKeySequence::Quit);
  connect(quit_action, &QAction::triggered, this, &QWidget::close);

  auto *file_menu = menuBar()->addMenu(QStringLiteral("File"));
  file_menu->addAction(open_action);
  file_menu->addAction(open_package_action);
  file_menu->addAction(open_confidence_action);
  file_menu->addAction(open_geometry_action);
  file_menu->addAction(confidence_save_action_);
  file_menu->addAction(confidence_rebuild_action_);
  file_menu->addAction(open_occupancy_action);
  file_menu->addSeparator();
  file_menu->addAction(open_session_action);
  file_menu->addAction(save_session_action);
  file_menu->addAction(save_view_action);
  file_menu->addSeparator();
  file_menu->addAction(confirm_review_action_);
  auto *export_menu = file_menu->addMenu(QStringLiteral("Export"));
  export_menu->addAction(export_rules_action);
  export_menu->addAction(export_patch_action);
  export_menu->addSeparator();
  export_menu->addAction(export_clean_action);
  export_menu->addAction(save_refinement_action);
  export_menu->addAction(export_navigation_action);
  file_menu->addSeparator();
  file_menu->addAction(quit_action);

  // Edit
  auto *undo_action = new QAction(QStringLiteral("Undo"), this);
  undo_action->setShortcut(QKeySequence::Undo);
  connect(undo_action, &QAction::triggered, this, &MainWindow::undo_edit);
  auto *redo_action = new QAction(QStringLiteral("Redo"), this);
  redo_action->setShortcuts({QKeySequence::Redo, QKeySequence(Qt::CTRL | Qt::Key_Y)});
  connect(redo_action, &QAction::triggered, this, &MainWindow::redo_edit);
  delete_points_action_ = new QAction(QStringLiteral("Delete Selected Points"), this);
  delete_points_action_->setShortcut(Qt::Key_Delete);
  connect(delete_points_action_, &QAction::triggered, this, &MainWindow::delete_selected);
  auto *invert_action = new QAction(QStringLiteral("Invert Selection"), this);
  invert_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_I));
  connect(invert_action, &QAction::triggered, this, [this]() {
    (viewer_->showing_confidence() ? confidence_selection_manager_
                                   : selection_manager_).invert_selection();
    viewer_->rebuild_selection_box_from_points();
  });
  auto *clear_selection_action = new QAction(QStringLiteral("Clear Selection"), this);
  connect(clear_selection_action, &QAction::triggered, this, [this]() {
    (viewer_->showing_confidence() ? confidence_selection_manager_
                                   : selection_manager_).clear_selection();
    viewer_->cancel_pending_polygon();
    viewer_->mark_edit_state_dirty();
  });
  auto *height_band_action = new QAction(QStringLiteral("Select Height Band..."), this);
  height_band_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_H));
  connect(height_band_action, &QAction::triggered, this, &MainWindow::select_height_band_dialog);
  hide_deleted_action_ = new QAction(QStringLiteral("Hide Deleted Points"), this);
  hide_deleted_action_->setCheckable(true);
  hide_deleted_action_->setShortcut(Qt::Key_H);
  connect(hide_deleted_action_, &QAction::toggled, this, [this](bool checked) {
    selection_manager_.set_hide_deleted(checked);
    viewer_->mark_edit_state_dirty();
  });
  isolate_selection_action_ = new QAction(QStringLiteral("Isolate Selection"), this);
  isolate_selection_action_->setCheckable(true);
  isolate_selection_action_->setShortcut(Qt::Key_I);
  connect(isolate_selection_action_, &QAction::toggled, this, [this](bool checked) {
    (viewer_->showing_confidence() ? confidence_selection_manager_
                                   : selection_manager_).set_isolate_selected(checked);
    viewer_->mark_edit_state_dirty();
  });

  auto *edit_menu = menuBar()->addMenu(QStringLiteral("Edit"));
  edit_menu->addAction(undo_action);
  edit_menu->addAction(redo_action);
  edit_menu->addSeparator();
  edit_menu->addAction(delete_points_action_);
  edit_menu->addAction(invert_action);
  edit_menu->addAction(clear_selection_action);
  edit_menu->addAction(height_band_action);
  edit_menu->addSeparator();
  edit_menu->addAction(hide_deleted_action_);
  edit_menu->addAction(isolate_selection_action_);

  // View
  auto *reset_action = new QAction(QStringLiteral("Reset Camera"), this);
  reset_action->setShortcut(Qt::Key_R);
  connect(reset_action, &QAction::triggered, this, &MainWindow::reset_camera);
  auto *isometric_action = new QAction(QStringLiteral("Isometric View"), this);
  isometric_action->setShortcut(Qt::Key_0);
  connect(isometric_action, &QAction::triggered, this, &MainWindow::set_isometric_view);
  auto *front_action = new QAction(QStringLiteral("Front View"), this);
  front_action->setShortcut(Qt::Key_1);
  connect(front_action, &QAction::triggered, this, &MainWindow::set_front_view);
  auto *top_action = new QAction(QStringLiteral("Top View"), this);
  top_action->setShortcut(Qt::Key_2);
  connect(top_action, &QAction::triggered, this, &MainWindow::set_top_view);
  show_axis_action_ = new QAction(QStringLiteral("Show Axis"), this);
  show_axis_action_->setCheckable(true);
  show_axis_action_->setChecked(true);
  connect(show_axis_action_, &QAction::toggled, this, &MainWindow::toggle_axis);
  dark_background_action_ = new QAction(QStringLiteral("Dark Background"), this);
  dark_background_action_->setCheckable(true);
  connect(dark_background_action_, &QAction::toggled, this, &MainWindow::toggle_background);
  height_coloring_action_ = new QAction(QStringLiteral("Color by Z Height"), this);
  height_coloring_action_->setCheckable(true);
  height_coloring_action_->setChecked(true);
  auto *color_group = new QActionGroup(this);
  color_group->setExclusive(true);
  color_group->addAction(height_coloring_action_);
  connect(height_coloring_action_, &QAction::triggered, this,
          [this]() { set_point_color_mode(PointColorMode::Height); });
  solid_coloring_action_ = new QAction(QStringLiteral("Solid Point Color"), this);
  solid_coloring_action_->setCheckable(true);
  color_group->addAction(solid_coloring_action_);
  connect(solid_coloring_action_, &QAction::triggered, this,
          [this]() { set_point_color_mode(PointColorMode::Solid); });
  const struct { const char *label; PointColorMode mode; } confidence_colors[] = {
      {"Auto Confidence (single-session)", PointColorMode::AutoConfidence},
      {"Final Confidence (review preview)", PointColorMode::FinalConfidence},
      {"Observation Score (not probability)", PointColorMode::ObservationScore},
      {"Persistence Evidence (not probability)", PointColorMode::PersistenceScore},
  };
  for (const auto &entry : confidence_colors) {
    auto *action = new QAction(QString::fromLatin1(entry.label), this);
    action->setCheckable(true);
    action->setEnabled(false);
    color_group->addAction(action);
    confidence_color_actions_.push_back(action);
    const PointColorMode mode = entry.mode;
    connect(action, &QAction::triggered, this, [this, mode]() {
      set_point_color_mode(mode);
      show_3d_view();
      if (confidence_dock_) confidence_dock_->show();
      inspect_confidence_voxel(confidence_selection_manager_.selected_indices().empty()
          ? static_cast<std::size_t>(-1)
          : confidence_selection_manager_.selected_indices().front());
    });
  }
  const struct { const char *label; PointColorMode mode; } geometry_colors[] = {
      {"Geometry PCA shape (RGB: linearity/planarity/scattering)", PointColorMode::GeometryNormalShape},
      {"Geometry Ht Q (directional diversity, not confidence)", PointColorMode::GeometryTranslationQ},
      {"Geometry Hr Q (rotation diversity, not confidence)", PointColorMode::GeometryRotationQ},
      {"Geometry Ht weak axis (RGB: map |x/y/z|)", PointColorMode::GeometryTranslationWeak},
      {"Geometry Hr weak axis (RGB: map |x/y/z|)", PointColorMode::GeometryRotationWeak},
  };
  for (const auto &entry : geometry_colors) {
    auto *action = new QAction(QString::fromLatin1(entry.label), this);
    action->setCheckable(true);
    action->setEnabled(false);
    color_group->addAction(action);
    geometry_color_actions_.push_back(action);
    const PointColorMode mode = entry.mode;
    connect(action, &QAction::triggered, this, [this, mode]() {
      set_point_color_mode(mode);
      show_3d_view();
      if (confidence_dock_) confidence_dock_->show();
      inspect_confidence_voxel(confidence_selection_manager_.selected_indices().empty()
          ? static_cast<std::size_t>(-1)
          : confidence_selection_manager_.selected_indices().front());
    });
  }
  stable_only_action_ = new QAction(QStringLiteral("Show Stable Preview Only"), this);
  stable_only_action_->setCheckable(true);
  stable_only_action_->setEnabled(false);
  stable_only_action_->setToolTip(QStringLiteral(
      "Preview from final_confidence >= stable_threshold, excluding FORCE_LOW and IGNORE; "
      "formal stable_map.pcd is only rebuilt by the spatial core"));
  connect(stable_only_action_, &QAction::toggled, viewer_, &PointCloudViewer::set_stable_only);
  auto *increase_point_action = new QAction(QStringLiteral("Increase Point Size"), this);
  increase_point_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_Plus));
  connect(increase_point_action, &QAction::triggered, this, [this]() { viewer_->adjust_point_size(0.5F); });
  auto *decrease_point_action = new QAction(QStringLiteral("Decrease Point Size"), this);
  decrease_point_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_Minus));
  connect(decrease_point_action, &QAction::triggered, this, [this]() { viewer_->adjust_point_size(-0.5F); });
  auto *default_point_action = new QAction(QStringLiteral("Default Point Size"), this);
  connect(default_point_action, &QAction::triggered, this, [this]() { viewer_->set_point_size(2.0F); });

  auto *view_menu = menuBar()->addMenu(QStringLiteral("View"));
  view_menu_ = view_menu;
  auto *view_group = new QActionGroup(this);
  view_group->setExclusive(true);
  show_3d_action_ = new QAction(QStringLiteral("3D Point Cloud"), this);
  show_2d_action_ = new QAction(QStringLiteral("2D Navigation Map"), this);
  for (auto *action : {show_3d_action_, show_2d_action_}) {
    action->setCheckable(true);
    view_group->addAction(action);
  }
  show_3d_action_->setChecked(true);
  show_3d_action_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_1));
  show_2d_action_->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_2));
  connect(show_3d_action_, &QAction::triggered, this, &MainWindow::show_3d_view);
  connect(show_2d_action_, &QAction::triggered, this, &MainWindow::show_2d_view);
  view_menu->addAction(show_3d_action_);
  view_menu->addAction(show_2d_action_);
  view_menu->addSeparator();
  view_menu->addAction(reset_action);
  view_menu->addAction(isometric_action);
  view_menu->addAction(front_action);
  view_menu->addAction(top_action);
  view_menu->addSeparator();
  view_menu->addAction(show_axis_action_);
  view_menu->addAction(dark_background_action_);
  view_menu->addAction(height_coloring_action_);
  view_menu->addAction(solid_coloring_action_);
  view_menu->addSeparator();
  for (auto *action : confidence_color_actions_) view_menu->addAction(action);
  view_menu->addSeparator();
  for (auto *action : geometry_color_actions_) view_menu->addAction(action);
  view_menu->addAction(stable_only_action_);
  auto *point_menu = view_menu->addMenu(QStringLiteral("Point Size"));
  point_menu->addAction(increase_point_action);
  point_menu->addAction(decrease_point_action);
  point_menu->addAction(default_point_action);

  // Tools
  auto *tools_menu = menuBar()->addMenu(QStringLiteral("Tools"));
  tools_menu_ = tools_menu;
  auto *preview_action = new QAction(QStringLiteral("Quick Occupancy Preview (studio projector)..."), this);
  connect(preview_action, &QAction::triggered, this, &MainWindow::generate_occupancy_preview_dialog);
  tools_menu->addAction(preview_action);
  tools_menu->addSeparator();
  auto *run_refine_action = new QAction(QStringLiteral("1. Apply 3D Refinement"), this);
  connect(run_refine_action, &QAction::triggered, this, &MainWindow::run_refine);
  auto *run_reloc_action = new QAction(QStringLiteral("2. Build Relocalization Assets"), this);
  connect(run_reloc_action, &QAction::triggered, this, &MainWindow::run_relocalization);
  auto *run_nav_action = new QAction(QStringLiteral("3. Generate Navigation Layers"), this);
  connect(run_nav_action, &QAction::triggered, this, &MainWindow::run_navigation);
  auto *run_patch_action = new QAction(QStringLiteral("4. Apply 2D Patch"), this);
  connect(run_patch_action, &QAction::triggered, this, &MainWindow::run_patch);
  auto *run_publish_action = new QAction(QStringLiteral("5. Publish Map Package"), this);
  connect(run_publish_action, &QAction::triggered, this, &MainWindow::run_publish);
  auto *run_all_action = new QAction(QStringLiteral("Run All Pending Steps"), this);
  run_all_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_Return));
  connect(run_all_action, &QAction::triggered, this, &MainWindow::run_all_pending);
  for (auto *action : {run_refine_action, run_reloc_action, run_nav_action, run_patch_action,
                       run_publish_action, run_all_action}) {
    tools_menu->addAction(action);
  }

  auto *help_menu = menuBar()->addMenu(QStringLiteral("Help"));
  auto *controls_action = new QAction(QStringLiteral("Controls"), this);
  controls_action->setShortcut(Qt::Key_F1);
  connect(controls_action, &QAction::triggered, this, &MainWindow::show_controls);
  help_menu->addAction(controls_action);
  auto *workflow_help_action = new QAction(QStringLiteral("Publish Workflow"), this);
  connect(workflow_help_action, &QAction::triggered, this, &MainWindow::show_workflow_help);
  help_menu->addAction(workflow_help_action);

  // 3D toolbar: mode + selection tool + z window + view helpers
  toolbar_3d_ = addToolBar(QStringLiteral("3D Edit"));
  toolbar_3d_->setMovable(false);
  auto *mode_group = new QActionGroup(this);
  mode_group->setExclusive(true);
  mode_navigate_action_ = toolbar_3d_->addAction(QStringLiteral("Navigate (N)"));
  mode_select_action_ = toolbar_3d_->addAction(QStringLiteral("Select (B)"));
  mode_delete_action_ = toolbar_3d_->addAction(QStringLiteral("Delete (X)"));
  mode_navigate_action_->setShortcut(Qt::Key_N);
  mode_select_action_->setShortcut(Qt::Key_B);
  mode_delete_action_->setShortcut(Qt::Key_X);
  for (auto *action : {mode_navigate_action_, mode_select_action_, mode_delete_action_}) {
    action->setCheckable(true);
    mode_group->addAction(action);
  }
  mode_navigate_action_->setChecked(true);
  connect(mode_navigate_action_, &QAction::triggered, this, &MainWindow::set_mode_navigate);
  connect(mode_select_action_, &QAction::triggered, this, &MainWindow::set_mode_select);
  connect(mode_delete_action_, &QAction::triggered, this, &MainWindow::set_mode_delete);
  toolbar_3d_->addSeparator();
  toolbar_3d_->addWidget(new QLabel(QStringLiteral(" Tool: "), this));
  selection_tool_combo_ = new QComboBox(this);
  selection_tool_combo_->addItem(QStringLiteral("Rectangle (drag)"), static_cast<int>(SelectionTool::ScreenRect));
  selection_tool_combo_->addItem(QStringLiteral("Polygon (click, double-click to close)"), static_cast<int>(SelectionTool::PolygonPrism));
  selection_tool_combo_->addItem(QStringLiteral("Sphere (click)"), static_cast<int>(SelectionTool::Sphere));
  selection_tool_combo_->setToolTip(QStringLiteral("How points are selected in Select/Delete mode"));
  toolbar_3d_->addWidget(selection_tool_combo_);
  connect(selection_tool_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int index) {
    viewer_->set_selection_tool(static_cast<SelectionTool>(selection_tool_combo_->itemData(index).toInt()));
    sphere_radius_spin_->setEnabled(viewer_->selection_tool() == SelectionTool::Sphere);
  });
  toolbar_3d_->addWidget(new QLabel(QStringLiteral(" r(m) "), this));
  sphere_radius_spin_ = new QDoubleSpinBox(this);
  sphere_radius_spin_->setRange(0.05, 20.0);
  sphere_radius_spin_->setSingleStep(0.1);
  sphere_radius_spin_->setDecimals(2);
  sphere_radius_spin_->setValue(0.5);
  sphere_radius_spin_->setEnabled(false);
  sphere_radius_spin_->setToolTip(QStringLiteral("Sphere selection radius"));
  toolbar_3d_->addWidget(sphere_radius_spin_);
  connect(sphere_radius_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          [this](double value) { viewer_->set_sphere_radius(value); viewer_->mark_edit_state_dirty(); });
  toolbar_3d_->addSeparator();
  z_window_check_ = new QCheckBox(QStringLiteral("Z window"), this);
  z_window_check_->setToolTip(QStringLiteral("Limit rectangle/polygon selections to a height band"));
  toolbar_3d_->addWidget(z_window_check_);
  z_min_spin_ = new QDoubleSpinBox(this);
  z_min_spin_->setRange(-1000.0, 1000.0);
  z_min_spin_->setDecimals(2);
  z_min_spin_->setValue(-1.0);
  z_min_spin_->setPrefix(QStringLiteral("min "));
  z_max_spin_ = new QDoubleSpinBox(this);
  z_max_spin_->setRange(-1000.0, 1000.0);
  z_max_spin_->setDecimals(2);
  z_max_spin_->setValue(3.0);
  z_max_spin_->setPrefix(QStringLiteral("max "));
  toolbar_3d_->addWidget(z_min_spin_);
  toolbar_3d_->addWidget(z_max_spin_);
  const auto apply_z_window = [this]() {
    viewer_->set_z_window(z_window_check_->isChecked(), z_min_spin_->value(), z_max_spin_->value());
  };
  connect(z_window_check_, &QCheckBox::toggled, this, [apply_z_window](bool) { apply_z_window(); });
  connect(z_min_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [apply_z_window](double) { apply_z_window(); });
  connect(z_max_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [apply_z_window](double) { apply_z_window(); });
  toolbar_3d_->addSeparator();
  toolbar_3d_->addAction(hide_deleted_action_);
  toolbar_3d_->addAction(isolate_selection_action_);
  toolbar_3d_->addAction(delete_points_action_);

  // 2D toolbar
  occupancy_toolbar_ = addToolBar(QStringLiteral("2D Edit"));
  occupancy_toolbar_->setMovable(false);
  auto *occupancy_group = new QActionGroup(this);
  occupancy_group->setExclusive(true);
  struct ModeEntry {
    const char *label;
    OccupancyInteractionMode mode;
    const char *tip;
  };
  const ModeEntry entries[] = {
      {"View", OccupancyInteractionMode::View, "Pan/zoom only"},
      {"Erase rect", OccupancyInteractionMode::Erase, "Drag: occupied -> free"},
      {"Obstacle line", OccupancyInteractionMode::Obstacle, "Drag a line of given width -> occupied"},
      {"Free polygon", OccupancyInteractionMode::FreePolygon, "Click vertices, double-click: fill free"},
      {"Occupied polygon", OccupancyInteractionMode::OccupiedPolygon, "Click vertices, double-click: fill occupied"},
      {"Unknown polygon", OccupancyInteractionMode::UnknownPolygon, "Click vertices, double-click: fill unknown"},
      {"Forbidden zone", OccupancyInteractionMode::Forbidden, "Keep-out polygon (exported to keepout_zones.yaml)"},
  };
  bool first = true;
  for (const auto &entry : entries) {
    auto *action = occupancy_toolbar_->addAction(QString::fromUtf8(entry.label));
    action->setToolTip(QString::fromUtf8(entry.tip));
    action->setCheckable(true);
    occupancy_group->addAction(action);
    if (first) action->setChecked(true);
    first = false;
    const OccupancyInteractionMode mode = entry.mode;
    connect(action, &QAction::triggered, this, [this, mode]() { set_occupancy_mode(mode); });
  }
  occupancy_toolbar_->addSeparator();
  occupancy_toolbar_->addWidget(new QLabel(QStringLiteral("Width (m):"), this));
  auto *width_spin = new QDoubleSpinBox(this);
  width_spin->setRange(0.01, 10.0);
  width_spin->setSingleStep(0.05);
  width_spin->setDecimals(2);
  width_spin->setValue(0.20);
  width_spin->setToolTip(QStringLiteral("Obstacle line width in meters"));
  occupancy_toolbar_->addWidget(width_spin);
  connect(width_spin, qOverload<double>(&QDoubleSpinBox::valueChanged), occupancy_viewer_,
          &OccupancyViewer::set_obstacle_width);
  occupancy_toolbar_->addSeparator();
  auto *finish_polygon_action = occupancy_toolbar_->addAction(QStringLiteral("Close polygon"));
  connect(finish_polygon_action, &QAction::triggered, occupancy_viewer_, &OccupancyViewer::finish_polygon);
  auto *undo_vertex_action = occupancy_toolbar_->addAction(QStringLiteral("Undo vertex"));
  connect(undo_vertex_action, &QAction::triggered, occupancy_viewer_, &OccupancyViewer::pop_polygon_vertex);
  auto *fit_action = occupancy_toolbar_->addAction(QStringLiteral("Fit (F)"));
  connect(fit_action, &QAction::triggered, occupancy_viewer_, &OccupancyViewer::fit_map);
  occupancy_toolbar_->addSeparator();
  occupancy_toolbar_->addAction(confirm_review_action_);
  occupancy_toolbar_->setVisible(false);
}

void MainWindow::create_workflow_dock() {
  workflow_panel_ = new WorkflowPanel(this);
  workflow_dock_ = new QDockWidget(QStringLiteral("Publish Workflow"), this);
  workflow_dock_->setObjectName(QStringLiteral("workflow_dock"));
  workflow_dock_->setWidget(workflow_panel_);
  workflow_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  addDockWidget(Qt::RightDockWidgetArea, workflow_dock_);
  workflow_dock_->setMinimumWidth(360);
  auto *toggle = workflow_dock_->toggleViewAction();
  toggle->setText(QStringLiteral("Publish Workflow Panel"));
  toggle->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_W));
  if (view_menu_) {
    view_menu_->addSeparator();
    view_menu_->addAction(toggle);
  }

  connect(workflow_panel_, &WorkflowPanel::refine_requested, this, &MainWindow::run_refine);
  connect(workflow_panel_, &WorkflowPanel::relocalization_requested, this, &MainWindow::run_relocalization);
  connect(workflow_panel_, &WorkflowPanel::navigation_requested, this, &MainWindow::run_navigation);
  connect(workflow_panel_, &WorkflowPanel::patch_requested, this, &MainWindow::run_patch);
  connect(workflow_panel_, &WorkflowPanel::publish_requested, this, &MainWindow::run_publish);
  connect(workflow_panel_, &WorkflowPanel::run_all_requested, this, &MainWindow::run_all_pending);
  connect(workflow_panel_, &WorkflowPanel::cancel_requested, this, &MainWindow::cancel_tool);
  connect(workflow_panel_, &WorkflowPanel::open_output_requested, this, [](const QString &path) {
    QDesktopServices::openUrl(QUrl::fromLocalFile(path));
  });
  connect(workflow_panel_, &WorkflowPanel::parameters_changed, this, [this]() {
    workflow_panel_->read_converter(&session_.converter());
    workflow_panel_->read_publish_target(&session_.publish_target());
    refresh_workflow();
  });
}

void MainWindow::create_confidence_dock() {
  confidence_dock_ = new QDockWidget(QStringLiteral("Spatial Confidence Evidence"), this);
  confidence_dock_->setObjectName(QStringLiteral("spatial_confidence_dock"));
  confidence_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  auto *scroll = new QScrollArea(confidence_dock_);
  scroll->setWidgetResizable(true);
  auto *panel = new QWidget(scroll);
  auto *layout = new QVBoxLayout(panel);
  confidence_status_label_ = new QLabel(panel);
  confidence_status_label_->setText(QStringLiteral(
      "Single-session observation evidence only. NOT long-term stability "
      "probability. Geometry score is deferred; LOW_GEOMETRY is a manual "
      "reason, not a semantic classifier. No online update or navigation use."));
  confidence_status_label_->setWordWrap(true);
  confidence_status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(confidence_status_label_);
  geometry_summary_label_ = new QLabel(QStringLiteral(
      "Geometry sidecar: not loaded. Optional read-only directional evidence; "
      "V1 geometry_score remains 1 (deferred)."), panel);
  geometry_summary_label_->setObjectName(QStringLiteral("geometry_readonly_status"));
  geometry_summary_label_->setWordWrap(true);
  geometry_summary_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(geometry_summary_label_);
  confidence_edit_state_label_ = new QLabel(QStringLiteral("No confidence derivative loaded"), panel);
  confidence_edit_state_label_->setWordWrap(true);
  layout->addWidget(confidence_edit_state_label_);
  auto *edit_form = new QFormLayout();
  confidence_override_mode_combo_ = new QComboBox(panel);
  for (const auto mode : {agt_spatial_map_core::ManualOverrideMode::AUTO,
                          agt_spatial_map_core::ManualOverrideMode::FORCE_HIGH,
                          agt_spatial_map_core::ManualOverrideMode::FORCE_LOW,
                          agt_spatial_map_core::ManualOverrideMode::IGNORE}) {
    confidence_override_mode_combo_->addItem(
        QString::fromLatin1(agt_spatial_map_core::override_mode_name(mode)),
        static_cast<int>(mode));
  }
  edit_form->addRow(QStringLiteral("Intent"), confidence_override_mode_combo_);
  confidence_reason_combo_ = new QComboBox(panel);
  for (const char *tag : {"PARKING_AREA", "VEGETATION", "TEMPORARY_OBJECT",
                          "CONSTRUCTION", "MOVING_OBJECT_PRONE", "LOW_GEOMETRY",
                          "MANUAL_ANCHOR", "OTHER"}) {
    confidence_reason_combo_->addItem(QString::fromLatin1(tag));
  }
  confidence_reason_combo_->setToolTip(QStringLiteral(
      "Reason records human judgment; LOW_GEOMETRY does not change geometry_score"));
  edit_form->addRow(QStringLiteral("Reason"), confidence_reason_combo_);
  confidence_custom_low_check_ = new QCheckBox(QStringLiteral("Custom FORCE_LOW value"), panel);
  confidence_low_value_spin_ = new QDoubleSpinBox(panel);
  confidence_low_value_spin_->setRange(0.0, 1.0);
  confidence_low_value_spin_->setDecimals(4);
  confidence_low_value_spin_->setSingleStep(0.01);
  confidence_low_value_spin_->setValue(0.05);
  confidence_low_value_spin_->setEnabled(false);
  edit_form->addRow(confidence_custom_low_check_);
  edit_form->addRow(QStringLiteral("Value [0,1]"), confidence_low_value_spin_);
  layout->addLayout(edit_form);
  auto *edit_buttons = new QHBoxLayout();
  confidence_apply_button_ = new QPushButton(QStringLiteral("Apply to selected"), panel);
  confidence_restore_button_ = new QPushButton(QStringLiteral("Restore Auto"), panel);
  edit_buttons->addWidget(confidence_apply_button_);
  edit_buttons->addWidget(confidence_restore_button_);
  layout->addLayout(edit_buttons);
  auto *history_buttons = new QHBoxLayout();
  confidence_undo_button_ = new QPushButton(QStringLiteral("Undo override"), panel);
  confidence_redo_button_ = new QPushButton(QStringLiteral("Redo override"), panel);
  history_buttons->addWidget(confidence_undo_button_);
  history_buttons->addWidget(confidence_redo_button_);
  layout->addLayout(history_buttons);
  confidence_save_button_ = new QPushButton(QStringLiteral("Save Overrides YAML (intent only)"), panel);
  confidence_save_button_->setObjectName(QStringLiteral("confidence_save_intent"));
  confidence_rebuild_button_ = new QPushButton(
      QStringLiteral("Core rebuild reviewed derivative (new directory)"), panel);
  confidence_rebuild_button_->setObjectName(QStringLiteral("confidence_core_rebuild"));
  confidence_rebuild_button_->setToolTip(QStringLiteral(
      "Separate from Save Overrides; copies verified auto evidence, applies saved intent "
      "in the core, creates new checksum-covered artifacts. Never publishes a navigation map."));
  layout->addWidget(confidence_save_button_);
  layout->addWidget(confidence_rebuild_button_);
  connect(confidence_save_button_, &QPushButton::clicked,
          this, &MainWindow::save_confidence_overrides_dialog);
  connect(confidence_rebuild_button_, &QPushButton::clicked,
          this, &MainWindow::rebuild_confidence_review_dialog);
  connect(confidence_apply_button_, &QPushButton::clicked,
          this, &MainWindow::apply_confidence_override);
  connect(confidence_restore_button_, &QPushButton::clicked,
          this, &MainWindow::restore_confidence_auto);
  connect(confidence_undo_button_, &QPushButton::clicked,
          this, &MainWindow::undo_confidence_override);
  connect(confidence_redo_button_, &QPushButton::clicked,
          this, &MainWindow::redo_confidence_override);
  const auto update_low_value = [this]() {
    const bool low = confidence_override_mode_combo_->currentData().toInt() ==
        static_cast<int>(agt_spatial_map_core::ManualOverrideMode::FORCE_LOW);
    confidence_custom_low_check_->setEnabled(low);
    confidence_low_value_spin_->setEnabled(low && confidence_custom_low_check_->isChecked());
  };
  connect(confidence_override_mode_combo_, qOverload<int>(&QComboBox::currentIndexChanged),
          this, [update_low_value](int) { update_low_value(); });
  connect(confidence_custom_low_check_, &QCheckBox::toggled, this,
          [update_low_value](bool) { update_low_value(); });
  update_low_value();
  confidence_details_label_ = new QLabel(panel);
  confidence_details_label_->setWordWrap(true);
  confidence_details_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(confidence_details_label_);
  layout->addStretch();
  scroll->setWidget(panel);
  confidence_dock_->setWidget(scroll);
  addDockWidget(Qt::RightDockWidgetArea, confidence_dock_);
  // The full-height editor shares the dock area with the legacy publish workflow.
  // Stacking them vertically hid Apply/Undo/Save below the viewport on 900px
  // screens; a tab keeps both workflows reachable without publishing a map.
  tabifyDockWidget(workflow_dock_, confidence_dock_);
  confidence_dock_->setMinimumWidth(360);
  confidence_dock_->hide();
  if (view_menu_) {
    auto *toggle = confidence_dock_->toggleViewAction();
    toggle->setText(QStringLiteral("Spatial Confidence Evidence Panel"));
    view_menu_->addAction(toggle);
  }
}

void MainWindow::create_relocalization_dock() {
  relocalization_dock_ = new QDockWidget(QStringLiteral("Relocalization MVP"), this);
  relocalization_dock_->setObjectName(QStringLiteral("relocalization_mvp_dock"));
  relocalization_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  auto *scroll = new QScrollArea(relocalization_dock_);
  scroll->setWidgetResizable(true);
  auto *panel = new QWidget(scroll);
  auto *layout = new QVBoxLayout(panel);
  auto *intro = new QLabel(QStringLiteral(
      "Offline inspection only. Source geometry remains read-only. Block size, query frames, "
      "and candidate Top-K are independent parameters. Block PCDs support visualization and "
      "leakage exclusion; existing per-keyframe GLOBAL/Top-K/GICP remains the analysis engine."), panel);
  intro->setWordWrap(true);
  layout->addWidget(intro);

  relocalization_status_label_ = new QLabel(QStringLiteral("Open a validated mapping source package to begin."), panel);
  relocalization_status_label_->setWordWrap(true);
  relocalization_status_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(relocalization_status_label_);

  auto *output_form = new QFormLayout();
  evidence_root_edit_ = new QLineEdit(
      QDir::home().filePath(QStringLiteral("ros2_ws/experiments/mapstudio_relocalization_mvp")), panel);
  output_form->addRow(QStringLiteral("New assets root"), evidence_root_edit_);
  block_keyframe_count_spin_ = new QSpinBox(panel);
  block_keyframe_count_spin_->setRange(1, 200);
  block_keyframe_count_spin_->setValue(11);
  output_form->addRow(QStringLiteral("Block keyframes"), block_keyframe_count_spin_);
  layout->addLayout(output_form);
  auto *block_buttons = new QHBoxLayout();
  auto *build_blocks_button = new QPushButton(QStringLiteral("Build new blocks"), panel);
  auto *load_blocks_button = new QPushButton(QStringLiteral("Load blocks"), panel);
  block_buttons->addWidget(build_blocks_button);
  block_buttons->addWidget(load_blocks_button);
  layout->addLayout(block_buttons);
  block_directory_edit_ = new QLineEdit(panel);
  block_directory_edit_->setReadOnly(true);
  block_directory_edit_->setPlaceholderText(QStringLiteral("Select an immutable block-set directory"));
  layout->addWidget(block_directory_edit_);

  auto *topology_form = new QFormLayout();
  topology_path_edit_ = new QLineEdit(panel);
  topology_path_edit_->setReadOnly(true);
  auto *topology_row = new QHBoxLayout();
  topology_row->addWidget(topology_path_edit_);
  auto *browse_topology = new QPushButton(QStringLiteral("Browse"), panel);
  topology_row->addWidget(browse_topology);
  topology_form->addRow(QStringLiteral("Schema v1 topology"), topology_row);
  layout->addLayout(topology_form);
  auto *structure_buttons = new QHBoxLayout();
  auto *load_structure_button = new QPushButton(QStringLiteral("Validate & show structure"), panel);
  auto *edit_structure_button = new QPushButton(QStringLiteral("Edit in new draft"), panel);
  structure_buttons->addWidget(load_structure_button);
  structure_buttons->addWidget(edit_structure_button);
  layout->addLayout(structure_buttons);

  auto *query_form = new QFormLayout();
  query_mode_combo_ = new QComboBox(panel);
  query_mode_combo_->addItem(QStringLiteral("Keyframe"), QStringLiteral("keyframe"));
  query_mode_combo_->addItem(QStringLiteral("Map point (snap to keyframe)"), QStringLiteral("map_point"));
  query_mode_combo_->addItem(QStringLiteral("Reviewed row position"), QStringLiteral("row_position"));
  query_form->addRow(QStringLiteral("Query selection"), query_mode_combo_);
  query_keyframe_spin_ = new QSpinBox(panel);
  query_keyframe_spin_->setRange(0, 2000000);
  query_form->addRow(QStringLiteral("Keyframe ID"), query_keyframe_spin_);
  auto *click_row = new QHBoxLayout();
  query_x_edit_ = new QLineEdit(panel); query_x_edit_->setReadOnly(true); query_x_edit_->setPlaceholderText(QStringLiteral("map x"));
  query_y_edit_ = new QLineEdit(panel); query_y_edit_->setReadOnly(true); query_y_edit_->setPlaceholderText(QStringLiteral("map y"));
  auto *pick_point_button = new QPushButton(QStringLiteral("Pick on 3D map"), panel);
  click_row->addWidget(query_x_edit_); click_row->addWidget(query_y_edit_); click_row->addWidget(pick_point_button);
  query_form->addRow(QStringLiteral("Map point"), click_row);
  row_id_edit_ = new QLineEdit(panel);
  row_id_edit_->setPlaceholderText(QStringLiteral("confirmed physical row ID"));
  query_form->addRow(QStringLiteral("Row ID"), row_id_edit_);
  along_row_s_spin_ = new QDoubleSpinBox(panel);
  along_row_s_spin_->setRange(0.0, 100000.0);
  along_row_s_spin_->setDecimals(2);
  along_row_s_spin_->setSuffix(QStringLiteral(" m"));
  query_form->addRow(QStringLiteral("Along row s"), along_row_s_spin_);
  query_frames_combo_ = new QComboBox(panel);
  for (int frames : {1, 3, 5}) query_frames_combo_->addItem(QString::number(frames), frames);
  query_form->addRow(QStringLiteral("Query accumulation frames"), query_frames_combo_);
  candidate_top_k_spin_ = new QSpinBox(panel);
  candidate_top_k_spin_->setRange(1, 50);
  candidate_top_k_spin_->setValue(10);
  query_form->addRow(QStringLiteral("Candidate Top-K"), candidate_top_k_spin_);
  native_localizer_path_edit_ = new QLineEdit(panel);
  native_localizer_path_edit_->setReadOnly(true);
  native_localizer_path_edit_->setPlaceholderText(
      QStringLiteral("Installed default; select a trace-capable existing binary if required"));
  auto *native_localizer_row = new QHBoxLayout();
  native_localizer_row->addWidget(native_localizer_path_edit_);
  auto *browse_native_localizer = new QPushButton(QStringLiteral("Browse"), panel);
  native_localizer_row->addWidget(browse_native_localizer);
  query_form->addRow(QStringLiteral("Native localizer override"), native_localizer_row);
  layout->addLayout(query_form);
  auto *run_query_button = new QPushButton(QStringLiteral("Run offline query and save evidence"), panel);
  layout->addWidget(run_query_button);

  auto *evidence_row = new QHBoxLayout();
  evidence_path_edit_ = new QLineEdit(panel);
  evidence_path_edit_->setReadOnly(true);
  evidence_path_edit_->setPlaceholderText(QStringLiteral("Saved immutable evidence directory"));
  evidence_row->addWidget(evidence_path_edit_);
  auto *open_evidence_button = new QPushButton(QStringLiteral("Load"), panel);
  evidence_row->addWidget(open_evidence_button);
  layout->addLayout(evidence_row);

  candidate_table_ = new QTableWidget(0, 6, panel);
  candidate_table_->setHorizontalHeaderLabels({QStringLiteral("Rank"), QStringLiteral("Keyframe"),
      QStringLiteral("Block"), QStringLiteral("Descriptor"), QStringLiteral("GICP"), QStringLiteral("Row")});
  candidate_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  candidate_table_->horizontalHeader()->setStretchLastSection(true);
  candidate_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  candidate_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  candidate_table_->setMaximumHeight(190);
  layout->addWidget(candidate_table_);
  relocalization_details_label_ = new QLabel(
      QStringLiteral("Load evidence to inspect query provenance, block bindings, and candidate metrics."), panel);
  relocalization_details_label_->setWordWrap(true);
  relocalization_details_label_->setTextInteractionFlags(Qt::TextSelectableByMouse);
  layout->addWidget(relocalization_details_label_);

  auto *layer_group = new QGroupBox(QStringLiteral("Independent display layers"), panel);
  auto *layer_layout = new QVBoxLayout(layer_group);
  show_structure_layer_ = new QCheckBox(QStringLiteral("Structure / scene markers"), layer_group);
  show_blocks_layer_ = new QCheckBox(QStringLiteral("Block bounds and centers"), layer_group);
  show_query_layer_ = new QCheckBox(QStringLiteral("Query at estimated pose"), layer_group);
  show_candidate_layer_ = new QCheckBox(QStringLiteral("Selected candidate"), layer_group);
  for (auto *check : {show_structure_layer_, show_blocks_layer_, show_query_layer_, show_candidate_layer_})
    layer_layout->addWidget(check);
  layout->addWidget(layer_group);
  layout->addStretch();
  scroll->setWidget(panel);
  relocalization_dock_->setWidget(scroll);
  addDockWidget(Qt::RightDockWidgetArea, relocalization_dock_);
  tabifyDockWidget(workflow_dock_, relocalization_dock_);
  relocalization_dock_->setMinimumWidth(400);
  if (view_menu_) {
    auto *toggle = relocalization_dock_->toggleViewAction();
    toggle->setText(QStringLiteral("Relocalization MVP Panel"));
    view_menu_->addAction(toggle);
  }

  connect(build_blocks_button, &QPushButton::clicked, this, &MainWindow::build_keyframe_blocks);
  connect(load_blocks_button, &QPushButton::clicked, this, &MainWindow::load_keyframe_blocks);
  connect(browse_topology, &QPushButton::clicked, this, &MainWindow::choose_topology_annotation);
  connect(browse_native_localizer, &QPushButton::clicked, this, [this]() {
    const QString path = QFileDialog::getOpenFileName(
        this, QStringLiteral("Select existing trace-capable native localizer"),
        QDir::home().filePath(QStringLiteral("ros2_ws/experiments")));
    if (!path.isEmpty()) native_localizer_path_edit_->setText(path);
  });
  connect(load_structure_button, &QPushButton::clicked, this, &MainWindow::load_structure_annotation);
  connect(edit_structure_button, &QPushButton::clicked, this, &MainWindow::start_structure_editor);
  connect(pick_point_button, &QPushButton::clicked, this, &MainWindow::pick_relocalization_point);
  connect(run_query_button, &QPushButton::clicked, this, &MainWindow::run_relocalization_query);
  connect(open_evidence_button, &QPushButton::clicked, this, &MainWindow::choose_relocalization_evidence);
  connect(candidate_table_, &QTableWidget::cellClicked, this, &MainWindow::select_candidate_overlay);
  connect(show_structure_layer_, &QCheckBox::toggled, this, [this](bool on) {
    viewer_->set_auxiliary_visible(AuxiliaryLayer::Structure, on);
  });
  connect(show_blocks_layer_, &QCheckBox::toggled, this, [this](bool on) {
    viewer_->set_auxiliary_visible(AuxiliaryLayer::Blocks, on);
  });
  connect(show_query_layer_, &QCheckBox::toggled, this, [this](bool on) {
    viewer_->set_auxiliary_visible(AuxiliaryLayer::Query, on);
  });
  connect(show_candidate_layer_, &QCheckBox::toggled, this, [this](bool on) {
    viewer_->set_auxiliary_visible(AuxiliaryLayer::Candidate, on);
  });
}

void MainWindow::start_relocalization_tool(const ToolInvocation &invocation,
                                            std::function<void(const ToolResult &)> on_done) {
  if (tool_runner_.is_running() || confidence_review_runner_.is_running()) {
    QMessageBox::information(this, QStringLiteral("Busy"),
                             QStringLiteral("Another Studio tool is still running."));
    return;
  }
  tool_callback_ = std::move(on_done);
  if (relocalization_status_label_)
    relocalization_status_label_->setText(QStringLiteral("RUNNING — %1").arg(invocation.label));
  tool_runner_.start(invocation);  // no implicit workflow log write into the source package area
}

void MainWindow::build_keyframe_blocks() {
  if (!session_.source_is_mapping_package()) {
    QMessageBox::warning(this, QStringLiteral("Mapping source required"),
                         QStringLiteral("Open a validated mapping source package first."));
    return;
  }
  QString missing;
  if (!tools_available({QStringLiteral("agt_mapping_artifacts/agt_build_keyframe_blocks")}, &missing)) {
    QMessageBox::critical(this, QStringLiteral("Block producer unavailable"), missing);
    return;
  }
  QString output_error;
  if (!validate_relocalization_output_root(&output_error)) {
    QMessageBox::warning(this, QStringLiteral("Unsafe output root"), output_error);
    return;
  }
  const QString root = QDir::cleanPath(evidence_root_edit_->text().trimmed());
  if (root.isEmpty()) return;
  const QString output = QDir(root).filePath(
      QStringLiteral("blocks_k%1_%2_%3")
          .arg(block_keyframe_count_spin_->value()).arg(stamp())
          .arg(QUuid::createUuid().toString(QUuid::WithoutBraces).left(8)));
  QStringList arguments{QStringLiteral("--source-package"), session_.source_package_dir(),
                        QStringLiteral("--output"), output,
                        QStringLiteral("--block-keyframe-count"),
                        QString::number(block_keyframe_count_spin_->value())};
  if (!topology_path_edit_->text().isEmpty())
    arguments << QStringLiteral("--topology") << topology_path_edit_->text();
  const auto invocation = ExternalToolRunner::ros2_run(
      QStringLiteral("Build immutable keyframe blocks"), QStringLiteral("agt_mapping_artifacts"),
      QStringLiteral("agt_build_keyframe_blocks"), arguments);
  start_relocalization_tool(invocation, [this, output](const ToolResult &result) {
    if (!result.ok) {
      relocalization_status_label_->setText(
          QStringLiteral("Block build failed: %1").arg(result.error_summary));
      return;
    }
    block_directory_edit_->setText(output);
    display_block_preview(output);
  });
}

void MainWindow::display_block_preview(const QString &directory) {
  QString output_error;
  if (!validate_relocalization_output_root(&output_error)) {
    relocalization_status_label_->setText(QStringLiteral("Unsafe preview output: %1").arg(output_error));
    return;
  }
  QString missing;
  if (!tools_available({QStringLiteral("agt_mapping_artifacts/agt_export_keyframe_block_preview")}, &missing)) {
    relocalization_status_label_->setText(QStringLiteral("Block assets verified by producer; preview tool unavailable: %1").arg(missing));
    return;
  }
  const QString preview_dir = QDir(evidence_root_edit_->text()).filePath(
      QStringLiteral("_studio_preview/%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));
  const QString output = QDir(preview_dir).filePath(QStringLiteral("block_bounds_display_only.pcd"));
  const auto invocation = ExternalToolRunner::ros2_run(
      QStringLiteral("Verify and preview block bounds"), QStringLiteral("agt_mapping_artifacts"),
      QStringLiteral("agt_export_keyframe_block_preview"),
      {QStringLiteral("--block-dir"), directory,
       QStringLiteral("--source-package"), session_.source_package_dir(),
       QStringLiteral("--output-pcd"), output});
  start_relocalization_tool(invocation, [this, output, directory](const ToolResult &result) {
    if (!result.ok) {
      relocalization_status_label_->setText(QStringLiteral("Block preview failed: %1").arg(result.error_summary));
      return;
    }
    LoadedPointCloud cloud;
    std::string error;
    if (!PCDLoader::load(output.toStdString(), &cloud, &error)) {
      relocalization_status_label_->setText(QStringLiteral("Could not load block preview: %1").arg(QString::fromStdString(error)));
      return;
    }
    block_directory_edit_->setText(directory);
    viewer_->set_auxiliary_cloud(AuxiliaryLayer::Blocks, cloud, show_blocks_layer_->isChecked());
    relocalization_status_label_->setText(QStringLiteral("Block set verified and displayed. Bounds are sparse display-only points."));
  });
}

void MainWindow::load_keyframe_blocks() {
  const QString directory = QFileDialog::getExistingDirectory(
      this, QStringLiteral("Load immutable block set"), evidence_root_edit_->text());
  if (directory.isEmpty()) return;
  block_directory_edit_->setText(directory);
  display_block_preview(directory);
}

void MainWindow::choose_topology_annotation() {
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("Open schema-v1 greenhouse topology"),
      session_.source_package_dir(), QStringLiteral("Topology YAML (*.yaml *.yml)"));
  if (!path.isEmpty()) topology_path_edit_->setText(path);
}

void MainWindow::load_structure_annotation() {
  if (!session_.source_is_mapping_package() || topology_path_edit_->text().isEmpty()) {
    QMessageBox::warning(this, QStringLiteral("Source and topology required"),
                         QStringLiteral("Open a mapping source and select its topology YAML."));
    return;
  }
  QString output_error;
  if (!validate_relocalization_output_root(&output_error)) {
    QMessageBox::warning(this, QStringLiteral("Unsafe structure preview output"), output_error);
    return;
  }
  QString missing;
  if (!tools_available({QStringLiteral("agt_mapping_artifacts/agt_export_structure_overlay")}, &missing)) {
    QMessageBox::critical(this, QStringLiteral("Structure loader unavailable"), missing);
    return;
  }
  const QString scratch = QDir(evidence_root_edit_->text()).filePath(
      QStringLiteral("_studio_preview/structure_%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));
  const QString overlay = QDir(scratch).filePath(QStringLiteral("structure_display_only.pcd"));
  const QString metadata = QDir(scratch).filePath(QStringLiteral("structure_overlay.json"));
  const auto invocation = ExternalToolRunner::ros2_run(
      QStringLiteral("Validate and display row/headland annotation"), QStringLiteral("agt_mapping_artifacts"),
      QStringLiteral("agt_export_structure_overlay"),
      {QStringLiteral("--source-package"), session_.source_package_dir(),
       QStringLiteral("--topology"), topology_path_edit_->text(),
       QStringLiteral("--output-pcd"), overlay, QStringLiteral("--output-json"), metadata});
  start_relocalization_tool(invocation, [this, overlay, metadata](const ToolResult &result) {
    if (!result.ok) {
      relocalization_status_label_->setText(QStringLiteral("Structure validation failed: %1").arg(result.error_summary));
      return;
    }
    try {
      const YAML::Node info = YAML::LoadFile(metadata.toStdString());
      const auto rows = info["rows"] ? info["rows"].size() : 0U;
      const auto heads = info["headlands"] ? info["headlands"].size() : 0U;
      const auto scenes = info["scenes"] ? info["scenes"].size() : 0U;
      const bool confirmed = info["manual_review_confirmed"] && info["manual_review_confirmed"].as<bool>();
      if (QFileInfo::exists(overlay)) {
        LoadedPointCloud cloud;
        std::string error;
        if (!PCDLoader::load(overlay.toStdString(), &cloud, &error))
          throw std::runtime_error(error);
        viewer_->set_auxiliary_cloud(AuxiliaryLayer::Structure, cloud, show_structure_layer_->isChecked());
      } else {
        viewer_->clear_auxiliary_cloud(AuxiliaryLayer::Structure);
      }
      show_structure_layer_->setEnabled(rows + heads + scenes > 0);
      relocalization_status_label_->setText(
          QStringLiteral("Structure validated against this exact source: %1 row(s), %2 headland(s), %3 manual scene(s); %4. Unconfirmed physical IDs stay UNKNOWN.")
              .arg(static_cast<qulonglong>(rows)).arg(static_cast<qulonglong>(heads))
              .arg(static_cast<qulonglong>(scenes))
              .arg(confirmed ? QStringLiteral("FROZEN / MANUALLY REVIEWED") : QStringLiteral("DRAFT / NOT REVIEWED")));
    } catch (const std::exception &error) {
      relocalization_status_label_->setText(QStringLiteral("Structure overlay could not be loaded: %1").arg(error.what()));
    }
  });
}

void MainWindow::start_structure_editor() {
  if (!session_.source_is_mapping_package()) {
    QMessageBox::warning(this, QStringLiteral("Mapping source required"),
                         QStringLiteral("Open a mapping source package before creating a topology draft."));
    return;
  }
  QString output_error;
  if (!validate_relocalization_output_root(&output_error)) {
    QMessageBox::warning(this, QStringLiteral("Unsafe annotation draft output"), output_error);
    return;
  }
  if (!ExternalToolRunner::program_available(QStringLiteral("ros2"))) {
    QMessageBox::critical(this, QStringLiteral("ROS 2 unavailable"), QStringLiteral("Source ROS 2 Humble and the workspace overlay first."));
    return;
  }
  const QString revision = QStringLiteral("%1_%2").arg(stamp())
      .arg(QUuid::createUuid().toString(QUuid::WithoutBraces).left(8));
  const QString draft_dir = QDir(evidence_root_edit_->text()).filePath(
      QStringLiteral("annotation_drafts/%1").arg(revision));
  if (!QDir().mkpath(draft_dir)) {
    QMessageBox::critical(this, QStringLiteral("Cannot create draft directory"), draft_dir);
    return;
  }
  const QString draft = QDir(draft_dir).filePath(QStringLiteral("greenhouse_topology.yaml"));
  if (QFileInfo::exists(draft)) {
    QMessageBox::critical(this, QStringLiteral("Draft revision already exists"), draft);
    return;
  }
  bool opened_frozen_as_draft = false;
  if (!topology_path_edit_->text().isEmpty() && QFileInfo::exists(topology_path_edit_->text())) {
    try {
      const QString selected = topology_path_edit_->text();
      YAML::Node draft_topology = YAML::LoadFile(selected.toStdString());
      const QString status = QString::fromStdString(
          draft_topology["annotation"]["status"].as<std::string>("UNKNOWN"));
      if (status == QStringLiteral("frozen")) {
        QFile frozen_file(selected);
        if (!frozen_file.open(QIODevice::ReadOnly))
          throw std::runtime_error("could not read the selected frozen annotation");
        const QByteArray source_hash = QCryptographicHash::hash(
            frozen_file.readAll(), QCryptographicHash::Sha256).toHex();
        draft_topology["annotation"]["status"] = "draft";
        draft_topology["annotation"]["manual_review_confirmed"] = false;
        draft_topology["annotation"].remove("frozen_at");
        draft_topology["annotation"]["derived_from_file"] = selected.toStdString();
        draft_topology["annotation"]["derived_from_sha256"] = source_hash.toStdString();
        opened_frozen_as_draft = true;
      }
      YAML::Emitter emitter;
      emitter << draft_topology;
      std::ofstream output(draft.toStdString(), std::ios::binary | std::ios::out | std::ios::trunc);
      if (!output || !emitter.good())
        throw std::runtime_error("could not serialize the separate topology draft");
      output << emitter.c_str() << '\n';
      output.close();
      if (!output) throw std::runtime_error("could not finish writing the separate topology draft");
    } catch (const std::exception &error) {
      QMessageBox::critical(this, QStringLiteral("Cannot create topology draft"),
                            QString::fromUtf8(error.what()));
      return;
    }
  }
  const QStringList args{QStringLiteral("run"), QStringLiteral("agt_greenhouse_annotation"),
      QStringLiteral("greenhouse_annotator"), QStringLiteral("--map-package"),
      session_.source_package_dir(), QStringLiteral("--output"), draft};
  if (!QProcess::startDetached(QStringLiteral("ros2"), args)) {
    QMessageBox::critical(this, QStringLiteral("Annotator did not start"),
                          QStringLiteral("Could not start the existing greenhouse annotation tool."));
    return;
  }
  topology_path_edit_->setText(draft);
  relocalization_status_label_->setText(opened_frozen_as_draft
      ? QStringLiteral("Existing annotation tool opened a new editable revision derived from the frozen annotation; manual review was reset. Save/review there, then validate and reload this path.")
      : QStringLiteral("Existing annotation tool opened with a separate working revision. Save/review there, then validate and reload this path."));
}

void MainWindow::pick_relocalization_point() {
  if (!viewer_->has_cloud()) return;
  query_mode_combo_->setCurrentIndex(1);
  viewer_->set_mode(InteractionMode::Navigate);
  viewer_->set_query_pick_mode(true);
  relocalization_status_label_->setText(QStringLiteral("Click once on the 3D map. The click selects the nearest source keyframe; it is not a sensor observation."));
  viewer_->setFocus();
}

void MainWindow::run_relocalization_query() {
  if (!session_.source_is_mapping_package() || block_directory_edit_->text().isEmpty()) {
    QMessageBox::warning(this, QStringLiteral("Source and blocks required"),
                         QStringLiteral("Open a mapping source and load/build a verified block set first."));
    return;
  }
  QString output_error;
  if (!validate_relocalization_output_root(&output_error)) {
    QMessageBox::warning(this, QStringLiteral("Unsafe evidence output"), output_error);
    return;
  }
  QString missing;
  if (!tools_available({QStringLiteral("agt_map_localization_benchmark/agt_run_relocalization_query")}, &missing)) {
    QMessageBox::critical(this, QStringLiteral("Offline query unavailable"), missing);
    return;
  }
  QStringList arguments{QStringLiteral("--map-package"), session_.source_package_dir(),
      QStringLiteral("--block-dir"), block_directory_edit_->text(),
      QStringLiteral("--output-root"), evidence_root_edit_->text(),
      QStringLiteral("--query-accumulation-frames"), query_frames_combo_->currentData().toString(),
      QStringLiteral("--candidate-top-k"), QString::number(candidate_top_k_spin_->value())};
  if (!topology_path_edit_->text().trimmed().isEmpty())
    arguments << QStringLiteral("--topology") << topology_path_edit_->text().trimmed();
  if (!native_localizer_path_edit_->text().trimmed().isEmpty())
    arguments << QStringLiteral("--native-localizer-path")
              << native_localizer_path_edit_->text().trimmed();
  const QString mode = query_mode_combo_->currentData().toString();
  if (mode == QStringLiteral("keyframe")) {
    arguments << QStringLiteral("--query-keyframe") << QString::number(query_keyframe_spin_->value());
  } else if (mode == QStringLiteral("map_point")) {
    if (!has_picked_query_point_) {
      QMessageBox::information(this, QStringLiteral("Pick a map point"),
                               QStringLiteral("Use Pick on 3D map and click near the desired location first."));
      return;
    }
    arguments << QStringLiteral("--map-x") << QString::number(picked_query_x_, 'g', 12)
              << QStringLiteral("--query-y") << QString::number(picked_query_y_, 'g', 12);
  } else {
    if (topology_path_edit_->text().isEmpty() || row_id_edit_->text().trimmed().isEmpty()) {
      QMessageBox::warning(this, QStringLiteral("Reviewed row required"),
                           QStringLiteral("Select a frozen topology and a physical row ID."));
      return;
    }
    arguments << QStringLiteral("--row-position") << QStringLiteral("--row-id")
              << row_id_edit_->text().trimmed() << QStringLiteral("--along-row-s-m")
              << QString::number(along_row_s_spin_->value(), 'g', 12);
  }
  const auto invocation = ExternalToolRunner::ros2_run(
      QStringLiteral("Run sparse offline GLOBAL/Top-K/GICP query"),
      QStringLiteral("agt_map_localization_benchmark"),
      QStringLiteral("agt_run_relocalization_query"), arguments);
  start_relocalization_tool(invocation, [this](const ToolResult &result) {
    if (!result.ok) {
      relocalization_status_label_->setText(QStringLiteral("Query job failed: %1").arg(result.error_summary));
      return;
    }
    const QRegularExpression expression(QStringLiteral("\\\"evidence_path\\\"\\s*:\\s*\\\"([^\\\"]+)\\\""));
    const auto match = expression.match(result.output);
    if (match.hasMatch()) {
      evidence_path_edit_->setText(match.captured(1));
      load_relocalization_evidence();
    } else {
      relocalization_status_label_->setText(QStringLiteral("Query completed, but its evidence path was not found in tool output."));
    }
  });
}

void MainWindow::choose_relocalization_evidence() {
  const QString path = QFileDialog::getExistingDirectory(
      this, QStringLiteral("Load saved Relocalization Evidence"), evidence_root_edit_->text());
  if (path.isEmpty()) return;
  evidence_path_edit_->setText(path);
  load_relocalization_evidence();
}

void MainWindow::load_relocalization_evidence() {
  const QString evidence = evidence_path_edit_->text();
  if (evidence.isEmpty() || !QFileInfo::exists(evidence)) return;
  if (!session_.source_is_mapping_package()) {
    QMessageBox::warning(this, QStringLiteral("Source required for staleness check"),
                         QStringLiteral("Open the exact mapping source before loading evidence."));
    return;
  }
  if (block_directory_edit_->text().isEmpty()) {
    QMessageBox::warning(this, QStringLiteral("Block set required for staleness check"),
                         QStringLiteral("Load the exact immutable block set before loading evidence."));
    return;
  }
  QString missing;
  if (!tools_available({QStringLiteral("agt_map_localization_benchmark/agt_verify_relocalization_evidence")}, &missing)) {
    QMessageBox::critical(this, QStringLiteral("Evidence verifier unavailable"), missing);
    return;
  }
  QStringList args{evidence, QStringLiteral("--map-package"), session_.source_package_dir()};
  if (!block_directory_edit_->text().isEmpty())
    args << QStringLiteral("--block-dir") << block_directory_edit_->text();
  if (!topology_path_edit_->text().isEmpty())
    args << QStringLiteral("--topology") << topology_path_edit_->text();
  const auto invocation = ExternalToolRunner::ros2_run(
      QStringLiteral("Verify evidence and current source/block identity"),
      QStringLiteral("agt_map_localization_benchmark"),
      QStringLiteral("agt_verify_relocalization_evidence"), args);
  start_relocalization_tool(invocation, [this, evidence](const ToolResult &result) {
    if (!result.ok) {
      relocalization_status_label_->setText(QStringLiteral("Evidence invalid: %1").arg(result.error_summary));
      return;
    }
    try {
      const YAML::Node manifest = YAML::LoadFile(QDir(evidence).filePath(QStringLiteral("manifest.yaml")).toStdString());
      const YAML::Node query = manifest["query"];
      const YAML::Node evidence_node = manifest["evidence"];
      const YAML::Node candidates = evidence_node["candidate_ambiguity"]["candidates"];
      candidate_table_->setRowCount(0);
      for (const auto &candidate : candidates) {
        const int row = candidate_table_->rowCount();
        candidate_table_->insertRow(row);
        const YAML::Node blocks = candidate["candidate_block_ids"];
        QStringList block_ids;
        for (const auto &id : blocks) block_ids << QString::fromStdString(id.as<std::string>());
        const auto descriptor = candidate["descriptor_sector_similarity"];
        const auto attempted = candidate["gicp_attempted"];
        const auto converged = candidate["gicp_converged"];
        const QString values[] = {
          QString::number(candidate["rank"].as<int>()),
          QString::number(candidate["candidate_keyframe"].as<int>()),
          block_ids.join(QStringLiteral(",")),
          yaml_number_or_unknown(descriptor, 3),
          gicp_state(attempted, converged),
          QString::fromStdString(candidate["row_id"].as<std::string>())};
        for (int column = 0; column < 6; ++column)
          candidate_table_->setItem(row, column, new QTableWidgetItem(values[column]));
      }
      loaded_evidence_directory_ = evidence;
      const QString query_overlay_path = QDir(evidence).filePath(
          QStringLiteral("analysis/visualization/query_at_estimated_pose_display_only.pcd"));
      if (QFileInfo::exists(query_overlay_path)) {
        LoadedPointCloud cloud;
        std::string error;
        if (PCDLoader::load(query_overlay_path.toStdString(), &cloud, &error))
          viewer_->set_auxiliary_cloud(AuxiliaryLayer::Query, cloud, show_query_layer_->isChecked());
      } else {
        viewer_->clear_auxiliary_cloud(AuxiliaryLayer::Query);
      }
      viewer_->clear_auxiliary_cloud(AuxiliaryLayer::Candidate);
      show_query_layer_->setEnabled(QFileInfo::exists(query_overlay_path));
      show_candidate_layer_->setEnabled(candidates.size() > 0);
      QStringList details;
      const auto source = manifest["source"];
      const auto block_set = manifest["block_set"];
      const auto algorithm = manifest["algorithm"];
      const auto pose = query["map_pose"];
      details << QStringLiteral("Query: %1 | KF %2 | t=%3 | x/y/yaw=%4 / %5 / %6 deg | row=%7 | s=%8 | frames=%9")
          .arg(QString::fromStdString(source["identity"].as<std::string>()))
          .arg(query["keyframe"].as<int>())
          .arg(query["timestamp"].as<double>(), 0, 'f', 6)
          .arg(yaml_number_or_unknown(pose["x_m"], 3))
          .arg(yaml_number_or_unknown(pose["y_m"], 3))
          .arg(yaml_number_or_unknown(pose["yaw_deg"], 2))
          .arg(QString::fromStdString(query["row_id"].as<std::string>()))
          .arg(query["along_row_s_m"] && !query["along_row_s_m"].IsNull()
                   ? QString::number(query["along_row_s_m"].as<double>(), 'f', 2)
                   : QStringLiteral("UNKNOWN"))
          .arg(query["query_accumulation_frames"].as<int>());
      details << QStringLiteral("Block set: %1 r%2 | block size=%3 | query-excluded blocks=%4 | frames=%5")
          .arg(QString::fromStdString(block_set["block_set_id"].as<std::string>()))
          .arg(block_set["revision"].as<int>())
          .arg(block_set["block_keyframe_count"].as<int>())
          .arg(static_cast<qulonglong>(block_set["query_excluded_block_ids"]
                                           ? block_set["query_excluded_block_ids"].size() : 0U))
          .arg(static_cast<qulonglong>(block_set["query_excluded_keyframes"]
                                           ? block_set["query_excluded_keyframes"].size() : 0U));
      const auto query_blocks = block_set["query_block_details"];
      for (const auto &block : query_blocks) {
        const auto bounds = block["bbox"];
        details << QStringLiteral("Excluded %1 r%2 center KF %3; frames [%4]; bounds %5; cloud SHA-256 %6")
            .arg(QString::fromStdString(block["block_id"].as<std::string>()))
            .arg(block["revision"].as<int>()).arg(block["center_keyframe_id"].as<int>())
            .arg(QString::fromStdString(YAML::Dump(block["ordered_patch_ids"])))
            .arg(QString::fromStdString(YAML::Dump(bounds)))
            .arg(QString::fromStdString(block["output_content_sha256"].as<std::string>()));
      }
      details << QStringLiteral("Analysis: Top-K=%1 | config SHA-256=%2 | native candidate binary SHA-256=%3")
          .arg(algorithm["candidate_top_k"].as<int>())
          .arg(QString::fromStdString(algorithm["config_digest"].as<std::string>()))
          .arg(QString::fromStdString(algorithm["binary_sha256"]["candidate_bbs_gicp_localizer"].as<std::string>()));
      details << QStringLiteral("Reference: %1 (same-session=%2, independent ground truth=%3) | raw trace: %4")
          .arg(QString::fromStdString(manifest["reference"]["type"].as<std::string>()))
          .arg(manifest["reference"]["same_session"].as<bool>() ? QStringLiteral("yes") : QStringLiteral("no"))
          .arg(manifest["reference"]["independent_ground_truth"].as<bool>() ? QStringLiteral("yes") : QStringLiteral("no"))
          .arg(QString::fromStdString(manifest["analysis_artifacts"]["raw_trace"].as<std::string>()));
      relocalization_details_label_->setText(details.join(QLatin1Char('\n')));
      const QString revision_state = result.output.contains(QStringLiteral("\"revision_state\": \"STALE\""))
          ? QStringLiteral("STALE")
          : (result.output.contains(QStringLiteral("\"revision_state\": \"CURRENT\""))
             ? QStringLiteral("CURRENT") : QStringLiteral("BLOCK NOT CHECKED"));
      relocalization_status_label_->setText(
          QStringLiteral("Evidence integrity PASS; revision %1. Query KF %2, %3 accumulated frame(s), row %4; reference is %5, not independent ground truth. Select a candidate row to overlay it.")
              .arg(revision_state).arg(query["keyframe"].as<int>())
              .arg(query["query_accumulation_frames"].as<int>())
              .arg(QString::fromStdString(query["row_id"].as<std::string>()))
              .arg(QString::fromStdString(manifest["reference"]["type"].as<std::string>())));
    } catch (const std::exception &error) {
      relocalization_status_label_->setText(QStringLiteral("Evidence manifest could not be loaded: %1").arg(error.what()));
    }
  });
}

void MainWindow::select_candidate_overlay(int row, int) {
  if (loaded_evidence_directory_.isEmpty() || row < 0 || !candidate_table_->item(row, 0)) return;
  const int rank = candidate_table_->item(row, 0)->text().toInt();
  QString output_error;
  if (!validate_relocalization_output_root(&output_error)) {
    relocalization_status_label_->setText(QStringLiteral("Unsafe candidate preview output: %1").arg(output_error));
    return;
  }
  try {
    const YAML::Node manifest = YAML::LoadFile(
        QDir(loaded_evidence_directory_).filePath(QStringLiteral("manifest.yaml")).toStdString());
    YAML::Node candidate;
    const YAML::Node candidates = manifest["evidence"]["candidate_ambiguity"]["candidates"];
    for (const auto &item : candidates) {
      if (item["rank"].as<int>() == rank) {
        candidate = static_cast<const YAML::Node &>(item);
        break;
      }
    }
    if (!candidate) throw std::runtime_error("selected rank is absent from the evidence manifest");
    QStringList details;
    details << QStringLiteral("Selected candidate rank %1: KF %2 patch %3 | block(s) %4 | row %5 | s=%6")
        .arg(rank).arg(candidate["candidate_keyframe"].as<int>())
        .arg(QString::fromStdString(candidate["candidate_patch_id"].as<std::string>()))
        .arg(QString::fromStdString(YAML::Dump(candidate["candidate_block_ids"])))
        .arg(QString::fromStdString(candidate["row_id"].as<std::string>()))
        .arg(candidate["along_row_s_m"] && !candidate["along_row_s_m"].IsNull()
                 ? QString::number(candidate["along_row_s_m"].as<double>(), 'f', 2)
                 : QStringLiteral("UNKNOWN"));
    details << QStringLiteral("Descriptor sector similarity=%1 | ring distance=%2 | BBS score=%3 | GICP attempted=%4 converged=%5 fitness=%6")
        .arg(yaml_number_or_unknown(candidate["descriptor_sector_similarity"], 4))
        .arg(yaml_number_or_unknown(candidate["descriptor_ring_distance"], 4))
        .arg(yaml_number_or_unknown(candidate["bbs_score"], 4))
        .arg(yaml_boolean_or_unknown(candidate["gicp_attempted"], QStringLiteral("yes"), QStringLiteral("no")))
        .arg(gicp_state(candidate["gicp_attempted"], candidate["gicp_converged"]))
        .arg(yaml_number_or_unknown(candidate["gicp_fitness"], 5));
    details << QStringLiteral("Reference-relative classification=%1 | estimated XY error=%2 m | yaw error=%3 deg | BBS time=%4 ms")
        .arg(QString::fromStdString(candidate["classification"].as<std::string>()))
        .arg(candidate["classification"].as<std::string>() != "UNKNOWN" &&
                     manifest["evidence"]["empirical_global_result"]["reference_error"]["xy_m"] &&
                     !manifest["evidence"]["empirical_global_result"]["reference_error"]["xy_m"].IsNull()
                 ? QString::number(manifest["evidence"]["empirical_global_result"]["reference_error"]["xy_m"].as<double>(), 'f', 3)
                 : QStringLiteral("UNKNOWN"))
        .arg(candidate["classification"].as<std::string>() != "UNKNOWN" &&
                     manifest["evidence"]["empirical_global_result"]["reference_error"]["yaw_deg"] &&
                     !manifest["evidence"]["empirical_global_result"]["reference_error"]["yaw_deg"].IsNull()
                 ? QString::number(manifest["evidence"]["empirical_global_result"]["reference_error"]["yaw_deg"].as<double>(), 'f', 2)
                 : QStringLiteral("UNKNOWN"))
        .arg(yaml_number_or_unknown(candidate["bbs_elapsed_ms"], 1));
    relocalization_details_label_->setText(details.join(QLatin1Char('\n')));
  } catch (const std::exception &error) {
    relocalization_details_label_->setText(QStringLiteral("Candidate metadata could not be loaded: %1").arg(error.what()));
  }
  QString missing;
  if (!tools_available({QStringLiteral("agt_map_localization_benchmark/agt_export_relocalization_candidate")}, &missing)) {
    relocalization_status_label_->setText(QStringLiteral("Candidate overlay tool unavailable: %1").arg(missing));
    return;
  }
  const QString scratch = QDir(evidence_root_edit_->text()).filePath(
      QStringLiteral("_studio_preview/candidate_%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));
  const QString output = QDir(scratch).filePath(QStringLiteral("candidate_display_only.pcd"));
  QStringList arguments{QStringLiteral("--evidence-dir"), loaded_evidence_directory_,
                        QStringLiteral("--map-package"), session_.source_package_dir(),
                        QStringLiteral("--block-dir"), block_directory_edit_->text(),
                        QStringLiteral("--rank"), QString::number(rank),
                        QStringLiteral("--output-pcd"), output};
  if (!topology_path_edit_->text().isEmpty())
    arguments << QStringLiteral("--topology") << topology_path_edit_->text();
  const auto invocation = ExternalToolRunner::ros2_run(
      QStringLiteral("Load verified Top-K candidate overlay"),
      QStringLiteral("agt_map_localization_benchmark"),
      QStringLiteral("agt_export_relocalization_candidate"),
      arguments);
  start_relocalization_tool(invocation, [this, output, rank](const ToolResult &result) {
    if (!result.ok) {
      relocalization_status_label_->setText(QStringLiteral("Candidate overlay unavailable: %1").arg(result.error_summary));
      return;
    }
    LoadedPointCloud cloud;
    std::string error;
    if (!PCDLoader::load(output.toStdString(), &cloud, &error)) {
      relocalization_status_label_->setText(QStringLiteral("Candidate PCD failed to load: %1").arg(QString::fromStdString(error)));
      return;
    }
    viewer_->set_auxiliary_cloud(AuxiliaryLayer::Candidate, cloud, show_candidate_layer_->isChecked());
    relocalization_status_label_->setText(QStringLiteral("Candidate rank %1 loaded in source map frame; display only, analysis inputs unchanged.").arg(rank));
  });
}

void MainWindow::set_point_color_mode(PointColorMode mode) {
  const bool confidence = mode != PointColorMode::Height && mode != PointColorMode::Solid;
  if (confidence && confidence_model_.empty()) return;
  if (is_geometry_color_mode(mode) && geometry_model_.empty()) return;
  if (confidence && viewer_->mode() == InteractionMode::Delete) set_mode_select();
  viewer_->set_color_mode(mode);
  mode_delete_action_->setEnabled(!confidence);
  delete_points_action_->setEnabled(!confidence);
  hide_deleted_action_->setEnabled(!confidence);
  {
    const QSignalBlocker blocker(isolate_selection_action_);
    isolate_selection_action_->setChecked((confidence ? confidence_selection_manager_
                                                      : selection_manager_).isolate_selected());
  }
  refresh_confidence_editor_ui();
}

void MainWindow::refresh_confidence_editor_ui() {
  if (!confidence_edit_state_label_) return;
  if (confidence_model_.empty()) {
    confidence_edit_state_label_->setText(QStringLiteral("No confidence derivative loaded"));
    for (auto *button : {confidence_apply_button_, confidence_restore_button_,
                         confidence_undo_button_, confidence_redo_button_,
                         confidence_save_button_, confidence_rebuild_button_}) {
      if (button) button->setEnabled(false);
    }
    if (confidence_save_action_) confidence_save_action_->setEnabled(false);
    if (confidence_rebuild_action_) confidence_rebuild_action_->setEnabled(false);
    if (!review_mode_) setWindowTitle(QStringLiteral("AGT Map Studio"));
    update_edit_state_label();
    return;
  }
  const bool active = viewer_->showing_confidence();
  const bool busy = confidence_review_runner_.is_running();
  const auto selected = confidence_selection_manager_.selected_count();
  const bool dirty = confidence_editor_.dirty();
  confidence_edit_state_label_->setText(QStringLiteral(
      "%1 | %2 voxel(s) selected | %3 current overrides | stable preview %4.\n"
      "Saved intent: %5\n%6\n"
      "Source derivative is unchanged; only the in-memory final preview changes "
      "until an EXPLICIT core rebuild to a new directory. No navigation publication.")
      .arg(dirty ? QStringLiteral("DIRTY (unsaved intent)") : QStringLiteral("Intent unchanged"))
      .arg(static_cast<qulonglong>(selected))
      .arg(static_cast<qulonglong>(confidence_editor_.override_count()))
      .arg(static_cast<qulonglong>(confidence_editor_.stable_preview_count()))
      .arg(saved_confidence_intent_path_.isEmpty()
               ? QStringLiteral("none — Save Overrides before rebuilding")
               : saved_confidence_intent_path_)
      .arg(busy ? QStringLiteral("CORE REBUILD RUNNING — edits/source changes locked")
                : QStringLiteral("Review is a separate action")));
  confidence_apply_button_->setEnabled(active && !busy && selected != 0U);
  confidence_restore_button_->setEnabled(active && !busy && selected != 0U);
  confidence_undo_button_->setEnabled(active && !busy && confidence_editor_.can_undo());
  confidence_redo_button_->setEnabled(active && !busy && confidence_editor_.can_redo());
  const bool can_save = !busy;
  const bool can_review = !busy && !dirty && !saved_confidence_intent_path_.isEmpty();
  confidence_save_button_->setEnabled(can_save);
  confidence_rebuild_button_->setEnabled(can_review);
  confidence_save_action_->setEnabled(can_save);
  confidence_rebuild_action_->setEnabled(can_review);
  if (!review_mode_) {
    setWindowTitle(dirty ? QStringLiteral("AGT Map Studio — DIRTY spatial overrides")
                         : QStringLiteral("AGT Map Studio"));
  }
  update_edit_state_label();
}

bool MainWindow::confirm_discard_confidence_edits() {
  if (tool_runner_.is_running()) {
    QMessageBox::warning(this, QStringLiteral("Map Studio job running"),
        QStringLiteral("Wait for the current mapping, block, structure, or relocalization job to finish before switching sources."));
    return false;
  }
  if (confidence_review_runner_.is_running()) {
    QMessageBox::warning(this, QStringLiteral("Core review running"),
        QStringLiteral("Wait until the separate core review finishes before switching "
                       "mapping sources; its immutable input snapshot must remain available."));
    return false;
  }
  if (!confidence_editor_.dirty()) return true;
  return QMessageBox::warning(this, QStringLiteral("DIRTY spatial overrides"),
      QStringLiteral("Unsaved manual confidence override intent would be lost. "
                     "Discard edits? The source map and verified derivative are unchanged."),
      QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Cancel) == QMessageBox::Discard;
}

bool MainWindow::validate_relocalization_output_root(QString *error) const {
  const QString root = QDir::cleanPath(evidence_root_edit_ ? evidence_root_edit_->text().trimmed() : QString());
  if (root.isEmpty() || root == QStringLiteral(".")) {
    if (error) *error = QStringLiteral("Choose a new-asset output root first.");
    return false;
  }
  if (session_.source_is_mapping_package() &&
      path_is_same_or_within(root, session_.source_package_dir())) {
    if (error) *error = QStringLiteral("The output root must be outside the read-only mapping source package.");
    return false;
  }
  if (block_directory_edit_ && !block_directory_edit_->text().isEmpty() &&
      path_is_same_or_within(root, block_directory_edit_->text())) {
    if (error) *error = QStringLiteral("The output root must be outside the immutable block set.");
    return false;
  }
  if (evidence_path_edit_ && !evidence_path_edit_->text().isEmpty() &&
      path_is_same_or_within(root, evidence_path_edit_->text())) {
    if (error) *error = QStringLiteral("The output root must be outside the immutable evidence bundle.");
    return false;
  }
  return true;
}

void MainWindow::clear_geometry_view() {
  if (geometry_model_.empty()) return;
  if (is_geometry_color_mode(viewer_->color_mode())) {
    set_point_color_mode(confidence_model_.empty() ? PointColorMode::Height
                                                   : PointColorMode::AutoConfidence);
    if (!confidence_model_.empty()) confidence_color_actions_.front()->setChecked(true);
  }
  viewer_->set_geometry_model(nullptr);
  geometry_model_.clear();
  for (auto *action : geometry_color_actions_) action->setEnabled(false);
  if (geometry_summary_label_) geometry_summary_label_->setText(QStringLiteral(
      "Geometry sidecar: not loaded. Optional read-only directional evidence; "
      "V1 geometry_score remains 1 (deferred)."));
}

void MainWindow::clear_confidence_view() {
  clear_geometry_view();
  if (confidence_model_.empty() && confidence_derivative_dir_.isEmpty()) return;
  set_point_color_mode(PointColorMode::Height);
  height_coloring_action_->setChecked(true);
  viewer_->set_confidence_editor(nullptr);
  confidence_editor_.set_model(nullptr);
  viewer_->set_confidence_model(nullptr);
  confidence_model_.clear();
  confidence_derivative_dir_.clear();
  loaded_confidence_checksums_sha256_.clear();
  saved_confidence_intent_path_.clear();
  saved_confidence_intent_sha256_.clear();
  confidence_selection_manager_.reset(0);
  if (stable_only_action_) {
    stable_only_action_->setChecked(false);
    stable_only_action_->setEnabled(false);
  }
  for (auto *action : confidence_color_actions_) action->setEnabled(false);
  mode_delete_action_->setEnabled(true);
  delete_points_action_->setEnabled(true);
  hide_deleted_action_->setEnabled(true);
  if (confidence_dock_) confidence_dock_->hide();
  refresh_confidence_editor_ui();
}

void MainWindow::inspect_confidence_voxel(std::size_t index) {
  if (confidence_model_.empty() || !confidence_details_label_) return;
  refresh_confidence_editor_ui();
  const auto &info = confidence_model_.info();
  QString text = QStringLiteral(
      "Verified derivative: %1\nSource PGO package: %2\n"
      "Evidence voxels: %3; keyframes: %4\n"
      "Stable threshold: %5; preview selected: %6 (not a formal rebuild)\n\n")
      .arg(confidence_derivative_dir_)
      .arg(QString::fromStdString(info.source_package.string()))
      .arg(static_cast<qulonglong>(confidence_model_.voxels().size()))
      .arg(static_cast<qulonglong>(info.source_keyframes))
      .arg(info.stable_threshold, 0, 'f', 3)
      .arg(static_cast<qulonglong>(confidence_editor_.stable_preview_count()));
  if (index >= confidence_model_.voxels().size()) {
    if (!geometry_model_.empty()) text += QStringLiteral(
        "Geometry sidecar (read-only): %1\nHt unitless; Hr m^2; "
        "invalid evidence shown explicitly.\n")
        .arg(QString::fromStdString(geometry_model_.sidecar_directory().string()));
    confidence_details_label_->setText(text + QStringLiteral(
        "Ctrl+click a voxel center for its evidence, or select multiple voxels "
        "with Rectangle / Polygon / Sphere. Raw PCD point indices are never "
        "voxel indices. No manual intent is written to the source files."));
    return;
  }
  const auto &v = confidence_model_.voxels()[index];
  const auto *intent = confidence_editor_.intent(index);
  text += QStringLiteral("Voxel key [%1, %2, %3]  (#%4)\n")
      .arg(v.key.x).arg(v.key.y).arg(v.key.z).arg(static_cast<qulonglong>(index));
  text += QStringLiteral("Center [x,y,z]: %1, %2, %3 m\n")
      .arg(v.center.x(), 0, 'f', 3).arg(v.center.y(), 0, 'f', 3)
      .arg(v.center.z(), 0, 'f', 3);
  text += QStringLiteral("Source point count: %1; observed keyframes: %2\n")
      .arg(v.point_count).arg(v.observed_keyframes);
  text += QStringLiteral("First / last / span (keyframe index): %1 / %2 / %3\n")
      .arg(v.first_keyframe).arg(v.last_keyframe).arg(v.keyframe_span);
  text += QStringLiteral("Observation score: %1; persistence evidence: %2\n")
      .arg(v.observation_score, 0, 'f', 4)
      .arg(v.persistence_score, 0, 'f', 4);
  text += QStringLiteral("Geometry score: %1 (deferred; not edited)\n")
      .arg(v.geometry_score, 0, 'f', 4);
  text += QStringLiteral("Auto confidence: %1 (single-session evidence)\n")
      .arg(v.auto_confidence, 0, 'f', 4);
  text += QStringLiteral("Source derivative: %1; final %2\n")
      .arg(QString::fromLatin1(agt_spatial_map_core::override_mode_name(v.override_mode)))
      .arg(v.final_confidence, 0, 'f', 4);
  text += QStringLiteral("In-memory preview: %1; final %2; stable %3\n")
      .arg(QString::fromLatin1(agt_spatial_map_core::override_mode_name(
          confidence_editor_.effective_mode(index))))
      .arg(confidence_editor_.preview_final(index), 0, 'f', 4)
      .arg(confidence_editor_.stable_preview(index) ? QStringLiteral("yes")
                                                   : QStringLiteral("no"));
  if (intent) {
    text += QStringLiteral("Custom FORCE_LOW value: %1\n")
        .arg(intent->has_manual_value
            ? QString::number(intent->manual_value, 'f', 4) : QStringLiteral("none"));
    text += QStringLiteral("Reason: %1; editor: %2; edited (UTC): %3\n")
        .arg(intent->audit.reason.empty() ? QStringLiteral("legacy / unspecified")
                                           : QString::fromStdString(intent->audit.reason))
        .arg(intent->audit.editor.empty() ? QStringLiteral("unspecified")
                                           : QString::fromStdString(intent->audit.editor))
        .arg(intent->audit.edited_at.empty() ? QStringLiteral("unspecified")
                                              : QString::fromStdString(intent->audit.edited_at));
  }
  if (!geometry_model_.empty()) {
    const auto *g = geometry_model_.at_confidence_index(index);
    if (!g) {
      text += QStringLiteral("\nGeometry: no matching voxel (invalid model).\n");
    } else {
      const auto vec = [](const Eigen::Vector3f &x) {
        return QStringLiteral("[%1, %2, %3]")
            .arg(QString::number(x.x(), 'g', 6))
            .arg(QString::number(x.y(), 'g', 6))
            .arg(QString::number(x.z(), 'g', 6));
      };
      text += QStringLiteral("\nGEOMETRY EVIDENCE (read-only; not V1 confidence)\n"
                             "Raw points for local PCA: %1; valid-normal voxels: %2; "
                             "contributing raw observations: %3\n")
          .arg(g->normal_support_points)
          .arg(g->valid_normal_voxels)
          .arg(static_cast<qulonglong>(g->supporting_observations));
      if (!g->normal_valid) {
        text += QStringLiteral("Normal: INVALID (insufficient/collinear raw points); "
                               "no PCA shape score.\n");
      } else {
        text += QStringLiteral("Normal (map, sign arbitrary): %1\n"
                               "PCA lambda [min,mid,max] (m^2): %2\n"
                               "Linearity / planarity / scattering: %3 / %4 / %5\n")
            .arg(vec(g->normal)).arg(vec(g->covariance_eigenvalues))
            .arg(g->linearity, 0, 'f', 4).arg(g->planarity, 0, 'f', 4)
            .arg(g->scattering, 0, 'f', 4);
      }
      const auto append_spectrum = [&text, &vec](const char *name,
          const agt_spatial_map_core::GeometrySpectrum &s, const char *units) {
        if (!s.valid) {
          text += QStringLiteral("%1: INVALID (insufficient valid normals or zero trace).\n")
              .arg(QString::fromLatin1(name));
        } else {
          text += QStringLiteral("%1: lambda [min,mid,max] (%2): %3\n"
                                 "Weak axis (map): %4; Q: %5; condition: %6\n")
              .arg(QString::fromLatin1(name)).arg(QString::fromLatin1(units))
              .arg(vec(s.eigenvalues)).arg(vec(s.weak_direction))
              .arg(s.isotropy, 0, 'f', 4).arg(QString::number(s.condition, 'g', 6));
        }
      };
      append_spectrum("Ht (translation)", g->translation, "dimensionless");
      append_spectrum("Hr (rotation; T_map_body origin)", g->rotation, "m^2");
      text += QStringLiteral("Q describes directional diversity, not point density, "
                             "stability probability, or localization improvement.\n");
    }
  }
  confidence_details_label_->setText(text);
}

void MainWindow::apply_confidence_override() {
  if (confidence_review_runner_.is_running()) return;
  if (!viewer_->showing_confidence() || confidence_model_.empty() ||
      confidence_selection_manager_.selected_indices().empty()) return;
  const auto mode = static_cast<agt_spatial_map_core::ManualOverrideMode>(
      confidence_override_mode_combo_->currentData().toInt());
  if (mode == agt_spatial_map_core::ManualOverrideMode::AUTO) {
    restore_confidence_auto();
    return;
  }
  ConfidenceOverrideIntent intent;
  intent.mode = mode;
  intent.has_manual_value = mode == agt_spatial_map_core::ManualOverrideMode::FORCE_LOW &&
                            confidence_custom_low_check_->isChecked();
  if (intent.has_manual_value) {
    intent.manual_value = static_cast<float>(confidence_low_value_spin_->value());
  }
  intent.audit.reason = confidence_reason_combo_->currentText().toStdString();
  intent.audit.edited_at = QDateTime::currentDateTimeUtc()
      .toString(QStringLiteral("yyyy-MM-ddTHH:mm:ss'Z'")).toStdString();
  QString editor = qEnvironmentVariable("USER");
  editor.replace(QRegularExpression(QStringLiteral("[^A-Za-z0-9_.@-]")), QStringLiteral("_"));
  if (editor.isEmpty()) editor = QStringLiteral("map_studio");
  intent.audit.editor = editor.left(64).toStdString();
  std::string error;
  const auto &indices = confidence_selection_manager_.selected_indices();
  if (!confidence_editor_.apply(indices, intent, &error)) {
    statusBar()->showMessage(error.empty() ? QStringLiteral("No override state changed")
                                          : QString::fromStdString(error), 6000);
    return;
  }
  viewer_->refresh_confidence_preview();
  inspect_confidence_voxel(indices.front());
  statusBar()->showMessage(QStringLiteral("DIRTY: previewed manual intent for %1 voxels; "
                                           "Save Overrides separately")
                               .arg(static_cast<qulonglong>(indices.size())), 7000);
}

void MainWindow::restore_confidence_auto() {
  if (confidence_review_runner_.is_running()) return;
  if (!viewer_->showing_confidence() || confidence_model_.empty()) return;
  std::string error;
  const auto &indices = confidence_selection_manager_.selected_indices();
  if (!confidence_editor_.restore_auto(indices, &error)) {
    statusBar()->showMessage(error.empty() ? QStringLiteral("Selected voxels are already AUTO")
                                          : QString::fromStdString(error), 5000);
    return;
  }
  viewer_->refresh_confidence_preview();
  inspect_confidence_voxel(indices.empty() ? static_cast<std::size_t>(-1) : indices.front());
  statusBar()->showMessage(QStringLiteral("Restore Auto previewed; source evidence is unchanged"), 6000);
}

void MainWindow::undo_confidence_override() {
  if (confidence_review_runner_.is_running()) return;
  if (confidence_editor_.undo()) {
    viewer_->refresh_confidence_preview();
    const auto &indices = confidence_selection_manager_.selected_indices();
    inspect_confidence_voxel(indices.empty() ? static_cast<std::size_t>(-1) : indices.front());
  }
}

void MainWindow::redo_confidence_override() {
  if (confidence_review_runner_.is_running()) return;
  if (confidence_editor_.redo()) {
    viewer_->refresh_confidence_preview();
    const auto &indices = confidence_selection_manager_.selected_indices();
    inspect_confidence_voxel(indices.empty() ? static_cast<std::size_t>(-1) : indices.front());
  }
}

bool MainWindow::save_confidence_overrides(const QString &path, bool allow_replace,
                                           QString *error) {
  if (confidence_model_.empty() || confidence_review_runner_.is_running()) {
    if (error) *error = QStringLiteral("Open a derivative and wait for any core review to finish");
    return false;
  }
  try {
    if (QString::fromStdString(agt_spatial_map_core::sha256_file(
            confidence_model_.info().derivative_dir / "checksums.sha256")) !=
        loaded_confidence_checksums_sha256_) {
      throw std::runtime_error("source derivative changed after it was loaded; reload before saving");
    }
    std::string io_error;
    if (!SpatialConfidenceIntentIO::save(path.toStdString(), confidence_model_,
                                         confidence_editor_, allow_replace, &io_error)) {
      if (error) *error = QString::fromStdString(io_error);
      return false;
    }
    const QString written = QFileInfo(path).canonicalFilePath();
    const QString digest = QString::fromStdString(agt_spatial_map_core::sha256_file(
        written.toStdString()));
    saved_confidence_intent_path_ = written;
    saved_confidence_intent_sha256_ = digest;
    confidence_editor_.mark_saved();
    refresh_confidence_editor_ui();
    statusBar()->showMessage(QStringLiteral("Saved override INTENT only: %1; "
                                            "no reviewed map has been rebuilt").arg(written), 9000);
    return true;
  } catch (const std::exception &exception) {
    if (error) *error = QString::fromUtf8(exception.what());
    return false;
  }
}

void MainWindow::save_confidence_overrides_dialog() {
  if (confidence_model_.empty()) return;
  const QString suggested = saved_confidence_intent_path_.isEmpty()
      ? QDir(QFileInfo(confidence_derivative_dir_).absolutePath()).filePath(
            QStringLiteral("spatial_override_intent_%1.yaml").arg(stamp()))
      : saved_confidence_intent_path_;
  const QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("Save OVERRIDE INTENT ONLY (outside source package/derivative)"),
      suggested, QStringLiteral("YAML intent (*.yaml *.yml);;All files (*)"));
  if (path.isEmpty()) return;
  const bool exists = QFileInfo::exists(path);
  if (exists && QMessageBox::question(this, QStringLiteral("Replace saved intent YAML?"),
        QStringLiteral("Replace only this separate intent file?\n%1\n\nThe source derivative "
                       "and PGO map will never be replaced.").arg(path),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;
  QString error;
  if (!save_confidence_overrides(path, exists, &error)) {
    QMessageBox::critical(this, QStringLiteral("Save Overrides failed"), error);
  }
}

bool MainWindow::start_confidence_review(const QString &new_directory, QString *error) {
  if (confidence_model_.empty() || saved_confidence_intent_path_.isEmpty()) {
    if (error) *error = QStringLiteral("Open a verified derivative and Save Overrides YAML first");
    return false;
  }
  if (confidence_review_runner_.is_running() || tool_runner_.is_running() ||
      confidence_editor_.dirty()) {
    if (error) *error = QStringLiteral("Finish running tools and Save DIRTY override intent first");
    return false;
  }
  if (confidence_editor_.stable_preview_count() == 0) {
    if (error) *error = QStringLiteral("No stable voxels: core cannot publish an empty PCD");
    return false;
  }
  const std::filesystem::path target(new_directory.toStdString());
  try {
    if (new_directory.isEmpty() || target.filename().empty() || target.filename() == "." ||
        target.filename() == ".." || !std::filesystem::is_directory(target.parent_path()) ||
        std::filesystem::is_symlink(std::filesystem::symlink_status(target)) ||
        (std::filesystem::exists(target) &&
         (!std::filesystem::is_directory(target) || !std::filesystem::is_empty(target)))) {
      throw std::invalid_argument("review target must be a NEW or empty directory with an existing parent");
    }
    if (QString::fromStdString(agt_spatial_map_core::sha256_file(
            confidence_model_.info().derivative_dir / "checksums.sha256")) !=
        loaded_confidence_checksums_sha256_) {
      throw std::runtime_error("source derivative changed after loading; reopen it before review");
    }
    if (QString::fromStdString(agt_spatial_map_core::sha256_file(
            saved_confidence_intent_path_.toStdString())) !=
        saved_confidence_intent_sha256_) {
      throw std::runtime_error("saved override YAML changed since Save Overrides; save again");
    }
  } catch (const std::exception &exception) {
    if (error) *error = QString::fromUtf8(exception.what());
    return false;
  }
  QString missing;
  if (!tools_available({QStringLiteral("agt_spatial_map_core/agt_spatial_map_export")}, &missing)) {
    if (error) *error = QStringLiteral("Core CLI unavailable: %1").arg(missing);
    return false;
  }
  const QString absolute_target = QFileInfo(new_directory).absoluteFilePath();
  pending_review_target_ = absolute_target;
  pending_review_parent_ = QString::fromStdString(confidence_model_.info().source_package.string());
  pending_review_stable_count_ = confidence_editor_.stable_preview_count();
  ToolInvocation invocation = ExternalToolRunner::ros2_run(
      QStringLiteral("review spatial confidence (no navigation publish)"),
      QStringLiteral("agt_spatial_map_core"), QStringLiteral("agt_spatial_map_export"),
      {QStringLiteral("--map-package"), pending_review_parent_,
       QStringLiteral("--source-derivative"), confidence_derivative_dir_,
       QStringLiteral("--manual-overrides"), saved_confidence_intent_path_,
       QStringLiteral("--output-dir"), absolute_target});
  // Separate runner from the old 3D/2D/navigation workflow: core alone
  // computes final/stable/checksums and never writes the current derivative.
  confidence_review_runner_.start(invocation);
  refresh_confidence_editor_ui();
  return true;
}

void MainWindow::rebuild_confidence_review_dialog() {
  if (confidence_model_.empty()) return;
  if (confidence_editor_.dirty() || saved_confidence_intent_path_.isEmpty()) {
    QMessageBox::warning(this, QStringLiteral("Save Overrides first"),
        QStringLiteral("Save current intent YAML separately before the formal core rebuild."));
    return;
  }
  const QString suggested = QDir(QFileInfo(confidence_derivative_dir_).absolutePath()).filePath(
      QStringLiteral("reviewed_spatial_confidence_%1").arg(stamp()));
  const QString output = QFileDialog::getSaveFileName(
      this, QStringLiteral("Choose a NEW reviewed derivative DIRECTORY (not an existing file)"),
      suggested);
  if (output.isEmpty()) return;
  if (QMessageBox::question(this, QStringLiteral("Run formal core review?"),
      QStringLiteral("Source derivative (unchanged): %1\nPGO package (unchanged): %2\n"
                     "Saved intent YAML: %3\nNEW reviewed derivative directory: %4\n\n"
                     "Core preserves single-session automatic evidence, applies only your saved "
                     "override intent, rebuilds stable PCD and checksums. This does NOT publish "
                     "a navigation or production map. Continue?")
          .arg(confidence_derivative_dir_,
               QString::fromStdString(confidence_model_.info().source_package.string()),
               saved_confidence_intent_path_, output),
      QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;
  QString error;
  if (!start_confidence_review(output, &error)) {
    QMessageBox::critical(this, QStringLiteral("Core review not started"), error);
  }
}

void MainWindow::load_config(const QString &path) {
  if (path.isEmpty()) return;
  try {
    const YAML::Node root = YAML::LoadFile(path.toStdString());
    const YAML::Node camera = root["camera"];
    if (camera) {
      viewer_->set_camera_speeds(camera["speed"].as<float>(0.5F), camera["fast_speed"].as<float>(3.0F));
    }
    const YAML::Node viewer = root["viewer"];
    if (viewer) {
      viewer_->set_point_size(viewer["point_size"].as<float>(2.0F));
      const std::string background = viewer["background"].as<std::string>("white");
      dark_background_action_->setChecked(background == "dark");
      show_axis_action_->setChecked(viewer["show_axis"].as<bool>(true));
      const std::string color_mode = viewer["color_mode"].as<std::string>("height");
      const bool height = color_mode == "height";
      (height ? height_coloring_action_ : solid_coloring_action_)->setChecked(true);
      viewer_->set_height_coloring(height);
    }
    const YAML::Node workflow = root["workflow"];
    if (workflow) {
      const std::string map_root = workflow["map_root"].as<std::string>("");
      if (!map_root.empty()) default_map_root_ = QString::fromStdString(map_root);
      ConverterParameters &converter = session_.converter();
      converter.resolution = workflow["resolution"].as<double>(converter.resolution);
      converter.margin = workflow["margin"].as<double>(converter.margin);
      converter.min_points = workflow["min_points"].as<int>(converter.min_points);
      converter.max_step = workflow["max_step"].as<double>(converter.max_step);
      converter.max_slope_deg = workflow["max_slope_deg"].as<double>(converter.max_slope_deg);
      converter.use_trajectory = workflow["use_trajectory"].as<bool>(converter.use_trajectory);
      workflow_panel_->set_converter(converter);
    }
    const YAML::Node selection = root["selection"];
    if (selection) {
      sphere_radius_spin_->setValue(selection["sphere_radius_m"].as<double>(0.5));
      z_min_spin_->setValue(selection["z_window_min"].as<double>(-1.0));
      z_max_spin_->setValue(selection["z_window_max"].as<double>(3.0));
    }
  } catch (const std::exception &exception) {
    statusBar()->showMessage(QStringLiteral("Config warning: %1").arg(exception.what()), 5000);
  }
}

// ---------------------------------------------------------------------------
// Sources

void MainWindow::set_source(const QString &pcd_path, const QString &package_dir) {
  clear_confidence_view();
  session_.set_work_dir(QString());
  session_.reset(pcd_path, package_dir);
  QString base;
  if (!package_dir.isEmpty()) {
    const QFileInfo info(package_dir);
    base = QDir(info.absolutePath()).filePath(info.fileName() + QStringLiteral("_studio"));
  } else {
    const QFileInfo info(pcd_path);
    base = QDir(info.absolutePath()).filePath(info.completeBaseName() + QStringLiteral("_studio"));
  }
  session_.set_work_dir(base);
  if (block_directory_edit_) block_directory_edit_->clear();
  if (topology_path_edit_) topology_path_edit_->clear();
  if (evidence_path_edit_) evidence_path_edit_->clear();
  if (candidate_table_) candidate_table_->setRowCount(0);
  loaded_evidence_directory_.clear();
  has_picked_query_point_ = false;
  if (query_x_edit_) query_x_edit_->clear();
  if (query_y_edit_) query_y_edit_->clear();
  if (query_keyframe_spin_) {
    int maximum = 2000000;
    if (!package_dir.isEmpty()) {
      try {
        const YAML::Node metadata = YAML::LoadFile(QDir(package_dir).filePath(QStringLiteral("metadata.yaml")).toStdString());
        maximum = std::max(0, metadata["keyframe_count"].as<int>(1) - 1);
      } catch (const std::exception &) {}
    }
    query_keyframe_spin_->setRange(0, maximum);
    query_keyframe_spin_->setValue(std::min(query_keyframe_spin_->value(), maximum));
  }
  if (relocalization_status_label_)
    relocalization_status_label_->setText(package_dir.isEmpty()
        ? QStringLiteral("Open a mapping source package to enable block-based offline relocalization.")
        : QStringLiteral("Mapping source loaded. Build or load a checksum-verified immutable block set."));
  for (auto layer : {AuxiliaryLayer::Structure, AuxiliaryLayer::Blocks,
                     AuxiliaryLayer::Query, AuxiliaryLayer::Candidate})
    viewer_->clear_auxiliary_cloud(layer);
  session_.publish_target().map_root = default_map_root_;
  session_.publish_target().map_id =
      QFileInfo(package_dir.isEmpty() ? pcd_path : package_dir).completeBaseName();
  session_.publish_target().map_version =
      QStringLiteral("v%1-studio").arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmm")));
  workflow_panel_->set_publish_target(session_.publish_target());
  workflow_panel_->clear_log();
  refresh_workflow();
}

bool MainWindow::open_pcd(const QString &path, QString *error) {
  LoadedPointCloud loaded;
  std::string loader_error;
  if (!PCDLoader::load(path.toStdString(), &loaded, &loader_error)) {
    if (error) *error = QString::fromStdString(loader_error);
    return false;
  }
  if (!confirm_discard_confidence_edits()) {
    if (error) *error = QStringLiteral("Source change cancelled: spatial overrides are DIRTY");
    return false;
  }
  selection_manager_.reset(loaded.point_count());
  source_path_ = path;
  show_3d_view();
  viewer_->set_cloud(std::move(loaded), QFileInfo(path).fileName());
  z_min_spin_->setValue(viewer_->cloud().min_bound.z());
  z_max_spin_->setValue(viewer_->cloud().max_bound.z());
  set_source(path, QString());
  statusBar()->showMessage(viewer_->stats_text());
  return true;
}

bool MainWindow::open_mapping_package(const QString &directory, QString *error) {
  const QDir dir(directory);
  if (dir.exists(QStringLiteral("map.pcd")) && dir.exists(QStringLiteral("manifest.yaml"))) {
    if (!open_pcd(dir.filePath(QStringLiteral("map.pcd")), error)) return false;
    set_source(dir.filePath(QStringLiteral("map.pcd")), dir.absolutePath());
    statusBar()->showMessage(QStringLiteral("Opened mapping package: %1").arg(directory), 6000);
    return true;
  }
  const QString global_map = dir.filePath(QStringLiteral("localization/global_map.pcd"));
  if (QFileInfo::exists(global_map)) {
    if (!open_pcd(global_map, error)) return false;
    // Re-derive publish identity from maps/<map_id>/<version>.
    session_.publish_target().map_id = QFileInfo(dir.absolutePath()).dir().dirName();
    session_.publish_target().map_version = QStringLiteral("%1-studio_%2")
        .arg(dir.dirName(), QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmm")));
    workflow_panel_->set_publish_target(session_.publish_target());
    const QString map_yaml = dir.filePath(QStringLiteral("navigation/map.yaml"));
    if (QFileInfo::exists(map_yaml)) {
      QString map_error;
      if (load_navigation_dir_into_2d(dir.filePath(QStringLiteral("navigation")), &map_error)) {
        // Existing layers are consistent with the loaded PCD by construction;
        // treat them as fresh navigation output so 2D-only edits can be
        // patched and published without regenerating.
        session_.mark_done(WorkflowSession::Navigation, dir.filePath(QStringLiteral("navigation")),
                           WorkflowSession::sha256_file(dir.filePath(QStringLiteral("navigation/map.pgm"))),
                           QStringLiteral("imported from map package"));
        const QString reloc = dir.filePath(QStringLiteral("localization/relocalization"));
        if (QFileInfo::exists(reloc)) {
          session_.mark_done(WorkflowSession::Relocalization, reloc, session_.source_pcd_sha256(),
                             QStringLiteral("imported from map package"));
        }
      }
      show_3d_view();
    }
    refresh_workflow();
    statusBar()->showMessage(QStringLiteral("Opened map package: %1").arg(directory), 6000);
    return true;
  }
  if (error) {
    *error = QStringLiteral("%1 is neither a mapping package (map.pcd + manifest.yaml) nor a map "
                            "package (localization/global_map.pcd)").arg(directory);
  }
  return false;
}

bool MainWindow::open_spatial_confidence(const QString &directory, QString *error) {
  if (!session_.source_is_mapping_package()) {
    if (error) *error = QStringLiteral(
        "Open its original optimized PGO mapping package first (map.pcd + manifest.yaml); "
        "navigation releases and arbitrary PCDs have no keyframe evidence.");
    return false;
  }
  const QString parent = session_.source_package_dir();
  statusBar()->showMessage(QStringLiteral("Validating mapping parent and confidence checksums..."));
  QApplication::setOverrideCursor(Qt::WaitCursor);
  QString validation_error;
  const bool validated = validate_mapping_parent(parent, &validation_error);
  SpatialConfidenceModel loaded;
  std::string loader_error;
  const bool loaded_ok = validated && SpatialConfidenceLoader::load(
      directory.toStdString(), parent.toStdString(), &loaded, &loader_error);
  QApplication::restoreOverrideCursor();
  if (!validated || !loaded_ok) {
    if (error) *error = validated ? QString::fromStdString(loader_error) : validation_error;
    return false;  // previous confidence model and view remain intact
  }
  if (!confirm_discard_confidence_edits()) {
    if (error) *error = QStringLiteral("Confidence source change cancelled: overrides DIRTY or core review running");
    return false;
  }
  QString source_checksum_hash;
  try {
    source_checksum_hash = QString::fromStdString(agt_spatial_map_core::sha256_file(
        loaded.info().derivative_dir / "checksums.sha256"));
  } catch (const std::exception &exception) {
    if (error) *error = QString::fromUtf8(exception.what());
    return false;
  }
  clear_geometry_view(); // never retain a sidecar bound to the old derivative
  // Raw PCD and voxel representatives have disjoint indices and selection
  // histories. No spatial core evidence or source PCD value is mutated here.
  if (viewer_->mode() == InteractionMode::Delete) set_mode_select();
  set_point_color_mode(PointColorMode::Height);
  viewer_->set_confidence_editor(nullptr);
  confidence_editor_.set_model(nullptr);
  viewer_->set_confidence_model(nullptr);
  confidence_model_ = std::move(loaded);
  confidence_derivative_dir_ = QFileInfo(directory).canonicalFilePath();
  loaded_confidence_checksums_sha256_ = source_checksum_hash;
  saved_confidence_intent_path_.clear();
  saved_confidence_intent_sha256_.clear();
  confidence_selection_manager_.reset(confidence_model_.voxels().size());
  confidence_editor_.set_model(&confidence_model_);
  viewer_->set_confidence_model(&confidence_model_);
  viewer_->set_confidence_editor(&confidence_editor_);
  confidence_low_value_spin_->setValue(confidence_model_.info().force_low_value);
  stable_only_action_->setChecked(false);
  viewer_->set_stable_only(false);
  for (auto *action : confidence_color_actions_) action->setEnabled(true);
  stable_only_action_->setEnabled(true);
  confidence_color_actions_.front()->setChecked(true);
  set_point_color_mode(PointColorMode::AutoConfidence);
  show_3d_view();
  confidence_dock_->show();
  confidence_dock_->raise();  // foreground the evidence/editor tab, not Publish Workflow
  inspect_confidence_voxel(static_cast<std::size_t>(-1));
  statusBar()->showMessage(QStringLiteral("Verified %1 confidence voxels (single-session evidence)")
                               .arg(static_cast<qulonglong>(confidence_model_.voxels().size())), 8000);
  return true;
}

bool MainWindow::open_geometry_evidence(const QString &directory, QString *error) {
  if (!session_.source_is_mapping_package() || confidence_model_.empty() ||
      confidence_derivative_dir_.isEmpty()) {
    if (error) *error = QStringLiteral(
        "Open the verified optimized PGO parent AND its V1 confidence derivative first.");
    return false;
  }
  const QString parent = session_.source_package_dir();
  statusBar()->showMessage(QStringLiteral("Validating PGO / V1 source and geometry sidecar..."));
  QApplication::setOverrideCursor(Qt::WaitCursor);
  QString validation_error;
  const bool validated = validate_mapping_parent(parent, &validation_error);
  GeometryEvidenceModel loaded;
  std::string loader_error;
  bool loaded_ok = false;
  if (validated) {
    try {
      const auto digest = agt_spatial_map_core::sha256_file(
          confidence_model_.info().derivative_dir / "checksums.sha256");
      if (QString::fromStdString(digest) != loaded_confidence_checksums_sha256_) {
        throw std::runtime_error("loaded V1 derivative changed; reopen confidence before geometry");
      }
      loaded_ok = GeometryEvidenceLoader::load(directory.toStdString(), parent.toStdString(),
          confidence_derivative_dir_.toStdString(), confidence_model_, &loaded, &loader_error);
    } catch (const std::exception &exception) {
      loader_error = exception.what();
    }
  }
  QApplication::restoreOverrideCursor();
  if (!validated || !loaded_ok) {
    if (error) *error = validated ? QString::fromStdString(loader_error) : validation_error;
    return false; // preserve previous geometry model and any DIRTY confidence intent
  }
  clear_geometry_view();
  if (viewer_->mode() == InteractionMode::Delete) set_mode_select();
  geometry_model_ = std::move(loaded);
  viewer_->set_geometry_model(&geometry_model_);
  for (auto *action : geometry_color_actions_) action->setEnabled(true);
  geometry_color_actions_.front()->setChecked(true);
  set_point_color_mode(PointColorMode::GeometryNormalShape);
  geometry_summary_label_->setText(QStringLiteral(
      "Verified %1 geometry voxels from %2 (read-only). "
      "Ht is dimensionless; Hr is m^2. No geometry score or V1 change.")
      .arg(static_cast<qulonglong>(geometry_model_.evidence().voxels.size()))
      .arg(directory));
  show_3d_view();
  confidence_dock_->show();
  confidence_dock_->raise();
  inspect_confidence_voxel(static_cast<std::size_t>(-1));
  statusBar()->showMessage(QStringLiteral("Verified %1 read-only geometry evidence voxels")
      .arg(static_cast<qulonglong>(geometry_model_.evidence().voxels.size())), 8000);
  return true;
}

bool MainWindow::load_navigation_dir_into_2d(const QString &directory, QString *error) {
  const QString yaml = QDir(directory).filePath(QStringLiteral("map.yaml"));
  GridMap map;
  MapYamlMetadata metadata;
  std::string loader_error;
  if (!MapYamlLoader::load(yaml.toStdString(), &map, &metadata, &loader_error)) {
    if (error) *error = QString::fromStdString(loader_error);
    return false;
  }
  const std::vector<RefinementOperation> previous = refinement_model_.history();
  refinement_model_.set_base_map(map, metadata);
  occupancy_viewer_->set_refinement_model(&refinement_model_);
  occupancy_viewer_->set_map(std::move(map));
  if (!previous.empty() && !replay_2d_history(previous)) {
    statusBar()->showMessage(QStringLiteral("Some 2D edits could not be replayed on the new layers"), 6000);
  }
  sync_edit_fingerprints();
  return true;
}

bool MainWindow::replay_2d_history(const std::vector<RefinementOperation> &history) {
  bool all_ok = true;
  for (const auto &entry : history) {
    if (entry.undone) continue;
    std::unique_ptr<GridCommand> command;
    if (entry.type == "erase_rectangle" && entry.geometry.size() >= 2U) {
      command = EraseRectangleCommand::create(refinement_model_, entry.geometry[0], entry.geometry[1]);
    } else if (entry.type == "draw_obstacle" && entry.geometry.size() >= 2U) {
      command = DrawObstacleCommand::create(refinement_model_, entry.geometry[0], entry.geometry[1], entry.width_m);
    } else if (entry.type == "forbidden_polygon") {
      command = ForbiddenPolygonCommand::create(refinement_model_, entry.geometry);
    } else if (entry.type == "fill_free_polygon") {
      command = FillPolygonCommand::create(refinement_model_, entry.geometry, GridMap::kFree);
    } else if (entry.type == "fill_occupied_polygon") {
      command = FillPolygonCommand::create(refinement_model_, entry.geometry, GridMap::kOccupied);
    } else if (entry.type == "fill_unknown_polygon") {
      command = FillPolygonCommand::create(refinement_model_, entry.geometry, GridMap::kUnknown);
    }
    if (!command) continue;  // no-op on the new raster (already in target state)
    std::string error;
    if (!refinement_model_.execute(std::move(command), &error)) all_ok = false;
  }
  refresh_occupancy_view();
  return all_ok;
}

bool MainWindow::open_occupancy_map(const QString &path, QString *error) {
  GridMap map;
  MapYamlMetadata metadata;
  std::string loader_error;
  if (!MapYamlLoader::load(path.toStdString(), &map, &metadata, &loader_error)) {
    if (error) *error = QString::fromStdString(loader_error);
    return false;
  }
  refinement_model_.set_base_map(map, metadata);
  occupancy_viewer_->set_refinement_model(&refinement_model_);
  occupancy_viewer_->set_map(std::move(map));
  if (!session_.empty()) {
    // A hand-picked map.yaml becomes the navigation layers of the session.
    const QString directory = QFileInfo(path).absolutePath();
    session_.mark_done(WorkflowSession::Navigation, directory,
                       WorkflowSession::sha256_file(QDir(directory).filePath(QStringLiteral("map.pgm"))),
                       QStringLiteral("opened manually: %1").arg(path));
  }
  sync_edit_fingerprints();
  show_2d_view();
  statusBar()->showMessage(QStringLiteral("Opened occupancy map: %1").arg(path));
  return true;
}

bool MainWindow::open_mapping_review(const QString &package_dir, const QString &map_yaml,
                                     const QString &review_output, QString *error) {
  const QDir package(package_dir);
  const QString pcd = package.filePath(QStringLiteral("map.pcd"));
  const QString manifest = package.filePath(QStringLiteral("manifest.yaml"));
  if (!QFileInfo(pcd).isFile() || !QFileInfo(manifest).isFile()) {
    if (error) {
      *error = QStringLiteral("Review source is not a mapping package: %1").arg(package_dir);
    }
    return false;
  }
  if (!QFileInfo(map_yaml).isFile()) {
    if (error) *error = QStringLiteral("Review map does not exist: %1").arg(map_yaml);
    return false;
  }
  if (review_output.trimmed().isEmpty()) {
    if (error) *error = QStringLiteral("Review output directory is empty");
    return false;
  }
  if (!confirm_discard_confidence_edits()) {
    if (error) *error = QStringLiteral("2D review change cancelled: spatial overrides are DIRTY");
    return false;
  }

  // Do not call open_pcd(): post-mapping review deliberately avoids uploading
  // a potentially huge cloud to the OpenGL viewer.
  selection_manager_.reset(0);
  source_path_ = pcd;
  set_source(pcd, package.absolutePath());
  session_.set_work_dir(QFileInfo(review_output).absolutePath());
  if (!open_occupancy_map(map_yaml, error)) return false;

  review_mode_ = true;
  review_base_map_ = QFileInfo(map_yaml).absoluteFilePath();
  review_output_ = QFileInfo(review_output).absoluteFilePath();
  confirm_review_action_->setEnabled(true);
  setWindowTitle(QStringLiteral("AGT Map Studio — Lightweight 2D Review"));
  if (tools_menu_) tools_menu_->menuAction()->setVisible(false);
  if (workflow_dock_) {
    workflow_dock_->hide();
    workflow_dock_->toggleViewAction()->setVisible(false);
  }
  show_2d_view();
  statusBar()->showMessage(
      QStringLiteral("Edit the 2D map, then click Confirm & Save 2D Map. Output: %1")
          .arg(review_output_),
      12000);
  return true;
}

bool MainWindow::open_session(const QString &session_file, QString *error) {
  WorkflowSession restored;
  if (!restored.load(session_file, error)) return false;
  if (!open_pcd(restored.source_pcd(), error)) return false;
  const PublishTarget target = restored.publish_target();
  const ConverterParameters converter = restored.converter();
  session_ = restored;
  session_.converter() = converter;
  session_.publish_target() = target;
  workflow_panel_->set_converter(converter);
  workflow_panel_->set_publish_target(target);
  const QString navigation = session_.effective_navigation_dir().isEmpty()
                                 ? session_.record(WorkflowSession::Navigation).path
                                 : session_.effective_navigation_dir();
  if (!navigation.isEmpty() && QFileInfo::exists(QDir(navigation).filePath(QStringLiteral("map.yaml")))) {
    QString map_error;
    load_navigation_dir_into_2d(navigation, &map_error);
  }
  // In-memory edits are not persisted; the recorded fingerprints tell the
  // user which stages must be re-run once they redo their edits.
  refresh_workflow();
  statusBar()->showMessage(QStringLiteral("Restored session: %1").arg(session_file), 6000);
  return true;
}

// ---------------------------------------------------------------------------
// Dialogs

void MainWindow::open_pcd_dialog() {
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("Open PCD"), QString(), QStringLiteral("Point Cloud (*.pcd);;All Files (*)"));
  if (path.isEmpty()) return;
  QString error;
  if (!open_pcd(path, &error)) QMessageBox::critical(this, QStringLiteral("Open PCD failed"), error);
}

void MainWindow::open_mapping_package_dialog() {
  const QString directory = QFileDialog::getExistingDirectory(
      this, QStringLiteral("Open mapping package (map.pcd + manifest.yaml) or map package"));
  if (directory.isEmpty()) return;
  QString error;
  if (!open_mapping_package(directory, &error)) {
    QMessageBox::critical(this, QStringLiteral("Open package failed"), error);
  }
}

void MainWindow::open_spatial_confidence_dialog() {
  if (!session_.source_is_mapping_package()) {
    QMessageBox::warning(this, QStringLiteral("Open PGO source first"),
        QStringLiteral("Open the original optimized PGO mapping package before its "
                       "spatial confidence derivative. A navigation release is not a PGO source."));
    return;
  }
  const QString directory = QFileDialog::getExistingDirectory(
      this, QStringLiteral("Open spatial confidence derivative (five verified artifacts)"));
  if (directory.isEmpty()) return;
  QString error;
  if (!open_spatial_confidence(directory, &error)) {
    QMessageBox::critical(this, QStringLiteral("Open confidence derivative failed"), error);
  }
}

void MainWindow::open_geometry_evidence_dialog() {
  if (!session_.source_is_mapping_package() || confidence_model_.empty()) {
    QMessageBox::warning(this, QStringLiteral("Open verified sources first"),
        QStringLiteral("Open the optimized PGO mapping package and its matching "
                       "V1 confidence derivative before geometry evidence."));
    return;
  }
  const QString directory = QFileDialog::getExistingDirectory(
      this, QStringLiteral("Open geometry evidence sidecar (three verified files; read-only)"));
  if (directory.isEmpty()) return;
  QString error;
  if (!open_geometry_evidence(directory, &error)) {
    QMessageBox::critical(this, QStringLiteral("Open geometry sidecar failed"), error);
  }
}

void MainWindow::open_occupancy_map_dialog() {
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("Open Occupancy Map"), QString(), QStringLiteral("Nav2 map (*.yaml *.yml);;All Files (*)"));
  if (path.isEmpty()) return;
  QString error;
  if (!open_occupancy_map(path, &error)) {
    QMessageBox::critical(this, QStringLiteral("Open Occupancy Map failed"), error);
  }
}

void MainWindow::open_session_dialog() {
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("Open Studio Session"), QString(), QStringLiteral("studio_session.yaml (*.yaml)"));
  if (path.isEmpty()) return;
  QString error;
  if (!open_session(path, &error)) QMessageBox::critical(this, QStringLiteral("Open Session failed"), error);
}

void MainWindow::save_view_dialog() {
  const QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("Save View"), QStringLiteral("view.yaml"), QStringLiteral("YAML (*.yaml *.yml);;All Files (*)"));
  if (path.isEmpty()) return;
  QString error;
  if (!viewer_->save_view(path, &error)) {
    QMessageBox::critical(this, QStringLiteral("Save View failed"), error);
    return;
  }
  statusBar()->showMessage(QStringLiteral("Saved view: %1").arg(path), 5000);
}

void MainWindow::export_refinement_rules_dialog() {
  if (!selection_manager_.has_active_deletes()) {
    QMessageBox::information(this, QStringLiteral("Export Refinement Rules"),
                             QStringLiteral("No active 3D deletions to export."));
    return;
  }
  const QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("Export refinement rules"),
      session_.empty() ? QStringLiteral("refinement.yaml") : session_.refinement_rules_path(),
      QStringLiteral("YAML (*.yaml *.yml)"));
  if (path.isEmpty()) return;
  QString error;
  if (!selection_manager_.write_refinement_rules(path, source_path_, &error)) {
    QMessageBox::critical(this, QStringLiteral("Export failed"), error);
    return;
  }
  statusBar()->showMessage(QStringLiteral("Wrote %1").arg(path), 6000);
}

void MainWindow::export_navigation_patch_dialog() {
  if (!refinement_model_.has_map()) {
    QMessageBox::information(this, QStringLiteral("Export 2D Patch"), QStringLiteral("Open a 2D map first."));
    return;
  }
  const QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("Export patch_nav_map YAML"),
      session_.empty() ? QStringLiteral("navigation_patch.yaml") : session_.navigation_patch_path(),
      QStringLiteral("YAML (*.yaml *.yml)"));
  if (path.isEmpty()) return;
  std::string error;
  if (!refinement_model_.write_navigation_patch(path.toStdString(), &error)) {
    QMessageBox::critical(this, QStringLiteral("Export failed"), QString::fromStdString(error));
    return;
  }
  const QString keepout = QDir(QFileInfo(path).absolutePath()).filePath(QStringLiteral("keepout_zones.yaml"));
  refinement_model_.write_keepout_zones(keepout.toStdString(), &error);
  statusBar()->showMessage(QStringLiteral("Wrote %1 (+ keepout_zones.yaml)").arg(path), 6000);
}

void MainWindow::export_clean_map_dialog() {
  if (!viewer_->has_cloud()) {
    QMessageBox::information(this, QStringLiteral("Export Clean Map"), QStringLiteral("Open a PCD before exporting."));
    return;
  }
  const QString parent = QFileDialog::getExistingDirectory(
      this, QStringLiteral("Choose export parent directory"),
      source_path_.isEmpty() ? QString() : QFileInfo(source_path_).absolutePath());
  if (parent.isEmpty()) return;
  const QString output_dir = QDir(parent).filePath(QStringLiteral("clean_map"));
  QString error;
  if (!selection_manager_.export_clean_map(viewer_->cloud(), output_dir, source_path_, &error)) {
    QMessageBox::critical(this, QStringLiteral("Export Clean Map failed"), error);
    return;
  }
  statusBar()->showMessage(QStringLiteral("Exported clean map (preview artifact): %1").arg(output_dir), 8000);
}

void MainWindow::generate_occupancy_preview_dialog() {
  if (!viewer_->has_cloud()) {
    QMessageBox::information(this, QStringLiteral("Occupancy Preview"), QStringLiteral("Open a PCD first."));
    return;
  }
  agt_pcd2grid_exporter::ProjectionParameters parameters;
  QString config_path;
  try {
    const std::string share = ament_index_cpp::get_package_share_directory("agt_pcd2grid_exporter");
    config_path = QString::fromStdString(share + "/config/projection.yaml");
  } catch (const std::exception &) {
  }
  if (!config_path.isEmpty() && QFileInfo::exists(config_path)) {
    std::string parameter_error;
    if (!agt_pcd2grid_exporter::ParameterLoader::load(config_path.toStdString(), &parameters, &parameter_error)) {
      QMessageBox::critical(this, QStringLiteral("Projection parameters"), QString::fromStdString(parameter_error));
      return;
    }
  }
  const QString summary = QStringLiteral(
      "This is a quick studio-side preview (agt_pcd2grid_exporter). It is NOT the navigation "
      "contract; publishable layers come from step 3 (pcd_to_nav_map).\n\nResolution: %1 m\n"
      "Z filter: [%2, %3] m\nOccupied threshold: %4 hits\n\nRender preview now?")
      .arg(parameters.resolution, 0, 'f', 3).arg(parameters.z_min, 0, 'f', 3)
      .arg(parameters.z_max, 0, 'f', 3).arg(parameters.occupied_threshold);
  if (QMessageBox::question(this, QStringLiteral("Occupancy Preview"), summary,
                            QMessageBox::Yes | QMessageBox::Cancel) != QMessageBox::Yes) {
    return;
  }
  agt_pcd2grid_exporter::OccupancyGrid grid;
  agt_pcd2grid_exporter::ProjectionStats stats;
  std::string error;
  if (!agt_pcd2grid_exporter::PCDProjector::project(*viewer_->cloud().source, parameters, &grid, &stats, &error)) {
    QMessageBox::critical(this, QStringLiteral("Occupancy Preview failed"), QString::fromStdString(error));
    return;
  }
  if (grid.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      grid.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    return;
  }
  QImage preview(static_cast<int>(grid.width), static_cast<int>(grid.height), QImage::Format_Grayscale8);
  for (std::uint32_t gy = 0; gy < grid.height; ++gy) {
    auto *line = preview.scanLine(static_cast<int>(grid.height - 1U - gy));
    for (std::uint32_t gx = 0; gx < grid.width; ++gx) {
      const auto value = grid.value(static_cast<std::size_t>(gy) * grid.width + gx, parameters);
      line[gx] = value == 100 ? 0U : (value == 0 ? 254U : 205U);
    }
  }
  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("Occupancy Preview (not publishable)"));
  auto *layout = new QVBoxLayout(&dialog);
  auto *label = new QLabel(&dialog);
  label->setAlignment(Qt::AlignCenter);
  label->setPixmap(QPixmap::fromImage(preview.scaled(1000, 700, Qt::KeepAspectRatio, Qt::FastTransformation)));
  layout->addWidget(label);
  dialog.resize(1020, 740);
  dialog.exec();
}

void MainWindow::save_refinement_dialog() {
  if (!refinement_model_.has_map()) {
    QMessageBox::information(this, QStringLiteral("Save 2D Refinement"), QStringLiteral("Open an occupancy map first."));
    return;
  }
  const QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("Save 2D Refinement History"), QStringLiteral("map_refinement.yaml"),
      QStringLiteral("YAML (*.yaml *.yml);;All Files (*)"));
  if (path.isEmpty()) return;
  std::string error;
  if (!refinement_model_.save_refinement_yaml(path.toStdString(), &error)) {
    QMessageBox::critical(this, QStringLiteral("Save failed"), QString::fromStdString(error));
    return;
  }
  statusBar()->showMessage(QStringLiteral("Saved refinement: %1").arg(path), 6000);
}

void MainWindow::export_navigation_map_dialog() {
  if (!refinement_model_.has_map()) {
    QMessageBox::information(this, QStringLiteral("Export Edited PGM"), QStringLiteral("Open an occupancy map first."));
    return;
  }
  const QString parent = QFileDialog::getExistingDirectory(
      this, QStringLiteral("Choose output parent (preview only, not a navigation contract)"),
      QFileInfo(QString::fromStdString(refinement_model_.base_map().yaml_path())).absolutePath());
  if (parent.isEmpty()) return;
  const QString output = QDir(parent).filePath(QStringLiteral("navigation_map_preview"));
  std::string error;
  if (!refinement_model_.export_navigation_map(output.toStdString(), &error)) {
    QMessageBox::critical(this, QStringLiteral("Export failed"), QString::fromStdString(error));
    return;
  }
  statusBar()->showMessage(QStringLiteral("Exported edited PGM preview: %1").arg(output), 8000);
}

void MainWindow::confirm_mapping_review() {
  if (!review_mode_ || !refinement_model_.has_map() || review_output_.isEmpty()) return;
  if (QMessageBox::question(
          this, QStringLiteral("Confirm 2D Map"),
          QStringLiteral("Save the current edited map as the confirmed result?\n\n%1")
              .arg(review_output_),
          QMessageBox::Yes | QMessageBox::Cancel) != QMessageBox::Yes) {
    return;
  }

  std::string export_error;
  if (!refinement_model_.export_navigation_map(review_output_.toStdString(), &export_error)) {
    QMessageBox::critical(this, QStringLiteral("Confirm failed"),
                          QString::fromStdString(export_error));
    return;
  }
  if (!refinement_model_.write_keepout_zones(
          QDir(review_output_).filePath(QStringLiteral("keepout_zones.yaml")).toStdString(),
          &export_error)) {
    QMessageBox::critical(this, QStringLiteral("Confirm failed"),
                          QString::fromStdString(export_error));
    return;
  }

  const QString status_path = QDir(review_output_).filePath(QStringLiteral("review_status.yaml"));
  try {
    YAML::Emitter document;
    document << YAML::BeginMap;
    document << YAML::Key << "schema_version" << YAML::Value << 1;
    document << YAML::Key << "status" << YAML::Value << "confirmed";
    document << YAML::Key << "confirmed_at" << YAML::Value
             << WorkflowSession::now_iso8601().toStdString();
    document << YAML::Key << "source_mapping_package" << YAML::Value
             << session_.source_package_dir().toStdString();
    document << YAML::Key << "source_pcd" << YAML::Value << session_.source_pcd().toStdString();
    document << YAML::Key << "base_map" << YAML::Value << review_base_map_.toStdString();
    document << YAML::Key << "confirmed_map" << YAML::Value
             << QDir(review_output_).filePath(QStringLiteral("map.yaml")).toStdString();
    document << YAML::Key << "edited_cells" << YAML::Value
             << refinement_model_.active_override_count();
    document << YAML::Key << "forbidden_zones" << YAML::Value
             << refinement_model_.forbidden_zones().size();
    document << YAML::EndMap;
    std::ofstream stream(status_path.toStdString(), std::ios::out | std::ios::trunc);
    if (!stream) throw std::runtime_error("cannot open confirmation status file");
    stream << document.c_str() << '\n';
    if (!stream.good()) throw std::runtime_error("cannot write confirmation status file");
  } catch (const std::exception &exception) {
    QMessageBox::critical(
        this, QStringLiteral("Confirm failed"),
        QStringLiteral("Map was exported, but confirmation metadata could not be written: %1")
            .arg(exception.what()));
    return;
  }

  sync_edit_fingerprints();
  const QString output_map = QDir(review_output_).filePath(QStringLiteral("map.pgm"));
  session_.mark_done(WorkflowSession::Patch, review_output_,
                     WorkflowSession::sha256_file(output_map),
                     QStringLiteral("confirmed by lightweight 2D review"));
  save_session_quietly();
  statusBar()->showMessage(QStringLiteral("Confirmed map saved: %1").arg(review_output_), 12000);
  QMessageBox::information(
      this, QStringLiteral("2D Map Confirmed"),
      QStringLiteral("The confirmed map and edit history were saved to:\n%1")
          .arg(review_output_));
}

void MainWindow::select_height_band_dialog() {
  if (!viewer_->has_cloud() && !viewer_->has_confidence()) return;
  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("Select height band"));
  auto *form = new QFormLayout(&dialog);
  auto *min_spin = new QDoubleSpinBox(&dialog);
  auto *max_spin = new QDoubleSpinBox(&dialog);
  for (auto *spin : {min_spin, max_spin}) {
    spin->setRange(-1000.0, 1000.0);
    spin->setDecimals(2);
  }
  float min_z = viewer_->cloud().min_bound.z();
  if (viewer_->showing_confidence()) {
    min_z = std::numeric_limits<float>::infinity();
    for (const auto &voxel : confidence_model_.voxels()) {
      min_z = std::min(min_z, voxel.center.z());
    }
  }
  min_spin->setValue(min_z);
  max_spin->setValue(min_z + 0.2);
  form->addRow(QStringLiteral("Z min (m)"), min_spin);
  form->addRow(QStringLiteral("Z max (m)"), max_spin);
  auto *note = new QLabel(viewer_->showing_confidence()
      ? QStringLiteral("Selects voxel representatives; Delete is disabled. Inspect or override evidence.")
      : QStringLiteral("Selects every visible point with min <= z <= max\n"
                       "(e.g. ceiling band or ground noise). Then press Delete in Delete mode."), &dialog);
  form->addRow(note);
  auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  form->addRow(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  if (dialog.exec() != QDialog::Accepted) return;
  if (viewer_->mode() == InteractionMode::Navigate) set_mode_select();
  viewer_->select_height_band(min_spin->value(), max_spin->value());
  statusBar()->showMessage(QStringLiteral("Height band selected: %1 %2")
                               .arg(static_cast<qulonglong>(viewer_->showing_confidence()
                                   ? confidence_selection_manager_.selected_count()
                                   : selection_manager_.selected_count()))
                               .arg(viewer_->showing_confidence() ? QStringLiteral("voxels")
                                                                  : QStringLiteral("points")), 5000);
}

// ---------------------------------------------------------------------------
// Editing

void MainWindow::reset_camera() { viewer_->reset_camera(); }

void MainWindow::undo_edit() {
  if (view_stack_->currentWidget() == occupancy_viewer_) {
    if (refinement_model_.undo()) refresh_occupancy_view();
  } else if (viewer_->showing_confidence()) {
    undo_confidence_override();
  } else if (selection_manager_.undo()) {
    viewer_->mark_edit_state_dirty();
    sync_edit_fingerprints();
  }
}

void MainWindow::redo_edit() {
  if (view_stack_->currentWidget() == occupancy_viewer_) {
    if (refinement_model_.redo()) refresh_occupancy_view();
  } else if (viewer_->showing_confidence()) {
    redo_confidence_override();
  } else if (selection_manager_.redo()) {
    viewer_->mark_edit_state_dirty();
    sync_edit_fingerprints();
  }
}

void MainWindow::delete_selected() {
  if (view_stack_->currentWidget() != viewer_) return;
  if (viewer_->showing_confidence()) {
    statusBar()->showMessage(QStringLiteral("Voxel deletion is disabled: use manual override intent"), 5000);
    return;
  }
  if (viewer_->mode() != InteractionMode::Delete) {
    statusBar()->showMessage(QStringLiteral("Switch to Delete mode (toolbar or X) first"), 3000);
    return;
  }
  if (selection_manager_.delete_selected()) {
    viewer_->mark_edit_state_dirty();
    sync_edit_fingerprints();
  }
}

void MainWindow::set_mode_navigate() {
  viewer_->set_mode(InteractionMode::Navigate);
  mode_navigate_action_->setChecked(true);
}
void MainWindow::set_mode_select() {
  viewer_->set_mode(InteractionMode::Select);
  mode_select_action_->setChecked(true);
}
void MainWindow::set_mode_delete() {
  if (viewer_->showing_confidence()) {
    set_mode_select();
    statusBar()->showMessage(QStringLiteral("Voxel deletion is disabled: use manual override intent"), 5000);
    return;
  }
  viewer_->set_mode(InteractionMode::Delete);
  mode_delete_action_->setChecked(true);
}
void MainWindow::set_isometric_view() { viewer_->isometric_view(); }
void MainWindow::set_front_view() { viewer_->front_view(); }
void MainWindow::set_top_view() { viewer_->top_view(); }

void MainWindow::show_3d_view() {
  view_stack_->setCurrentWidget(viewer_);
  if (show_3d_action_) show_3d_action_->setChecked(true);
  if (occupancy_toolbar_) occupancy_toolbar_->setVisible(false);
  if (toolbar_3d_) toolbar_3d_->setVisible(true);
  statusBar()->showMessage(viewer_->stats_text());
}

void MainWindow::show_2d_view() {
  view_stack_->setCurrentWidget(occupancy_viewer_);
  if (show_2d_action_) show_2d_action_->setChecked(true);
  if (occupancy_toolbar_) occupancy_toolbar_->setVisible(true);
  if (toolbar_3d_) toolbar_3d_->setVisible(false);
  statusBar()->showMessage(refinement_model_.has_map()
                               ? QStringLiteral("2D navigation view")
                               : QStringLiteral("2D view: no layers yet - run step 3 or open a map.yaml"));
}

void MainWindow::set_occupancy_mode(OccupancyInteractionMode mode) { occupancy_viewer_->set_mode(mode); }

void MainWindow::apply_erase_rectangle(double min_x, double min_y, double max_x, double max_y) {
  if (!refinement_model_.has_map()) return;
  auto command = EraseRectangleCommand::create(refinement_model_, {min_x, min_y}, {max_x, max_y});
  if (!command) {
    statusBar()->showMessage(QStringLiteral("No occupied cells in selected rectangle"), 3000);
    return;
  }
  std::string error;
  if (!refinement_model_.execute(std::move(command), &error)) {
    QMessageBox::critical(this, QStringLiteral("Erase failed"), QString::fromStdString(error));
    return;
  }
  refresh_occupancy_view();
}

void MainWindow::apply_obstacle_line(double start_x, double start_y, double end_x, double end_y, double width_m) {
  auto command = DrawObstacleCommand::create(refinement_model_, {start_x, start_y}, {end_x, end_y}, width_m);
  if (!command) {
    statusBar()->showMessage(QStringLiteral("No free cells in obstacle line"), 3000);
    return;
  }
  std::string error;
  if (!refinement_model_.execute(std::move(command), &error)) {
    QMessageBox::critical(this, QStringLiteral("Obstacle draw failed"), QString::fromStdString(error));
    return;
  }
  refresh_occupancy_view();
}

void MainWindow::apply_forbidden_polygon(const QVector<QPointF> &polygon) {
  std::vector<GridWorldPoint> points;
  points.reserve(static_cast<std::size_t>(polygon.size()));
  for (const auto &point : polygon) points.push_back({point.x(), point.y()});
  auto command = ForbiddenPolygonCommand::create(refinement_model_, std::move(points));
  if (!command) return;
  std::string error;
  if (!refinement_model_.execute(std::move(command), &error)) {
    QMessageBox::critical(this, QStringLiteral("Forbidden zone failed"), QString::fromStdString(error));
    return;
  }
  refresh_occupancy_view();
}

void MainWindow::apply_fill_polygon(const QVector<QPointF> &polygon, int value) {
  std::vector<GridWorldPoint> points;
  points.reserve(static_cast<std::size_t>(polygon.size()));
  for (const auto &point : polygon) points.push_back({point.x(), point.y()});
  auto command = FillPolygonCommand::create(refinement_model_, points, static_cast<std::int8_t>(value));
  if (!command) {
    statusBar()->showMessage(QStringLiteral("Polygon changes no cells"), 3000);
    return;
  }
  std::string error;
  if (!refinement_model_.execute(std::move(command), &error)) {
    QMessageBox::critical(this, QStringLiteral("Polygon fill failed"), QString::fromStdString(error));
    return;
  }
  refresh_occupancy_view();
}

void MainWindow::refresh_occupancy_view() {
  occupancy_viewer_->refresh();
  statusBar()->showMessage(QStringLiteral("2D edits: %1 cell overrides, %2 patch polygons, %3 forbidden zones")
                               .arg(refinement_model_.active_override_count())
                               .arg(refinement_model_.patch_edit_count())
                               .arg(refinement_model_.forbidden_zones().size()));
  sync_edit_fingerprints();
}

void MainWindow::sync_edit_fingerprints() {
  session_.set_refinement_fingerprint(selection_manager_.active_fingerprint());
  // Forbidden zones do not change the raster; only patch edits mark the
  // navigation layers stale. Zones still ride along in pipeline.yaml.
  session_.set_patch_fingerprint(
      refinement_model_.patch_edit_count() > 0
          ? QString::fromStdString(refinement_model_.active_fingerprint())
          : QString());
  refresh_workflow();
}

void MainWindow::refresh_workflow() {
  if (!workflow_panel_) return;
  workflow_panel_->refresh(session_, tool_runner_.is_running());
  update_edit_state_label();
}

void MainWindow::update_edit_state_label() {
  if (!edit_state_label_) return;
  QStringList parts;
  if (session_.has_3d_edits()) {
    parts << QStringLiteral("3D: %1").arg(session_.state(WorkflowSession::Refine) == StageState::Fresh
                                               ? QStringLiteral("refined") : QStringLiteral("unapplied"));
  }
  if (session_.has_2d_edits()) {
    parts << QStringLiteral("2D: %1").arg(session_.state(WorkflowSession::Patch) == StageState::Fresh
                                               ? QStringLiteral("patched") : QStringLiteral("unapplied"));
  }
  // Legacy workflow edits and confidence intent are disjoint, but neither may
  // be silently called "no pending edits" when a human override is unsaved.
  if (confidence_review_runner_.is_running()) {
    parts << QStringLiteral("spatial review: core rebuilding a new derivative");
  } else if (confidence_editor_.dirty()) {
    parts << QStringLiteral("spatial overrides: DIRTY intent (not rebuilt)");
  } else if (!saved_confidence_intent_path_.isEmpty()) {
    parts << QStringLiteral("spatial overrides: saved intent (review separate)");
  }
  edit_state_label_->setText(parts.isEmpty() ? QStringLiteral("no pending edits")
                                             : parts.join(QStringLiteral(" | ")));
}

// ---------------------------------------------------------------------------
// Workflow execution

QString MainWindow::stamp() const {
  return QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
}

bool MainWindow::ensure_work_dir(QString *error) {
  if (session_.empty()) {
    if (error) *error = QStringLiteral("Open a PCD or package first");
    return false;
  }
  if (!QDir().mkpath(session_.work_dir())) {
    if (error) *error = QStringLiteral("Cannot create %1").arg(session_.work_dir());
    return false;
  }
  return true;
}

bool MainWindow::tools_available(const QStringList &required, QString *missing) const {
  if (!ExternalToolRunner::program_available(QStringLiteral("ros2"))) {
    if (missing) *missing = QStringLiteral("ros2 (source /opt/ros/humble/setup.bash and the workspace overlay)");
    return false;
  }
  QStringList absent;
  for (const QString &entry : required) {
    const QStringList parts = entry.split('/');
    if (parts.size() == 2 && !ExternalToolRunner::ros2_executable_available(parts[0], parts[1])) absent << entry;
  }
  if (!absent.isEmpty()) {
    if (missing) *missing = absent.join(QStringLiteral(", "));
    return false;
  }
  return true;
}

void MainWindow::run_tool(const ToolInvocation &invocation, std::function<void(const ToolResult &)> on_done) {
  if (tool_runner_.is_running() || confidence_review_runner_.is_running()) {
    QMessageBox::information(this, QStringLiteral("Busy"), QStringLiteral("Another tool is still running."));
    return;
  }
  ToolInvocation prepared = invocation;
  prepared.log_path = session_.log_path();
  tool_callback_ = std::move(on_done);
  tool_runner_.start(prepared);
}

void MainWindow::cancel_tool() {
  step_queue_.clear();
  queue_running_ = false;
  tool_runner_.cancel();
}

void MainWindow::save_session_quietly() {
  QString error;
  if (!session_.save(&error)) statusBar()->showMessage(QStringLiteral("Session not saved: %1").arg(error), 5000);
}

void MainWindow::continue_queue() {
  if (!queue_running_) return;
  if (step_queue_.empty()) {
    queue_running_ = false;
    statusBar()->showMessage(QStringLiteral("All pending steps completed"), 8000);
    workflow_panel_->set_progress(QStringLiteral("Done"), false);
    return;
  }
  StepFn next = std::move(step_queue_.front());
  step_queue_.erase(step_queue_.begin());
  next();
}

void MainWindow::fail_queue(const QString &message) {
  step_queue_.clear();
  queue_running_ = false;
  workflow_panel_->set_progress(QStringLiteral("Failed"), false);
  QMessageBox::critical(this, QStringLiteral("Workflow step failed"), message);
  refresh_workflow();
}

void MainWindow::run_all_pending() {
  if (session_.empty()) return;
  step_queue_.clear();
  if (session_.has_3d_edits() && session_.state(WorkflowSession::Refine) != StageState::Fresh) {
    step_queue_.push_back([this]() { run_refine(); });
  }
  step_queue_.push_back([this]() {
    if (session_.state(WorkflowSession::Relocalization) != StageState::Fresh) run_relocalization();
    else continue_queue();
  });
  step_queue_.push_back([this]() {
    if (session_.state(WorkflowSession::Navigation) != StageState::Fresh) run_navigation();
    else continue_queue();
  });
  step_queue_.push_back([this]() {
    if (session_.has_2d_edits() && session_.state(WorkflowSession::Patch) != StageState::Fresh) run_patch();
    else continue_queue();
  });
  step_queue_.push_back([this]() {
    if (session_.blocking_reasons_for_publish().isEmpty()) run_publish();
    else {
      statusBar()->showMessage(QStringLiteral("Publish skipped: %1")
                                   .arg(session_.blocking_reasons_for_publish().join(QStringLiteral("; "))), 8000);
      continue_queue();
    }
  });
  queue_running_ = true;
  continue_queue();
}

void MainWindow::run_refine() {
  QString error;
  if (!ensure_work_dir(&error)) return fail_queue(error);
  if (!selection_manager_.has_active_deletes()) {
    statusBar()->showMessage(QStringLiteral("No 3D deletions to apply"), 4000);
    return continue_queue();
  }
  if (!selection_manager_.write_refinement_rules(session_.refinement_rules_path(), session_.source_pcd(), &error)) {
    return fail_queue(error);
  }
  const QString output = QDir(session_.work_dir()).filePath(QStringLiteral("refined_mapping_source_%1").arg(stamp()));
  if (session_.source_is_mapping_package()) {
    QString missing;
    if (!tools_available({QStringLiteral("%1/%2").arg(kRefinementPackage, kRefinementTool)}, &missing)) {
      return fail_queue(QStringLiteral("Missing tool: %1").arg(missing));
    }
    ToolInvocation invocation = ExternalToolRunner::ros2_run(
        QStringLiteral("apply_map_refinement"), kRefinementPackage, kRefinementTool,
        {QStringLiteral("--map-package"), session_.source_package_dir(),
         QStringLiteral("--refinement"), session_.refinement_rules_path(),
         QStringLiteral("--output"), output,
         QStringLiteral("--pcd-format"), QStringLiteral("binary"),
         QStringLiteral("--no-preview-nav-map")});
    run_tool(invocation, [this, output](const ToolResult &result) {
      if (!result.ok) return fail_queue(result.error_summary);
      const QString map = QDir(output).filePath(QStringLiteral("map.pcd"));
      session_.mark_done(WorkflowSession::Refine, output, WorkflowSession::sha256_file(map),
                         QStringLiteral("apply_map_refinement"));
      save_session_quietly();
      statusBar()->showMessage(QStringLiteral("Refined package: %1").arg(output), 8000);
      continue_queue();
    });
    return;
  }
  // Bare PCD: the studio writes the filtered binary PCD itself.
  workflow_panel_->append_log(QStringLiteral("$ studio export_clean_map -> %1\n").arg(output));
  if (!selection_manager_.export_clean_map(viewer_->cloud(), output, session_.source_pcd(), &error)) {
    return fail_queue(error);
  }
  session_.mark_done(WorkflowSession::Refine, output,
                     WorkflowSession::sha256_file(QDir(output).filePath(QStringLiteral("map.pcd"))),
                     QStringLiteral("studio clean map (no poses)"));
  save_session_quietly();
  refresh_workflow();
  statusBar()->showMessage(QStringLiteral("Refined PCD written: %1").arg(output), 8000);
  continue_queue();
}

void MainWindow::run_relocalization() {
  QString error;
  if (!ensure_work_dir(&error)) return fail_queue(error);
  if (session_.has_3d_edits() && session_.state(WorkflowSession::Refine) != StageState::Fresh) {
    return fail_queue(QStringLiteral("3D edits are not applied yet: run step 1 first."));
  }
  QString missing;
  if (!tools_available({QStringLiteral("%1/%2").arg(kRelocPackage, kRelocTool)}, &missing)) {
    return fail_queue(QStringLiteral("Missing tool: %1").arg(missing));
  }
  const QString output = QDir(session_.work_dir()).filePath(QStringLiteral("relocalization_%1").arg(stamp()));
  QDir().mkpath(output);
  const QString pcd = session_.effective_pcd();
  ToolInvocation invocation = ExternalToolRunner::ros2_run(
      QStringLiteral("build_relocalization_assets"), kRelocPackage, kRelocTool,
      {QStringLiteral("--map"), pcd, QStringLiteral("--output"), output});
  run_tool(invocation, [this, output](const ToolResult &result) {
    if (!result.ok) return fail_queue(result.error_summary);
    session_.mark_done(WorkflowSession::Relocalization, output, session_.effective_pcd_sha256(),
                       QStringLiteral("build_relocalization_assets"));
    save_session_quietly();
    continue_queue();
  });
}

void MainWindow::run_navigation() {
  QString error;
  if (!ensure_work_dir(&error)) return fail_queue(error);
  if (session_.has_3d_edits() && session_.state(WorkflowSession::Refine) != StageState::Fresh) {
    return fail_queue(QStringLiteral("3D edits are not applied yet: run step 1 first."));
  }
  QString missing;
  if (!tools_available({QStringLiteral("%1/pcd_to_nav_map").arg(kConverterPackage),
                        QStringLiteral("%1/validate_nav_map").arg(kConverterPackage)}, &missing)) {
    return fail_queue(QStringLiteral("Missing tool: %1").arg(missing));
  }
  workflow_panel_->read_converter(&session_.converter());
  const QString output = QDir(session_.work_dir()).filePath(QStringLiteral("navigation_%1").arg(stamp()));
  const QString pcd = session_.effective_pcd();
  QStringList arguments{pcd, QStringLiteral("-o"), output};
  arguments << session_.converter().to_arguments(session_.effective_poses_path());
  ToolInvocation convert = ExternalToolRunner::ros2_run(QStringLiteral("pcd_to_nav_map"), kConverterPackage,
                                                        QStringLiteral("pcd_to_nav_map"), arguments);
  run_tool(convert, [this, output](const ToolResult &result) {
    if (!result.ok) return fail_queue(result.error_summary);
    ToolInvocation validate = ExternalToolRunner::ros2_run(
        QStringLiteral("validate_nav_map"), kConverterPackage, QStringLiteral("validate_nav_map"), {output});
    run_tool(validate, [this, output](const ToolResult &validate_result) {
      if (!validate_result.ok) return fail_queue(validate_result.error_summary);
      session_.mark_done(WorkflowSession::Navigation, output,
                         WorkflowSession::sha256_file(QDir(output).filePath(QStringLiteral("map.pgm"))),
                         QStringLiteral("pcd_to_nav_map + validate_nav_map"));
      QString map_error;
      if (!load_navigation_dir_into_2d(output, &map_error)) {
        statusBar()->showMessage(QStringLiteral("Layers generated but could not be displayed: %1").arg(map_error), 8000);
      } else if (!queue_running_) {
        show_2d_view();
      }
      save_session_quietly();
      continue_queue();
    });
  });
}

void MainWindow::run_patch() {
  QString error;
  if (!ensure_work_dir(&error)) return fail_queue(error);
  if (!refinement_model_.has_map() || refinement_model_.patch_edit_count() == 0) {
    statusBar()->showMessage(QStringLiteral("No 2D raster edits to patch"), 4000);
    return continue_queue();
  }
  if (session_.state(WorkflowSession::Navigation) != StageState::Fresh) {
    return fail_queue(QStringLiteral("Navigation layers are missing or stale: run step 3 first."));
  }
  QString missing;
  if (!tools_available({QStringLiteral("%1/patch_nav_map").arg(kConverterPackage)}, &missing)) {
    return fail_queue(QStringLiteral("Missing tool: %1").arg(missing));
  }
  std::string write_error;
  if (!refinement_model_.write_navigation_patch(session_.navigation_patch_path().toStdString(), &write_error) ||
      !refinement_model_.write_keepout_zones(session_.keepout_zones_path().toStdString(), &write_error)) {
    return fail_queue(QString::fromStdString(write_error));
  }
  const QString base = session_.record(WorkflowSession::Navigation).path;
  const QString output = QDir(session_.work_dir()).filePath(QStringLiteral("navigation_patched_%1").arg(stamp()));
  ToolInvocation invocation = ExternalToolRunner::ros2_run(
      QStringLiteral("patch_nav_map"), kConverterPackage, QStringLiteral("patch_nav_map"),
      {base, session_.navigation_patch_path(), QStringLiteral("--output"), output});
  run_tool(invocation, [this, output](const ToolResult &result) {
    if (!result.ok) return fail_queue(result.error_summary);
    session_.mark_done(WorkflowSession::Patch, output,
                       WorkflowSession::sha256_file(QDir(output).filePath(QStringLiteral("map.pgm"))),
                       QStringLiteral("patch_nav_map"));
    save_session_quietly();
    continue_queue();
  });
}

bool MainWindow::write_pipeline_config(QString *error) const {
  try {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "generator" << YAML::Value << "agt_map_studio";
    out << YAML::Key << "created_at" << YAML::Value << WorkflowSession::now_iso8601().toStdString();
    out << YAML::Key << "source" << YAML::Value << YAML::BeginMap
        << YAML::Key << "pcd" << YAML::Value << session_.source_pcd().toStdString()
        << YAML::Key << "pcd_sha256" << YAML::Value << session_.source_pcd_sha256().toStdString()
        << YAML::Key << "mapping_package" << YAML::Value << session_.source_package_dir().toStdString()
        << YAML::EndMap;
    out << YAML::Key << "refinement" << YAML::Value << YAML::BeginMap
        << YAML::Key << "applied" << YAML::Value << session_.has_3d_edits()
        << YAML::Key << "rules" << YAML::Value
        << (session_.has_3d_edits() ? session_.refinement_rules_path().toStdString() : std::string())
        << YAML::Key << "refined_package" << YAML::Value << session_.record(WorkflowSession::Refine).path.toStdString()
        << YAML::Key << "effective_pcd" << YAML::Value << session_.effective_pcd().toStdString()
        << YAML::Key << "effective_pcd_sha256" << YAML::Value << session_.effective_pcd_sha256().toStdString()
        << YAML::EndMap;
    out << YAML::Key << "converter" << YAML::Value << YAML::BeginMap
        << YAML::Key << "tool" << YAML::Value << "agt_map_converter/pcd_to_nav_map"
        << YAML::Key << "arguments" << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const QString &argument : session_.converter().to_arguments(session_.effective_poses_path())) {
      out << argument.toStdString();
    }
    out << YAML::EndSeq << YAML::Key << "output" << YAML::Value
        << session_.record(WorkflowSession::Navigation).path.toStdString() << YAML::EndMap;
    out << YAML::Key << "manual_patch" << YAML::Value << YAML::BeginMap
        << YAML::Key << "applied" << YAML::Value << session_.has_2d_edits()
        << YAML::Key << "patch_yaml" << YAML::Value
        << (session_.has_2d_edits() ? session_.navigation_patch_path().toStdString() : std::string())
        << YAML::Key << "keepout_zones" << YAML::Value
        << (refinement_model_.forbidden_zones().empty() ? std::string() : session_.keepout_zones_path().toStdString())
        << YAML::Key << "output" << YAML::Value << session_.record(WorkflowSession::Patch).path.toStdString()
        << YAML::EndMap;
    out << YAML::Key << "relocalization" << YAML::Value << YAML::BeginMap
        << YAML::Key << "tool" << YAML::Value << "agt_global_relocalization_native/build_relocalization_assets"
        << YAML::Key << "output" << YAML::Value << session_.record(WorkflowSession::Relocalization).path.toStdString()
        << YAML::EndMap;
    out << YAML::EndMap;
    std::ofstream stream(session_.pipeline_config_path().toStdString());
    stream << out.c_str() << '\n';
    if (!stream.good()) {
      if (error) *error = QStringLiteral("cannot write %1").arg(session_.pipeline_config_path());
      return false;
    }
  } catch (const std::exception &exception) {
    if (error) *error = QString::fromUtf8(exception.what());
    return false;
  }
  return true;
}

void MainWindow::run_publish() {
  QString error;
  if (!ensure_work_dir(&error)) return fail_queue(error);
  workflow_panel_->read_publish_target(&session_.publish_target());
  const QStringList reasons = session_.blocking_reasons_for_publish();
  if (!reasons.isEmpty()) return fail_queue(QStringLiteral("Cannot publish:\n- %1").arg(reasons.join(QStringLiteral("\n- "))));
  QString missing;
  if (!tools_available({QStringLiteral("%1/create_map_package").arg(kManagerPackage)}, &missing)) {
    return fail_queue(QStringLiteral("Missing tool: %1").arg(missing));
  }
  const PublishTarget &target = session_.publish_target();
  const QString destination = QDir(target.map_root).filePath(target.map_id + '/' + target.map_version);
  if (QFileInfo::exists(destination)) {
    return fail_queue(QStringLiteral("%1 already exists. Map package versions are immutable; choose a new version.")
                          .arg(destination));
  }
  if (!write_pipeline_config(&error)) return fail_queue(error);
  const QString summary = QStringLiteral(
      "Publish map package\n\n  root:      %1\n  map_id:    %2\n  version:   %3\n  source:    %4\n"
      "  nav dir:   %5\n  reloc dir: %6\n  activate:  %7\n\nContinue?")
      .arg(target.map_root, target.map_id, target.map_version, session_.effective_pcd(),
           session_.effective_navigation_dir(), session_.record(WorkflowSession::Relocalization).path,
           target.activate ? QStringLiteral("yes") : QStringLiteral("no"));
  if (!queue_running_ &&
      QMessageBox::question(this, QStringLiteral("Publish"), summary, QMessageBox::Yes | QMessageBox::Cancel) != QMessageBox::Yes) {
    return;
  }
  ToolInvocation invocation = ExternalToolRunner::ros2_run(
      QStringLiteral("create_map_package"), kManagerPackage, QStringLiteral("create_map_package"),
      {QStringLiteral("--map-root"), target.map_root,
       QStringLiteral("--map-id"), target.map_id,
       QStringLiteral("--map-version"), target.map_version,
       QStringLiteral("--source-pcd"), session_.effective_pcd(),
       QStringLiteral("--navigation-dir"), session_.effective_navigation_dir(),
       QStringLiteral("--relocalization-assets-dir"), session_.record(WorkflowSession::Relocalization).path,
       QStringLiteral("--generation-pipeline"), session_.pipeline_config_path()});
  run_tool(invocation, [this, destination](const ToolResult &result) {
    if (!result.ok) return fail_queue(result.error_summary);
    session_.mark_done(WorkflowSession::Publish, destination, QString(), QStringLiteral("create_map_package"));
    save_session_quietly();
    const PublishTarget target = session_.publish_target();
    if (!target.activate) {
      statusBar()->showMessage(QStringLiteral("Published %1").arg(destination), 10000);
      return continue_queue();
    }
    ToolInvocation select = ExternalToolRunner::ros2_run(
        QStringLiteral("select_map_package"), kManagerPackage, QStringLiteral("select_map_package"),
        {QStringLiteral("--map-root"), target.map_root, QStringLiteral("--map-id"), target.map_id,
         QStringLiteral("--map-version"), target.map_version});
    run_tool(select, [this, destination](const ToolResult &select_result) {
      if (!select_result.ok) return fail_queue(select_result.error_summary);
      statusBar()->showMessage(QStringLiteral("Published and activated %1").arg(destination), 10000);
      continue_queue();
    });
  });
}

// ---------------------------------------------------------------------------
// Misc

void MainWindow::toggle_axis(bool checked) { viewer_->set_show_axis(checked); }
void MainWindow::toggle_background(bool checked) { viewer_->set_dark_background(checked); }

void MainWindow::show_controls() {
  QMessageBox::information(
      this, QStringLiteral("AGT Map Studio Controls"),
      QStringLiteral(
          "3D 视图\n"
          "  左键拖动：旋转  右键拖动：平移  滚轮：缩放\n"
          "  W/A/S/D/Q/E：移动相机（Shift 加速）  R：重置  0/1/2：等轴/前视/顶视\n"
          "  N：Navigate  B：Select  X：Delete（模式键不再占用 S/D）\n"
          "  工具：矩形拖选 / 多边形（单击加点，双击或 Enter 闭合，Esc 取消）/ 球体（单击）\n"
          "  Z window：把矩形/多边形选择限制在高度带内；Ctrl+H：按高度带整体选择\n"
          "  Delete：删除选中点（仅 Delete 模式）  Ctrl+I：反选  H：隐藏已删除  I：只看选中\n"
          "  Ctrl+Z / Ctrl+Y：撤销 / 重做\n\n"
          "2D 视图 (Ctrl+2)\n"
          "  左键拖动平移（View 模式）  滚轮缩放  F：适配  R：重置\n"
          "  Erase rect：拖框 占用->空闲   Obstacle line：拖线 空闲->占用\n"
          "  Free/Occupied/Unknown polygon：单击加点，双击/Enter 闭合，Backspace 撤一点\n"
          "  Forbidden zone：禁行多边形（导出 keepout_zones.yaml，不改栅格）\n\n"
          "所有 3D 删除会写成 refinement.yaml 规则，所有 2D 栅格编辑会写成 patch_nav_map 的 polygon_m 补丁；\n"
          "右侧 Publish Workflow 面板按 1→5 顺序执行并标记过期（STALE）的产物。"));
}

void MainWindow::show_workflow_help() {
  QMessageBox::information(
      this, QStringLiteral("Publish Workflow"),
      QStringLiteral(
          "1. 3D refinement   apply_map_refinement（建图包）或 studio 导出（裸 PCD）→ 二进制 map.pcd\n"
          "2. Relocalization  build_relocalization_assets --map <有效PCD>\n"
          "3. Navigation      pcd_to_nav_map + validate_nav_map（agt_navigation_v3 的正式转换器）\n"
          "4. 2D patch        patch_nav_map <navigation> navigation_patch.yaml --output ...\n"
          "5. Publish         create_map_package --map-root/--map-id/--map-version ...（可选 select_map_package 激活）\n\n"
          "规则：\n"
          "  • 源建图包永不被修改；所有产物写入 <源>_studio/ 工作目录并带时间戳。\n"
          "  • 修改 3D 删除后，步骤 1-5 变为 STALE；修改 2D 编辑后，步骤 4-5 变为 STALE。\n"
          "  • 发布目标 maps/<map_id>/<version> 已存在时拒绝发布（版本不可变）。\n"
          "  • 工具日志追加到 <工作目录>/studio_tools.log；会话保存在 studio_session.yaml。\n"
          "  • 需要先 source ROS 2 与工作区 overlay 再启动 Studio，才能找到上述 ros2 run 可执行文件。"));
}

void MainWindow::show_stats(const QString &text) { statusBar()->showMessage(text); }

void MainWindow::closeEvent(QCloseEvent *event) {
  if (confidence_review_runner_.is_running()) {
    QMessageBox::warning(this, QStringLiteral("Core review running"),
        QStringLiteral("Wait for the core to finish its staged derivative rebuild before "
                       "closing Studio. The source package and derivative remain unchanged."));
    event->ignore();
    return;
  }
  if (tool_runner_.is_running()) {
    if (QMessageBox::question(this, QStringLiteral("Tool running"),
                              QStringLiteral("An external tool is still running. Cancel it and quit?"),
                              QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
      event->ignore();
      return;
    }
    cancel_tool();
  }
  if (confidence_editor_.dirty() &&
      QMessageBox::warning(this, QStringLiteral("DIRTY spatial overrides"),
          QStringLiteral("Unsaved manual confidence override intent would be lost. Discard edits? "
                         "The source map and verified derivative are unchanged."),
          QMessageBox::Discard | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Discard) {
    event->ignore();
    return;
  }
  const bool unapplied = (session_.has_3d_edits() && session_.state(WorkflowSession::Refine) != StageState::Fresh) ||
                         (session_.has_2d_edits() && session_.state(WorkflowSession::Patch) != StageState::Fresh);
  if (unapplied) {
    const auto answer = QMessageBox::question(
        this, QStringLiteral("Unapplied edits"),
        QStringLiteral("There are edits that were not applied/published. Export them (refinement.yaml / "
                       "navigation_patch.yaml) before quitting?"),
        QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    if (answer == QMessageBox::Cancel) {
      event->ignore();
      return;
    }
    if (answer == QMessageBox::Yes) {
      QString error;
      if (ensure_work_dir(&error)) {
        if (selection_manager_.has_active_deletes()) {
          selection_manager_.write_refinement_rules(session_.refinement_rules_path(), session_.source_pcd(), &error);
        }
        std::string patch_error;
        if (refinement_model_.has_map()) {
          refinement_model_.write_navigation_patch(session_.navigation_patch_path().toStdString(), &patch_error);
          refinement_model_.write_keepout_zones(session_.keepout_zones_path().toStdString(), &patch_error);
        }
        session_.save(&error);
      }
    }
  } else if (!session_.empty()) {
    save_session_quietly();
  }
  QMainWindow::closeEvent(event);
}

}  // namespace agt_map_studio
