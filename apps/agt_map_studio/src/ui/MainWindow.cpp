#include "ui/MainWindow.hpp"

#include "io/PCDLoader.hpp"
#include "occupancy/MapYamlLoader.hpp"
#include "occupancy/commands/DrawObstacleCommand.hpp"
#include "occupancy/commands/EraseRectangleCommand.hpp"
#include "occupancy/commands/FillPolygonCommand.hpp"
#include "occupancy/commands/ForbiddenPolygonCommand.hpp"
#include "ui/WorkflowPanel.hpp"

#include <agt_pcd2grid_exporter/OccupancyGridWriter.hpp>
#include <agt_pcd2grid_exporter/PCDProjector.hpp>
#include <agt_pcd2grid_exporter/ParameterLoader.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <QAction>
#include <QActionGroup>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDesktopServices>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QDockWidget>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QImage>
#include <QLabel>
#include <QMenuBar>
#include <QMessageBox>
#include <QPixmap>
#include <QProcessEnvironment>
#include <QStatusBar>
#include <QToolBar>
#include <QUrl>
#include <QVBoxLayout>

#include <yaml-cpp/yaml.h>

#include <exception>
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
  occupancy_viewer_->set_refinement_model(&refinement_model_);
  default_map_root_ = QProcessEnvironment::systemEnvironment().value(
      QStringLiteral("AGT_MAP_ROOT"), QDir::home().filePath(QStringLiteral("ros2_ws/maps")));
  create_actions();
  create_workflow_dock();
  load_config(config_path);
  session_.publish_target().map_root = default_map_root_;
  workflow_panel_->set_publish_target(session_.publish_target());

  connect(viewer_, &PointCloudViewer::stats_changed, this, &MainWindow::show_stats);
  connect(viewer_, &PointCloudViewer::delete_requested_outside_delete_mode, this, [this]() {
    statusBar()->showMessage(
        QStringLiteral("Switch to Delete mode (toolbar or X) before pressing Delete"), 4000);
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
  auto *delete_action = new QAction(QStringLiteral("Delete Selected Points"), this);
  delete_action->setShortcut(Qt::Key_Delete);
  connect(delete_action, &QAction::triggered, this, &MainWindow::delete_selected);
  auto *invert_action = new QAction(QStringLiteral("Invert Selection"), this);
  invert_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_I));
  connect(invert_action, &QAction::triggered, this, [this]() {
    selection_manager_.invert_selection();
    viewer_->rebuild_selection_box_from_points();
  });
  auto *clear_selection_action = new QAction(QStringLiteral("Clear Selection"), this);
  connect(clear_selection_action, &QAction::triggered, this, [this]() {
    selection_manager_.clear_selection();
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
    selection_manager_.set_isolate_selected(checked);
    viewer_->mark_edit_state_dirty();
  });

  auto *edit_menu = menuBar()->addMenu(QStringLiteral("Edit"));
  edit_menu->addAction(undo_action);
  edit_menu->addAction(redo_action);
  edit_menu->addSeparator();
  edit_menu->addAction(delete_action);
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
  connect(height_coloring_action_, &QAction::toggled, this, &MainWindow::toggle_height_coloring);
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
  auto *show_3d_action = new QAction(QStringLiteral("3D Point Cloud"), this);
  auto *show_2d_action = new QAction(QStringLiteral("2D Navigation Map"), this);
  for (auto *action : {show_3d_action, show_2d_action}) {
    action->setCheckable(true);
    view_group->addAction(action);
  }
  show_3d_action->setChecked(true);
  show_3d_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_1));
  show_2d_action->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_2));
  connect(show_3d_action, &QAction::triggered, this, &MainWindow::show_3d_view);
  connect(show_2d_action, &QAction::triggered, this, &MainWindow::show_2d_view);
  view_menu->addAction(show_3d_action);
  view_menu->addAction(show_2d_action);
  view_menu->addSeparator();
  view_menu->addAction(reset_action);
  view_menu->addAction(isometric_action);
  view_menu->addAction(front_action);
  view_menu->addAction(top_action);
  view_menu->addSeparator();
  view_menu->addAction(show_axis_action_);
  view_menu->addAction(dark_background_action_);
  view_menu->addAction(height_coloring_action_);
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
  toolbar_3d_->addAction(delete_action);

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
      height_coloring_action_->setChecked(color_mode == "height");
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
  if (!viewer_->has_cloud()) return;
  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("Select height band"));
  auto *form = new QFormLayout(&dialog);
  auto *min_spin = new QDoubleSpinBox(&dialog);
  auto *max_spin = new QDoubleSpinBox(&dialog);
  for (auto *spin : {min_spin, max_spin}) {
    spin->setRange(-1000.0, 1000.0);
    spin->setDecimals(2);
  }
  min_spin->setValue(viewer_->cloud().min_bound.z());
  max_spin->setValue(viewer_->cloud().min_bound.z() + 0.2);
  form->addRow(QStringLiteral("Z min (m)"), min_spin);
  form->addRow(QStringLiteral("Z max (m)"), max_spin);
  auto *note = new QLabel(QStringLiteral("Selects every visible point with min <= z <= max\n"
                                         "(e.g. ceiling band or ground noise). Then press Delete in Delete mode."), &dialog);
  form->addRow(note);
  auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
  form->addRow(buttons);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  if (dialog.exec() != QDialog::Accepted) return;
  if (viewer_->mode() == InteractionMode::Navigate) set_mode_select();
  viewer_->select_height_band(min_spin->value(), max_spin->value());
  statusBar()->showMessage(QStringLiteral("Height band selected: %1 points")
                               .arg(static_cast<qulonglong>(selection_manager_.selected_count())), 5000);
}

// ---------------------------------------------------------------------------
// Editing

void MainWindow::reset_camera() { viewer_->reset_camera(); }

void MainWindow::undo_edit() {
  if (view_stack_->currentWidget() == occupancy_viewer_) {
    if (refinement_model_.undo()) refresh_occupancy_view();
  } else if (selection_manager_.undo()) {
    viewer_->mark_edit_state_dirty();
    sync_edit_fingerprints();
  }
}

void MainWindow::redo_edit() {
  if (view_stack_->currentWidget() == occupancy_viewer_) {
    if (refinement_model_.redo()) refresh_occupancy_view();
  } else if (selection_manager_.redo()) {
    viewer_->mark_edit_state_dirty();
    sync_edit_fingerprints();
  }
}

void MainWindow::delete_selected() {
  if (view_stack_->currentWidget() != viewer_) return;
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
  viewer_->set_mode(InteractionMode::Delete);
  mode_delete_action_->setChecked(true);
}
void MainWindow::set_isometric_view() { viewer_->isometric_view(); }
void MainWindow::set_front_view() { viewer_->front_view(); }
void MainWindow::set_top_view() { viewer_->top_view(); }

void MainWindow::show_3d_view() {
  view_stack_->setCurrentWidget(viewer_);
  if (occupancy_toolbar_) occupancy_toolbar_->setVisible(false);
  if (toolbar_3d_) toolbar_3d_->setVisible(true);
  statusBar()->showMessage(viewer_->stats_text());
}

void MainWindow::show_2d_view() {
  view_stack_->setCurrentWidget(occupancy_viewer_);
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
  if (edit_state_label_) {
    QStringList parts;
    if (session_.has_3d_edits()) {
      parts << QStringLiteral("3D: %1").arg(session_.state(WorkflowSession::Refine) == StageState::Fresh
                                                 ? QStringLiteral("refined") : QStringLiteral("unapplied"));
    }
    if (session_.has_2d_edits()) {
      parts << QStringLiteral("2D: %1").arg(session_.state(WorkflowSession::Patch) == StageState::Fresh
                                                 ? QStringLiteral("patched") : QStringLiteral("unapplied"));
    }
    edit_state_label_->setText(parts.isEmpty() ? QStringLiteral("no pending edits") : parts.join(QStringLiteral(" | ")));
  }
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
  if (tool_runner_.is_running()) {
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
void MainWindow::toggle_height_coloring(bool checked) { viewer_->set_height_coloring(checked); }

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
  if (tool_runner_.is_running()) {
    if (QMessageBox::question(this, QStringLiteral("Tool running"),
                              QStringLiteral("An external tool is still running. Cancel it and quit?"),
                              QMessageBox::Yes | QMessageBox::No) != QMessageBox::Yes) {
      event->ignore();
      return;
    }
    cancel_tool();
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
