#include "ui/MainWindow.hpp"

#include "io/PCDLoader.hpp"
#include "occupancy/MapYamlLoader.hpp"
#include "occupancy/commands/DrawObstacleCommand.hpp"
#include "occupancy/commands/EraseRectangleCommand.hpp"
#include "occupancy/commands/ForbiddenPolygonCommand.hpp"

#include <agt_pcd2grid_exporter/OccupancyGridWriter.hpp>
#include <agt_pcd2grid_exporter/PCDProjector.hpp>
#include <agt_pcd2grid_exporter/ParameterLoader.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <QAction>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QMenuBar>
#include <QToolBar>
#include <QActionGroup>
#include <QDir>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QStatusBar>
#include <QVBoxLayout>

#include <yaml-cpp/yaml.h>

#include <exception>
#include <limits>
#include <utility>

namespace agt_map_studio {

MainWindow::MainWindow(const QString &config_path, QWidget *parent)
    : QMainWindow(parent), viewer_(new PointCloudViewer(this)),
      occupancy_viewer_(new OccupancyViewer(this)),
      view_stack_(new QStackedWidget(this)) {
  setWindowTitle(QStringLiteral("AGT Map Studio"));
  resize(1280, 800);
  view_stack_->addWidget(viewer_);
  view_stack_->addWidget(occupancy_viewer_);
  view_stack_->setCurrentWidget(viewer_);
  setCentralWidget(view_stack_);
  viewer_->set_selection_manager(&selection_manager_);
  occupancy_viewer_->set_refinement_model(&refinement_model_);
  create_actions();
  create_menus();
  load_config(config_path);
  connect(viewer_, &PointCloudViewer::stats_changed, this,
          &MainWindow::show_stats);
  connect(occupancy_viewer_, &OccupancyViewer::status_changed, this,
          &MainWindow::show_stats);
  connect(occupancy_viewer_, &OccupancyViewer::erase_rectangle_requested, this,
          &MainWindow::apply_erase_rectangle);
  connect(occupancy_viewer_, &OccupancyViewer::obstacle_line_requested, this,
          &MainWindow::apply_obstacle_line);
  connect(occupancy_viewer_, &OccupancyViewer::forbidden_polygon_requested, this,
          &MainWindow::apply_forbidden_polygon);
  statusBar()->showMessage(viewer_->stats_text());
}

void MainWindow::create_actions() {
  auto *open_action = new QAction(QStringLiteral("Open PCD"), this);
  open_action->setShortcut(QKeySequence::Open);
  connect(open_action, &QAction::triggered, this, &MainWindow::open_pcd_dialog);

  auto *open_occupancy_action = new QAction(QStringLiteral("Open Occupancy Map"), this);
  connect(open_occupancy_action, &QAction::triggered, this,
          &MainWindow::open_occupancy_map_dialog);

  auto *save_action = new QAction(QStringLiteral("Save View"), this);
  save_action->setShortcut(QKeySequence::Save);
  connect(save_action, &QAction::triggered, this, &MainWindow::save_view_dialog);

  auto *export_action = new QAction(QStringLiteral("Export Clean Map"), this);
  connect(export_action, &QAction::triggered, this,
          &MainWindow::export_clean_map_dialog);

  auto *save_refinement_action = new QAction(QStringLiteral("Save Map Refinement"), this);
  connect(save_refinement_action, &QAction::triggered, this,
          &MainWindow::save_refinement_dialog);
  auto *export_navigation_action = new QAction(QStringLiteral("Export Navigation Map"), this);
  connect(export_navigation_action, &QAction::triggered, this,
          &MainWindow::export_navigation_map_dialog);

  auto *reset_action = new QAction(QStringLiteral("Reset Camera"), this);
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

  auto *undo_action = new QAction(QStringLiteral("Undo Edit"), this);
  undo_action->setShortcut(QKeySequence::Undo);
  connect(undo_action, &QAction::triggered, this, &MainWindow::undo_edit);
  auto *redo_action = new QAction(QStringLiteral("Redo Edit"), this);
  redo_action->setShortcut(QKeySequence::Redo);
  connect(redo_action, &QAction::triggered, this, &MainWindow::redo_edit);
  auto *delete_action = new QAction(QStringLiteral("Delete Selected"), this);
  delete_action->setShortcut(Qt::Key_Delete);
  connect(delete_action, &QAction::triggered, this, [this]() {
    if (viewer_->mode() != InteractionMode::Delete) {
      statusBar()->showMessage(QStringLiteral("Switch to Delete mode first"), 3000);
      return;
    }
    if (selection_manager_.delete_selected()) viewer_->mark_edit_state_dirty();
  });

  show_axis_action_ = new QAction(QStringLiteral("Show Axis"), this);
  show_axis_action_->setCheckable(true);
  show_axis_action_->setChecked(true);
  connect(show_axis_action_, &QAction::toggled, this, &MainWindow::toggle_axis);

  dark_background_action_ = new QAction(QStringLiteral("Dark Background"), this);
  dark_background_action_->setCheckable(true);
  connect(dark_background_action_, &QAction::toggled, this,
          &MainWindow::toggle_background);

  height_coloring_action_ = new QAction(QStringLiteral("Color by Z Height"), this);
  height_coloring_action_->setCheckable(true);
  height_coloring_action_->setChecked(true);
  connect(height_coloring_action_, &QAction::toggled, this,
          &MainWindow::toggle_height_coloring);

  auto *increase_point_action = new QAction(QStringLiteral("Increase Point Size"), this);
  connect(increase_point_action, &QAction::triggered, this,
          [this]() { viewer_->adjust_point_size(0.5F); });
  auto *decrease_point_action = new QAction(QStringLiteral("Decrease Point Size"), this);
  connect(decrease_point_action, &QAction::triggered, this,
          [this]() { viewer_->adjust_point_size(-0.5F); });
  auto *default_point_action = new QAction(QStringLiteral("Default Point Size"), this);
  connect(default_point_action, &QAction::triggered, this,
          [this]() { viewer_->set_point_size(2.0F); });

  auto *file_menu = menuBar()->addMenu(QStringLiteral("File"));
  file_menu->addAction(open_action);
  file_menu->addAction(open_occupancy_action);
  file_menu->addAction(save_action);
  file_menu->addSeparator();
  file_menu->addAction(export_action);
  file_menu->addAction(save_refinement_action);
  file_menu->addAction(export_navigation_action);

  auto *tools_menu = menuBar()->addMenu(QStringLiteral("Tools"));
  auto *occupancy_action = new QAction(QStringLiteral("Generate Occupancy Map"), this);
  connect(occupancy_action, &QAction::triggered, this,
          &MainWindow::generate_occupancy_map_dialog);
  tools_menu->addAction(occupancy_action);

  auto *edit_menu = menuBar()->addMenu(QStringLiteral("Edit"));
  edit_menu->addAction(undo_action);
  edit_menu->addAction(redo_action);
  edit_menu->addSeparator();
  edit_menu->addAction(delete_action);

  auto *view_menu = menuBar()->addMenu(QStringLiteral("View"));
  auto *view_group = new QActionGroup(this);
  view_group->setExclusive(true);
  auto *show_3d_action = new QAction(QStringLiteral("3D Point Cloud"), this);
  auto *show_2d_action = new QAction(QStringLiteral("2D Occupancy Map"), this);
  show_3d_action->setCheckable(true);
  show_2d_action->setCheckable(true);
  show_3d_action->setChecked(true);
  view_group->addAction(show_3d_action);
  view_group->addAction(show_2d_action);
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
  view_menu->addAction(show_axis_action_);
  view_menu->addAction(dark_background_action_);
  view_menu->addAction(height_coloring_action_);
  auto *point_menu = view_menu->addMenu(QStringLiteral("Point Size"));
  point_menu->addAction(increase_point_action);
  point_menu->addAction(decrease_point_action);
  point_menu->addAction(default_point_action);

  auto *help_menu = menuBar()->addMenu(QStringLiteral("Help"));
  auto *controls_action = new QAction(QStringLiteral("Controls"), this);
  connect(controls_action, &QAction::triggered, this, &MainWindow::show_controls);
  help_menu->addAction(controls_action);

  auto *toolbar = addToolBar(QStringLiteral("Edit Mode"));
  toolbar->setMovable(false);
  auto *mode_group = new QActionGroup(this);
  mode_group->setExclusive(true);
  auto *navigate_action = toolbar->addAction(QStringLiteral("Navigate"));
  auto *select_action = toolbar->addAction(QStringLiteral("Select"));
  auto *mode_delete_action = toolbar->addAction(QStringLiteral("Delete"));
  for (auto *action : {navigate_action, select_action, mode_delete_action}) {
    action->setCheckable(true);
    mode_group->addAction(action);
  }
  navigate_action->setChecked(true);
  connect(navigate_action, &QAction::triggered, this, &MainWindow::set_mode_navigate);
  connect(select_action, &QAction::triggered, this, &MainWindow::set_mode_select);
  connect(mode_delete_action, &QAction::triggered, this, &MainWindow::set_mode_delete);

  occupancy_toolbar_ = addToolBar(QStringLiteral("Occupancy Edit Mode"));
  occupancy_toolbar_->setMovable(false);
  auto *occupancy_group = new QActionGroup(this);
  occupancy_group->setExclusive(true);
  auto *occupancy_view_action = occupancy_toolbar_->addAction(QStringLiteral("View"));
  auto *occupancy_erase_action = occupancy_toolbar_->addAction(QStringLiteral("Erase"));
  auto *occupancy_obstacle_action = occupancy_toolbar_->addAction(QStringLiteral("Obstacle"));
  auto *occupancy_forbidden_action = occupancy_toolbar_->addAction(QStringLiteral("Forbidden"));
  for (auto *action : {occupancy_view_action, occupancy_erase_action,
                       occupancy_obstacle_action, occupancy_forbidden_action}) {
    action->setCheckable(true);
    occupancy_group->addAction(action);
  }
  occupancy_view_action->setChecked(true);
  connect(occupancy_view_action, &QAction::triggered, this,
          &MainWindow::set_occupancy_view_mode);
  connect(occupancy_erase_action, &QAction::triggered, this,
          &MainWindow::set_occupancy_erase_mode);
  connect(occupancy_obstacle_action, &QAction::triggered, this,
          &MainWindow::set_occupancy_obstacle_mode);
  connect(occupancy_forbidden_action, &QAction::triggered, this,
          &MainWindow::set_occupancy_forbidden_mode);
  occupancy_toolbar_->addSeparator();
  occupancy_toolbar_->addWidget(new QLabel(QStringLiteral("Width (m):"), this));
  auto *width_spin = new QDoubleSpinBox(this);
  width_spin->setRange(0.01, 10.0);
  width_spin->setSingleStep(0.05);
  width_spin->setDecimals(2);
  width_spin->setValue(0.20);
  width_spin->setToolTip(QStringLiteral("Obstacle line width in meters"));
  occupancy_toolbar_->addWidget(width_spin);
  connect(width_spin, qOverload<double>(&QDoubleSpinBox::valueChanged),
          occupancy_viewer_, &OccupancyViewer::set_obstacle_width);
  occupancy_toolbar_->setVisible(false);
}

void MainWindow::create_menus() {}

void MainWindow::load_config(const QString &path) {
  if (path.isEmpty()) return;
  try {
    const YAML::Node root = YAML::LoadFile(path.toStdString());
    const YAML::Node camera = root["camera"];
    if (camera) {
      viewer_->set_camera_speeds(camera["speed"].as<float>(0.5F),
                                 camera["fast_speed"].as<float>(3.0F));
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
  } catch (const std::exception &exception) {
    statusBar()->showMessage(
        QStringLiteral("Config warning: %1").arg(exception.what()), 5000);
  }
}

bool MainWindow::open_pcd(const QString &path, QString *error) {
  LoadedPointCloud loaded;
  std::string loader_error;
  if (!PCDLoader::load(path.toStdString(), &loaded, &loader_error)) {
    const QString message = QString::fromStdString(loader_error);
    if (error) *error = message;
    return false;
  }
  selection_manager_.reset(loaded.point_count());
  source_path_ = path;
  show_3d_view();
  view_stack_->setCurrentWidget(viewer_);
  viewer_->set_cloud(std::move(loaded), QFileInfo(path).fileName());
  statusBar()->showMessage(viewer_->stats_text());
  return true;
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
  show_2d_view();
  view_stack_->setCurrentWidget(occupancy_viewer_);
  statusBar()->showMessage(QStringLiteral("Opened occupancy map: %1").arg(path));
  return true;
}

void MainWindow::export_clean_map_dialog() {
  if (!viewer_->has_cloud()) {
    QMessageBox::information(this, QStringLiteral("Export Clean Map"),
                             QStringLiteral("Open a PCD before exporting."));
    return;
  }
  const QString parent = QFileDialog::getExistingDirectory(
      this, QStringLiteral("Choose export parent directory"), source_path_.isEmpty()
          ? QString() : QFileInfo(source_path_).absolutePath());
  if (parent.isEmpty()) return;
  const QString output_dir = QDir(parent).filePath(QStringLiteral("clean_map"));
  QString error;
  if (!selection_manager_.export_clean_map(viewer_->cloud(), output_dir,
                                            source_path_, &error)) {
    QMessageBox::critical(this, QStringLiteral("Export Clean Map failed"), error);
    return;
  }
  statusBar()->showMessage(QStringLiteral("Exported clean map: %1").arg(output_dir), 8000);
}

void MainWindow::generate_occupancy_map_dialog() {
  if (!viewer_->has_cloud()) {
    QMessageBox::information(this, QStringLiteral("Generate Occupancy Map"),
                             QStringLiteral("Open a PCD before generating a map."));
    return;
  }

  agt_pcd2grid_exporter::ProjectionParameters parameters;
  QString config_path;
  try {
    const std::string share = ament_index_cpp::get_package_share_directory(
        "agt_pcd2grid_exporter");
    config_path = QString::fromStdString(share + "/config/projection.yaml");
  } catch (const std::exception &) {
  }
  if (!config_path.isEmpty() && QFileInfo::exists(config_path)) {
    std::string parameter_error;
    if (!agt_pcd2grid_exporter::ParameterLoader::load(
            config_path.toStdString(), &parameters, &parameter_error)) {
      QMessageBox::critical(this, QStringLiteral("Projection parameters"),
                            QString::fromStdString(parameter_error));
      return;
    }
  }
  const QString policy = parameters.empty_cell ==
                                 agt_pcd2grid_exporter::EmptyCellPolicy::Free
                             ? QStringLiteral("free")
                             : QStringLiteral("unknown");
  const QString summary = QStringLiteral(
      "Resolution: %1 m\nZ filter: [%2, %3] m\nOccupied threshold: %4 hits\nEmpty cells: %5\n\nGenerate a new navigation_map directory?")
                              .arg(parameters.resolution, 0, 'f', 3)
                              .arg(parameters.z_min, 0, 'f', 3)
                              .arg(parameters.z_max, 0, 'f', 3)
                              .arg(parameters.occupied_threshold)
                              .arg(policy);
  if (QMessageBox::question(this, QStringLiteral("PCD to Occupancy Grid"), summary,
                            QMessageBox::Yes | QMessageBox::Cancel) != QMessageBox::Yes) {
    return;
  }
  const QString parent = QFileDialog::getExistingDirectory(
      this, QStringLiteral("Choose navigation map output parent"), source_path_.isEmpty()
          ? QString() : QFileInfo(source_path_).absolutePath());
  if (parent.isEmpty()) return;
  const QString output_dir = QDir(parent).filePath(QStringLiteral("navigation_map"));

  agt_pcd2grid_exporter::OccupancyGrid grid;
  agt_pcd2grid_exporter::ProjectionStats stats;
  std::string error;
  if (!agt_pcd2grid_exporter::PCDProjector::project(
          *viewer_->cloud().source, parameters, &grid, &stats, &error) ||
      !agt_pcd2grid_exporter::OccupancyGridWriter::write_navigation_map(
          grid, parameters, stats, output_dir.toStdString(),
          source_path_.toStdString(), &error)) {
    QMessageBox::critical(this, QStringLiteral("Generate Occupancy Map failed"),
                          QString::fromStdString(error));
    return;
  }
  statusBar()->showMessage(QStringLiteral("Generated occupancy map: %1").arg(output_dir),
                           8000);

  if (grid.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
      grid.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
    return;
  }
  QImage preview(static_cast<int>(grid.width), static_cast<int>(grid.height),
                 QImage::Format_Grayscale8);
  for (std::uint32_t gy = 0; gy < grid.height; ++gy) {
    auto *line = preview.scanLine(static_cast<int>(grid.height - 1U - gy));
    for (std::uint32_t gx = 0; gx < grid.width; ++gx) {
      const auto value = grid.value(static_cast<std::size_t>(gy) * grid.width + gx,
                                    parameters);
      line[gx] = value == 100 ? 0U : (value == 0 ? 254U : 205U);
    }
  }
  QDialog dialog(this);
  dialog.setWindowTitle(QStringLiteral("Occupancy Preview - %1").arg(output_dir));
  auto *layout = new QVBoxLayout(&dialog);
  auto *label = new QLabel(&dialog);
  label->setAlignment(Qt::AlignCenter);
  label->setPixmap(QPixmap::fromImage(preview.scaled(
      1000, 700, Qt::KeepAspectRatio, Qt::FastTransformation)));
  layout->addWidget(label);
  dialog.resize(1020, 740);
  dialog.exec();
}

void MainWindow::open_pcd_dialog() {
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("Open PCD"), QString(),
      QStringLiteral("Point Cloud (*.pcd);;All Files (*)"));
  if (path.isEmpty()) return;
  QString error;
  if (!open_pcd(path, &error)) {
    QMessageBox::critical(this, QStringLiteral("Open PCD failed"), error);
  }
}

void MainWindow::open_occupancy_map_dialog() {
  const QString path = QFileDialog::getOpenFileName(
      this, QStringLiteral("Open Occupancy Map"), QString(),
      QStringLiteral("Nav2 map (*.yaml *.yml);;All Files (*)"));
  if (path.isEmpty()) return;
  QString error;
  if (!open_occupancy_map(path, &error)) {
    QMessageBox::critical(this, QStringLiteral("Open Occupancy Map failed"), error);
  }
}

void MainWindow::save_view_dialog() {
  const QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("Save View"), QStringLiteral("view.yaml"),
      QStringLiteral("YAML (*.yaml *.yml);;All Files (*)"));
  if (path.isEmpty()) return;
  QString error;
  if (!viewer_->save_view(path, &error)) {
    QMessageBox::critical(this, QStringLiteral("Save View failed"), error);
    return;
  }
  statusBar()->showMessage(QStringLiteral("Saved view: %1").arg(path), 5000);
}

void MainWindow::reset_camera() {
  viewer_->reset_camera();
}

void MainWindow::undo_edit() {
  if (view_stack_->currentWidget() == occupancy_viewer_) {
    if (refinement_model_.undo()) refresh_occupancy_view();
  } else if (selection_manager_.undo()) {
    viewer_->mark_edit_state_dirty();
  }
}

void MainWindow::redo_edit() {
  if (view_stack_->currentWidget() == occupancy_viewer_) {
    if (refinement_model_.redo()) refresh_occupancy_view();
  } else if (selection_manager_.redo()) {
    viewer_->mark_edit_state_dirty();
  }
}

void MainWindow::set_mode_navigate() { viewer_->set_mode(InteractionMode::Navigate); }
void MainWindow::set_mode_select() { viewer_->set_mode(InteractionMode::Select); }
void MainWindow::set_mode_delete() { viewer_->set_mode(InteractionMode::Delete); }
void MainWindow::set_isometric_view() { viewer_->isometric_view(); }
void MainWindow::set_front_view() { viewer_->front_view(); }
void MainWindow::set_top_view() { viewer_->top_view(); }

void MainWindow::show_3d_view() {
  view_stack_->setCurrentWidget(viewer_);
  if (occupancy_toolbar_) occupancy_toolbar_->setVisible(false);
  statusBar()->showMessage(viewer_->stats_text());
}

void MainWindow::show_2d_view() {
  view_stack_->setCurrentWidget(occupancy_viewer_);
  if (occupancy_toolbar_) occupancy_toolbar_->setVisible(true);
  statusBar()->showMessage(QStringLiteral("2D occupancy view"));
}

void MainWindow::set_occupancy_view_mode() {
  occupancy_viewer_->set_mode(OccupancyInteractionMode::View);
}

void MainWindow::set_occupancy_erase_mode() {
  occupancy_viewer_->set_mode(OccupancyInteractionMode::Erase);
}

void MainWindow::set_occupancy_obstacle_mode() {
  occupancy_viewer_->set_mode(OccupancyInteractionMode::Obstacle);
}

void MainWindow::set_occupancy_forbidden_mode() {
  occupancy_viewer_->set_mode(OccupancyInteractionMode::Forbidden);
}

void MainWindow::apply_erase_rectangle(double min_x, double min_y, double max_x,
                                       double max_y) {
  if (!refinement_model_.has_map()) return;
  if (QMessageBox::question(
          this, QStringLiteral("Erase Rectangle"),
          QStringLiteral("Convert occupied cells in this rectangle to free?"),
          QMessageBox::Yes | QMessageBox::Cancel) != QMessageBox::Yes) {
    return;
  }
  auto command = EraseRectangleCommand::create(
      refinement_model_, {min_x, min_y}, {max_x, max_y});
  if (!command) {
    statusBar()->showMessage(QStringLiteral("No occupied cells in selected rectangle"), 3000);
    return;
  }
  std::string error;
  if (!refinement_model_.execute(std::move(command), &error)) {
    QMessageBox::critical(this, QStringLiteral("Erase failed"),
                          QString::fromStdString(error));
    return;
  }
  refresh_occupancy_view();
}

void MainWindow::apply_obstacle_line(double start_x, double start_y, double end_x,
                                     double end_y, double width_m) {
  auto command = DrawObstacleCommand::create(
      refinement_model_, {start_x, start_y}, {end_x, end_y}, width_m);
  if (!command) {
    statusBar()->showMessage(QStringLiteral("No free cells in obstacle line"), 3000);
    return;
  }
  std::string error;
  if (!refinement_model_.execute(std::move(command), &error)) {
    QMessageBox::critical(this, QStringLiteral("Obstacle draw failed"),
                          QString::fromStdString(error));
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
    QMessageBox::critical(this, QStringLiteral("Forbidden zone failed"),
                          QString::fromStdString(error));
    return;
  }
  refresh_occupancy_view();
}

void MainWindow::refresh_occupancy_view() {
  occupancy_viewer_->refresh();
  statusBar()->showMessage(QStringLiteral("Occupancy edits: %1 overrides, %2 forbidden zones")
                               .arg(refinement_model_.active_override_count())
                               .arg(refinement_model_.forbidden_zones().size()));
}

void MainWindow::save_refinement_dialog() {
  if (!refinement_model_.has_map()) {
    QMessageBox::information(this, QStringLiteral("Save Map Refinement"),
                             QStringLiteral("Open an occupancy map first."));
    return;
  }
  const QString path = QFileDialog::getSaveFileName(
      this, QStringLiteral("Save Map Refinement"), QStringLiteral("map_refinement.yaml"),
      QStringLiteral("YAML (*.yaml *.yml);;All Files (*)"));
  if (path.isEmpty()) return;
  std::string error;
  if (!refinement_model_.save_refinement_yaml(path.toStdString(), &error)) {
    QMessageBox::critical(this, QStringLiteral("Save Refinement failed"),
                          QString::fromStdString(error));
    return;
  }
  statusBar()->showMessage(QStringLiteral("Saved refinement: %1").arg(path), 6000);
}

void MainWindow::export_navigation_map_dialog() {
  if (!refinement_model_.has_map()) {
    QMessageBox::information(this, QStringLiteral("Export Navigation Map"),
                             QStringLiteral("Open an occupancy map first."));
    return;
  }
  const QString parent = QFileDialog::getExistingDirectory(
      this, QStringLiteral("Choose navigation map output parent"),
      QFileInfo(QString::fromStdString(refinement_model_.base_map().yaml_path()))
          .absolutePath());
  if (parent.isEmpty()) return;
  const QString output = QDir(parent).filePath(QStringLiteral("navigation_map"));
  std::string error;
  if (!refinement_model_.export_navigation_map(output.toStdString(), &error)) {
    QMessageBox::critical(this, QStringLiteral("Export Navigation Map failed"),
                          QString::fromStdString(error));
    return;
  }
  statusBar()->showMessage(QStringLiteral("Exported navigation map: %1").arg(output),
                           8000);
}

void MainWindow::toggle_axis(bool checked) {
  viewer_->set_show_axis(checked);
}

void MainWindow::toggle_background(bool checked) {
  viewer_->set_dark_background(checked);
}

void MainWindow::toggle_height_coloring(bool checked) {
  viewer_->set_height_coloring(checked);
}

void MainWindow::show_controls() {
  QMessageBox::information(
      this, QStringLiteral("AGT Map Studio Controls"),
      QStringLiteral(
          "鼠标:\n"
          "  左键拖动：旋转相机\n"
          "  右键拖动：平移相机\n"
          "  滚轮：缩放\n\n"
          "键盘:\n"
          "  W/S：前进/后退\n"
          "  A/D：左移/右移\n"
          "  Q/E：下降/上升\n"
          "  Shift：加速\n"
          "  R：重置视角\n"
          "  0/1/2：等轴/前视/顶视\n"
          "  +/-：调整点大小\n\n"
          "  N：Navigate，S：Select，D：Delete\n"
          "  Delete：删除选中点（红色保留显示）\n"
          "  Ctrl+Z/Ctrl+Y：撤销/重做\n\n"
          "Select/Delete 模式下左键拖动框选 AABB。View 菜单可切换坐标轴、背景和 Z 高度着色。"));
}

void MainWindow::show_stats(const QString &text) {
  statusBar()->showMessage(text);
}

}  // namespace agt_map_studio
