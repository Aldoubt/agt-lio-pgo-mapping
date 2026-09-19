#pragma once

#include "viewer/PointCloudViewer.hpp"
#include "selection/SelectionManager.h"
#include "occupancy/GridMap.hpp"
#include "occupancy/OccupancyViewer.hpp"
#include "occupancy/RefinementModel.hpp"

#include <QMainWindow>
#include <QStackedWidget>
#include <QString>

class QAction;
class QToolBar;

namespace agt_map_studio {

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MainWindow(const QString &config_path, QWidget *parent = nullptr);

  bool open_pcd(const QString &path, QString *error = nullptr);
  bool open_occupancy_map(const QString &path, QString *error = nullptr);

private slots:
  void open_pcd_dialog();
  void open_occupancy_map_dialog();
  void save_view_dialog();
  void export_clean_map_dialog();
  void generate_occupancy_map_dialog();
  void save_refinement_dialog();
  void export_navigation_map_dialog();
  void reset_camera();
  void undo_edit();
  void redo_edit();
  void set_mode_navigate();
  void set_mode_select();
  void set_mode_delete();
  void set_isometric_view();
  void set_front_view();
  void set_top_view();
  void show_3d_view();
  void show_2d_view();
  void set_occupancy_view_mode();
  void set_occupancy_erase_mode();
  void set_occupancy_obstacle_mode();
  void set_occupancy_forbidden_mode();
  void toggle_axis(bool checked);
  void toggle_background(bool checked);
  void toggle_height_coloring(bool checked);
  void show_controls();
  void show_stats(const QString &text);

private:
  void create_actions();
  void create_menus();
  void load_config(const QString &path);
  void apply_erase_rectangle(double min_x, double min_y, double max_x,
                             double max_y);
  void apply_obstacle_line(double start_x, double start_y, double end_x,
                           double end_y, double width_m);
  void apply_forbidden_polygon(const QVector<QPointF> &polygon);
  void refresh_occupancy_view();

  PointCloudViewer *viewer_ = nullptr;
  OccupancyViewer *occupancy_viewer_ = nullptr;
  QStackedWidget *view_stack_ = nullptr;
  QToolBar *occupancy_toolbar_ = nullptr;
  RefinementModel refinement_model_;
  SelectionManager selection_manager_;
  QString source_path_;
  QAction *show_axis_action_ = nullptr;
  QAction *dark_background_action_ = nullptr;
  QAction *height_coloring_action_ = nullptr;
};

}  // namespace agt_map_studio
