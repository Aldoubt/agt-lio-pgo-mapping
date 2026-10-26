#pragma once

#include "tools/ExternalToolRunner.hpp"

#include <QDialog>
#include <QFileSystemWatcher>
#include <QTimer>
#include <functional>

class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QPushButton;
class QSpinBox;
class QTreeWidget;

namespace agt_map_studio {

// Offline project/query/study authoring. The installed workflow CLI owns asset
// validation and classification; this widget owns only input and job state.
class StudyWorkflowDialog : public QDialog {
  Q_OBJECT
public:
  explicit StudyWorkflowDialog(QWidget *parent = nullptr);
  void set_inputs(const QString &source, const QString &blocks, const QString &topology,
                  const QString &native_localizer);
  void set_picked_point(double x, double y);
  bool load_project_file(const QString &path);
  QString source_path() const;
  bool busy() const { return runner_.is_running(); }
  QString active_study_path() const { return active_study_path_; }

signals:
  void pick_requested(const QString &source);
  void source_requested(const QString &source);
  void markers_requested(const QString &study, const QString &source,
                         const QString &classification, const QString &scene,
                         const QString &frames);
  void markers_cleared();
  void evidence_requested(const QString &directory);

protected:
  void closeEvent(QCloseEvent *event) override;

private:
  bool run(const QStringList &arguments, std::function<void()> completed = {},
           const QString &study_job = {});
  void error(const QString &message);
  void read_project();
  void read_queries();
  bool read_study();
  void validate_queries();
  void validate_study();
  void attach_asset(const QString &path, bool study);
  void create_query_set(bool sample_rows);
  void revise_query_set(bool freeze);
  void edit_query(const QString &operation);
  void create_study();
  void run_study();
  void export_study();
  void preview_markers();
  void poll_job();
  void watch_inputs();
  QString new_file(const QString &title, const QString &suggestion);
  QString selected_query_id() const;

  ExternalToolRunner runner_;
  QTimer poll_timer_;
  QFileSystemWatcher watcher_;
  std::function<void()> completion_;
  QString active_study_path_;
  QString active_command_;
  bool cancel_requested_ = false;
  QString validated_study_path_;
  QString pending_input_;
  QWidget *controls_ = nullptr;
  QLineEdit *project_ = nullptr;
  QLineEdit *source_ = nullptr;
  QLineEdit *blocks_ = nullptr;
  QLineEdit *topology_ = nullptr;
  QLineEdit *query_set_ = nullptr;
  QLineEdit *query_id_ = nullptr;
  QLineEdit *study_ = nullptr;
  QLineEdit *study_id_ = nullptr;
  QLineEdit *ros_install_ = nullptr;
  QLineEdit *native_ = nullptr;
  QComboBox *project_assets_ = nullptr;
  QComboBox *query_scene_ = nullptr;
  QComboBox *sample_ratios_ = nullptr;
  QComboBox *frames_ = nullptr;
  QComboBox *class_filter_ = nullptr;
  QComboBox *scene_filter_ = nullptr;
  QComboBox *frame_filter_ = nullptr;
  QDoubleSpinBox *query_x_ = nullptr;
  QDoubleSpinBox *query_y_ = nullptr;
  QSpinBox *top_k_ = nullptr;
  QSpinBox *cancel_grace_ = nullptr;
  QTreeWidget *queries_ = nullptr;
  QTreeWidget *results_ = nullptr;
  QLabel *status_ = nullptr;
  QLabel *summary_ = nullptr;
  QPlainTextEdit *log_ = nullptr;
  QPushButton *cancel_ = nullptr;
};

}  // namespace agt_map_studio
