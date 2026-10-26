#pragma once

#include "tools/ExternalToolRunner.hpp"

#include <QDialog>
#include <QJsonObject>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFileSystemWatcher;
class QGraphicsScene;
class QGraphicsView;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QSpinBox;

namespace agt_map_studio {

class CoverageAuthoringDialog : public QDialog {
  Q_OBJECT
public:
  explicit CoverageAuthoringDialog(const QString &map_yaml,
                                   const QJsonObject &annotations,
                                   const QString &annotation_file,
                                   QWidget *parent = nullptr);
  bool load_request_file(const QString &path, QString *error = nullptr);

protected:
  void closeEvent(QCloseEvent *event) override;
  void reject() override;

private:
  void load_request();
  void export_annotations();
  void export_route_draft();
  void run(const QString &operation);
  void mark_stale();
  void watch_sources(const QString &request_path);
  void load_result();
  void draw_preview(const QJsonObject &result);
  void set_running(bool running);

  ExternalToolRunner runner_;
  QFileSystemWatcher *watcher_ = nullptr;
  QLineEdit *map_ = nullptr;
  QLineEdit *profile_ = nullptr;
  QLineEdit *structure_ = nullptr;
  QLineEdit *output_ = nullptr;
  QComboBox *mode_ = nullptr;
  QComboBox *backend_ = nullptr;
  QSpinBox *ros_domain_ = nullptr;
  QDoubleSpinBox *width_ = nullptr;
  QDoubleSpinBox *clearance_ = nullptr;
  QDoubleSpinBox *start_[3] = {};
  QDoubleSpinBox *end_[3] = {};
  QCheckBox *return_start_ = nullptr;
  QCheckBox *reverse_ = nullptr;
  QCheckBox *show_clearance_ = nullptr;
  QPushButton *analyze_ = nullptr;
  QPushButton *plan_ = nullptr;
  QPushButton *cancel_ = nullptr;
  QPushButton *load_ = nullptr;
  QPushButton *export_ = nullptr;
  QPushButton *export_route_ = nullptr;
  QLabel *status_ = nullptr;
  QProgressBar *progress_ = nullptr;
  QPlainTextEdit *report_ = nullptr;
  QPlainTextEdit *log_ = nullptr;
  QGraphicsScene *scene_ = nullptr;
  QGraphicsView *view_ = nullptr;
  QJsonObject request_defaults_;
  QJsonObject annotations_;
  QJsonObject current_result_;
  QString annotation_file_;
  QString active_output_;
  QString active_operation_;
  QString request_path_;
  QString export_output_;
  bool plan_ready_ = false;
  bool stale_ = false;
  bool canceled_ = false;
};

}  // namespace agt_map_studio
