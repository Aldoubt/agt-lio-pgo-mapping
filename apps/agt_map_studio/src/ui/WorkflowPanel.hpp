#pragma once

#include "workflow/WorkflowSession.hpp"

#include <QWidget>

#include <array>

class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPlainTextEdit;
class QProgressBar;
class QPushButton;
class QSpinBox;

namespace agt_map_studio {

// Right-hand dock: the five-step publish workflow with freshness badges,
// converter parameters, publish target and a tool log.
class WorkflowPanel : public QWidget {
  Q_OBJECT

public:
  explicit WorkflowPanel(QWidget *parent = nullptr);

  void refresh(const WorkflowSession &session, bool tool_running);
  void append_log(const QString &text);
  void clear_log();
  void set_progress(const QString &label, bool busy);
  void read_converter(ConverterParameters *parameters) const;
  void read_publish_target(PublishTarget *target) const;
  void set_publish_target(const PublishTarget &target);
  void set_converter(const ConverterParameters &parameters);

signals:
  void refine_requested();
  void relocalization_requested();
  void navigation_requested();
  void patch_requested();
  void publish_requested();
  void run_all_requested();
  void cancel_requested();
  void open_output_requested(const QString &path);
  void parameters_changed();

private:
  struct StepRow {
    QLabel *title = nullptr;
    QLabel *badge = nullptr;
    QLabel *detail = nullptr;
    QPushButton *run = nullptr;
    QPushButton *open = nullptr;
  };

  StepRow make_step(int number, const QString &title, const QString &hint, QWidget *host);
  static void paint_badge(QLabel *badge, StageState state, bool applicable);

  std::array<StepRow, 5> steps_{};
  QLabel *source_label_ = nullptr;
  QDoubleSpinBox *resolution_ = nullptr;
  QDoubleSpinBox *margin_ = nullptr;
  QSpinBox *min_points_ = nullptr;
  QDoubleSpinBox *max_step_ = nullptr;
  QDoubleSpinBox *max_slope_ = nullptr;
  QCheckBox *use_trajectory_ = nullptr;
  QLineEdit *map_root_ = nullptr;
  QLineEdit *map_id_ = nullptr;
  QLineEdit *map_version_ = nullptr;
  QCheckBox *activate_ = nullptr;
  QPushButton *run_all_ = nullptr;
  QPushButton *cancel_ = nullptr;
  QProgressBar *progress_ = nullptr;
  QLabel *progress_label_ = nullptr;
  QPlainTextEdit *log_ = nullptr;
};

}  // namespace agt_map_studio
