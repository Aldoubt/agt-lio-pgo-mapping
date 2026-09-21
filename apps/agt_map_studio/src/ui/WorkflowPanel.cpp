#include "ui/WorkflowPanel.hpp"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QVBoxLayout>

namespace agt_map_studio {

WorkflowPanel::WorkflowPanel(QWidget *parent) : QWidget(parent) {
  auto *outer = new QVBoxLayout(this);
  outer->setContentsMargins(6, 6, 6, 6);

  auto *scroll = new QScrollArea(this);
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  auto *host = new QWidget(scroll);
  auto *layout = new QVBoxLayout(host);
  layout->setContentsMargins(0, 0, 0, 0);

  source_label_ = new QLabel(QStringLiteral("No source loaded"), host);
  source_label_->setWordWrap(true);
  source_label_->setStyleSheet(QStringLiteral("color: #555;"));
  layout->addWidget(source_label_);

  steps_[0] = make_step(1, QStringLiteral("3D refinement"),
                        QStringLiteral("Write refinement.yaml and a binary refined map.pcd "
                                       "(apply_map_refinement when the source is a mapping "
                                       "package, otherwise the studio clean-map export)."),
                        host);
  steps_[1] = make_step(2, QStringLiteral("Relocalization assets"),
                        QStringLiteral("build_relocalization_assets from the effective PCD."),
                        host);
  steps_[2] = make_step(3, QStringLiteral("Navigation layers"),
                        QStringLiteral("pcd_to_nav_map + validate_nav_map (agt_navigation_v3)."),
                        host);
  steps_[3] = make_step(4, QStringLiteral("2D patch"),
                        QStringLiteral("patch_nav_map with the polygon_m edits drawn in the 2D view."),
                        host);
  steps_[4] = make_step(5, QStringLiteral("Publish map package"),
                        QStringLiteral("create_map_package into maps/<map_id>/<version>; never "
                                       "overwrites an existing version."),
                        host);
  for (auto &row : steps_) layout->addWidget(row.title->parentWidget());

  auto *converter_box = new QGroupBox(QStringLiteral("Converter parameters (pcd_to_nav_map)"), host);
  auto *form = new QFormLayout(converter_box);
  resolution_ = new QDoubleSpinBox(converter_box);
  resolution_->setRange(0.02, 1.0);
  resolution_->setSingleStep(0.01);
  resolution_->setDecimals(3);
  resolution_->setValue(0.10);
  margin_ = new QDoubleSpinBox(converter_box);
  margin_->setRange(0.0, 20.0);
  margin_->setDecimals(2);
  margin_->setValue(1.0);
  min_points_ = new QSpinBox(converter_box);
  min_points_->setRange(1, 100);
  min_points_->setValue(2);
  max_step_ = new QDoubleSpinBox(converter_box);
  max_step_->setRange(0.01, 2.0);
  max_step_->setDecimals(3);
  max_step_->setValue(0.22);
  max_slope_ = new QDoubleSpinBox(converter_box);
  max_slope_->setRange(1.0, 89.0);
  max_slope_->setDecimals(1);
  max_slope_->setValue(20.0);
  use_trajectory_ = new QCheckBox(QStringLiteral("Use mapping trajectory (poses.txt) as drivable prior"), converter_box);
  use_trajectory_->setChecked(true);
  form->addRow(QStringLiteral("Resolution (m)"), resolution_);
  form->addRow(QStringLiteral("Margin (m)"), margin_);
  form->addRow(QStringLiteral("Min points / cell"), min_points_);
  form->addRow(QStringLiteral("Max step (m)"), max_step_);
  form->addRow(QStringLiteral("Max slope (deg)"), max_slope_);
  form->addRow(use_trajectory_);
  layout->addWidget(converter_box);
  for (auto *spin : {resolution_, margin_, max_step_, max_slope_}) {
    connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
            [this](double) { emit parameters_changed(); });
  }
  connect(min_points_, qOverload<int>(&QSpinBox::valueChanged), this,
          [this](int) { emit parameters_changed(); });
  connect(use_trajectory_, &QCheckBox::toggled, this, [this](bool) { emit parameters_changed(); });

  auto *publish_box = new QGroupBox(QStringLiteral("Publish target"), host);
  auto *publish_form = new QFormLayout(publish_box);
  map_root_ = new QLineEdit(publish_box);
  map_id_ = new QLineEdit(publish_box);
  map_id_->setPlaceholderText(QStringLiteral("e.g. bunker_mid360_mapping_20260901_205036"));
  map_version_ = new QLineEdit(publish_box);
  map_version_->setPlaceholderText(QStringLiteral("e.g. v004-studio"));
  activate_ = new QCheckBox(QStringLiteral("Activate after publish (select_map_package)"), publish_box);
  publish_form->addRow(QStringLiteral("Map root"), map_root_);
  publish_form->addRow(QStringLiteral("Map id"), map_id_);
  publish_form->addRow(QStringLiteral("Version"), map_version_);
  publish_form->addRow(activate_);
  layout->addWidget(publish_box);
  for (auto *edit : {map_root_, map_id_, map_version_}) {
    connect(edit, &QLineEdit::textChanged, this, [this](const QString &) { emit parameters_changed(); });
  }
  connect(activate_, &QCheckBox::toggled, this, [this](bool) { emit parameters_changed(); });

  auto *buttons = new QHBoxLayout();
  run_all_ = new QPushButton(QStringLiteral("Run all pending steps"), host);
  cancel_ = new QPushButton(QStringLiteral("Cancel"), host);
  cancel_->setEnabled(false);
  buttons->addWidget(run_all_);
  buttons->addWidget(cancel_);
  layout->addLayout(buttons);
  connect(run_all_, &QPushButton::clicked, this, &WorkflowPanel::run_all_requested);
  connect(cancel_, &QPushButton::clicked, this, &WorkflowPanel::cancel_requested);

  progress_label_ = new QLabel(QStringLiteral("Idle"), host);
  progress_ = new QProgressBar(host);
  progress_->setRange(0, 1);
  progress_->setValue(0);
  progress_->setTextVisible(false);
  layout->addWidget(progress_label_);
  layout->addWidget(progress_);

  log_ = new QPlainTextEdit(host);
  log_->setReadOnly(true);
  log_->setMaximumBlockCount(4000);
  log_->setMinimumHeight(160);
  log_->setPlaceholderText(QStringLiteral("External tool output appears here."));
  layout->addWidget(log_, 1);

  scroll->setWidget(host);
  outer->addWidget(scroll);
}

WorkflowPanel::StepRow WorkflowPanel::make_step(int number, const QString &title,
                                                const QString &hint, QWidget *host) {
  auto *frame = new QFrame(host);
  frame->setFrameShape(QFrame::StyledPanel);
  auto *layout = new QVBoxLayout(frame);
  layout->setContentsMargins(6, 4, 6, 4);
  auto *header = new QHBoxLayout();
  StepRow row;
  row.title = new QLabel(QStringLiteral("<b>%1. %2</b>").arg(number).arg(title), frame);
  row.badge = new QLabel(frame);
  row.badge->setAlignment(Qt::AlignCenter);
  row.badge->setMinimumWidth(64);
  header->addWidget(row.title, 1);
  header->addWidget(row.badge);
  layout->addLayout(header);
  row.detail = new QLabel(hint, frame);
  row.detail->setWordWrap(true);
  row.detail->setStyleSheet(QStringLiteral("color: #666; font-size: 11px;"));
  layout->addWidget(row.detail);
  auto *actions = new QHBoxLayout();
  row.run = new QPushButton(QStringLiteral("Run"), frame);
  row.open = new QPushButton(QStringLiteral("Open output"), frame);
  row.open->setEnabled(false);
  actions->addWidget(row.run);
  actions->addWidget(row.open);
  actions->addStretch(1);
  layout->addLayout(actions);
  switch (number) {
    case 1: connect(row.run, &QPushButton::clicked, this, &WorkflowPanel::refine_requested); break;
    case 2: connect(row.run, &QPushButton::clicked, this, &WorkflowPanel::relocalization_requested); break;
    case 3: connect(row.run, &QPushButton::clicked, this, &WorkflowPanel::navigation_requested); break;
    case 4: connect(row.run, &QPushButton::clicked, this, &WorkflowPanel::patch_requested); break;
    default: connect(row.run, &QPushButton::clicked, this, &WorkflowPanel::publish_requested); break;
  }
  return row;
}

void WorkflowPanel::paint_badge(QLabel *badge, StageState state, bool applicable) {
  QString text;
  QString color;
  if (!applicable) {
    text = QStringLiteral("n/a");
    color = QStringLiteral("#9e9e9e");
  } else {
    switch (state) {
      case StageState::Fresh: text = QStringLiteral("fresh"); color = QStringLiteral("#2e7d32"); break;
      case StageState::Stale: text = QStringLiteral("STALE"); color = QStringLiteral("#ef6c00"); break;
      default: text = QStringLiteral("missing"); color = QStringLiteral("#757575"); break;
    }
  }
  badge->setText(text);
  badge->setStyleSheet(QStringLiteral("QLabel { background: %1; color: white; border-radius: 6px; "
                                      "padding: 2px 6px; font-weight: bold; }").arg(color));
}

void WorkflowPanel::refresh(const WorkflowSession &session, bool tool_running) {
  if (session.empty()) {
    source_label_->setText(QStringLiteral("No source loaded. Open a PCD or a mapping package."));
  } else {
    source_label_->setText(QStringLiteral("Source: %1%2\nWork dir: %3")
                               .arg(session.source_pcd(),
                                    session.source_is_mapping_package()
                                        ? QStringLiteral(" (mapping package)")
                                        : QStringLiteral(" (bare PCD)"),
                                    session.work_dir()));
  }
  const bool applicable[5] = {session.has_3d_edits(), true, true, session.has_2d_edits(), true};
  const WorkflowSession::Stage stages[5] = {
      WorkflowSession::Refine, WorkflowSession::Relocalization, WorkflowSession::Navigation,
      WorkflowSession::Patch, WorkflowSession::Publish};
  for (int i = 0; i < 5; ++i) {
    const auto &record = session.record(stages[i]);
    const StageState state = session.state(stages[i]);
    paint_badge(steps_[i].badge, state, applicable[i]);
    steps_[i].run->setEnabled(!tool_running && !session.empty() && applicable[i]);
    steps_[i].open->setEnabled(!record.path.isEmpty());
    steps_[i].open->disconnect();
    if (!record.path.isEmpty()) {
      const QString path = record.path;
      connect(steps_[i].open, &QPushButton::clicked, this,
              [this, path]() { emit open_output_requested(path); });
      steps_[i].detail->setToolTip(QStringLiteral("%1\n%2").arg(record.path, record.updated_at));
    }
  }
  if (!session.has_3d_edits()) {
    steps_[0].detail->setText(QStringLiteral("No 3D deletions yet: downstream steps use the source PCD."));
  } else {
    steps_[0].detail->setText(QStringLiteral("Refinement rules -> %1").arg(session.refinement_rules_path()));
  }
  if (!session.has_2d_edits()) {
    steps_[3].detail->setText(QStringLiteral("No 2D edits yet: publish uses the generated navigation layers."));
  } else {
    steps_[3].detail->setText(QStringLiteral("Patch YAML -> %1").arg(session.navigation_patch_path()));
  }
  const QStringList reasons = session.blocking_reasons_for_publish();
  steps_[4].detail->setText(reasons.isEmpty()
                                ? QStringLiteral("Ready to publish.")
                                : QStringLiteral("Blocked: %1").arg(reasons.join(QStringLiteral("; "))));
  steps_[4].run->setEnabled(!tool_running && reasons.isEmpty());
  run_all_->setEnabled(!tool_running && !session.empty());
  cancel_->setEnabled(tool_running);
}

void WorkflowPanel::append_log(const QString &text) {
  log_->moveCursor(QTextCursor::End);
  log_->insertPlainText(text);
  log_->moveCursor(QTextCursor::End);
}

void WorkflowPanel::clear_log() { log_->clear(); }

void WorkflowPanel::set_progress(const QString &label, bool busy) {
  progress_label_->setText(label);
  progress_->setRange(0, busy ? 0 : 1);
  progress_->setValue(busy ? 0 : 1);
}

void WorkflowPanel::read_converter(ConverterParameters *parameters) const {
  parameters->resolution = resolution_->value();
  parameters->margin = margin_->value();
  parameters->min_points = min_points_->value();
  parameters->max_step = max_step_->value();
  parameters->max_slope_deg = max_slope_->value();
  parameters->use_trajectory = use_trajectory_->isChecked();
}

void WorkflowPanel::set_converter(const ConverterParameters &parameters) {
  resolution_->setValue(parameters.resolution);
  margin_->setValue(parameters.margin);
  min_points_->setValue(parameters.min_points);
  max_step_->setValue(parameters.max_step);
  max_slope_->setValue(parameters.max_slope_deg);
  use_trajectory_->setChecked(parameters.use_trajectory);
}

void WorkflowPanel::read_publish_target(PublishTarget *target) const {
  target->map_root = map_root_->text().trimmed();
  target->map_id = map_id_->text().trimmed();
  target->map_version = map_version_->text().trimmed();
  target->activate = activate_->isChecked();
}

void WorkflowPanel::set_publish_target(const PublishTarget &target) {
  map_root_->setText(target.map_root);
  map_id_->setText(target.map_id);
  map_version_->setText(target.map_version);
  activate_->setChecked(target.activate);
}

}  // namespace agt_map_studio
