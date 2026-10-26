#include "ui/CoverageAuthoringDialog.hpp"
#include "ui/CoverageAuthoringIO.hpp"

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDir>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFileSystemWatcher>
#include <QFormLayout>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QHBoxLayout>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QProcessEnvironment>
#include <QSaveFile>
#include <QSplitter>
#include <QSpinBox>
#include <QTabWidget>
#include <QUuid>
#include <QVBoxLayout>

#include <cmath>

namespace agt_map_studio {
namespace {

QString absolute_reference(const QString &owner, const QString &reference) {
  if (reference.isEmpty()) return {};
  return QFileInfo(reference).isAbsolute() ? reference : QFileInfo(owner).dir().absoluteFilePath(reference);
}

QDoubleSpinBox *number(QWidget *parent, double minimum, double maximum,
                       double value, int decimals = 3) {
  auto *spin = new QDoubleSpinBox(parent);
  spin->setRange(minimum, maximum);
  spin->setDecimals(decimals);
  spin->setValue(value);
  spin->setKeyboardTracking(false);
  return spin;
}

}  // namespace

CoverageAuthoringDialog::CoverageAuthoringDialog(const QString &map_yaml,
                                                 const QJsonObject &annotations,
                                                 const QString &annotation_file,
                                                 QWidget *parent)
    : QDialog(parent), runner_(this), annotations_(annotations), annotation_file_(annotation_file) {
  setWindowTitle(QStringLiteral("Coverage authoring / 全覆盖路线生成（离线）"));
  resize(1200, 800);
  auto *layout = new QVBoxLayout(this);
  auto *intro = new QLabel(QStringLiteral(
      "Select a reviewed map, robot profile and confirmed structure. Green = work; orange = forward connection; "
      "purple = reverse. Paths use the rear axle reference. Results are offline drafts."), this);
  intro->setWordWrap(true);
  layout->addWidget(intro);
  show_clearance_ = new QCheckBox(QStringLiteral("净空热图 / Clearance: red 0 m → green 2 m; ≥2 m saturated, unknown transparent"), this);
  show_clearance_->setChecked(true);
  layout->addWidget(show_clearance_);
  auto *split = new QSplitter(this);
  auto *controls = new QWidget(split);
  auto *form = new QFormLayout(controls);
  const auto path_field = [this, form](const QString &label, QLineEdit **target,
                                      const QString &filter, bool directory) {
    auto *row = new QWidget(this);
    auto *line = new QHBoxLayout(row);
    line->setContentsMargins(0, 0, 0, 0);
    auto *edit = new QLineEdit(row);
    auto *browse = new QPushButton(QStringLiteral("…"), row);
    browse->setMaximumWidth(35);
    line->addWidget(edit);
    line->addWidget(browse);
    form->addRow(label, row);
    *target = edit;
    connect(edit, &QLineEdit::textChanged, this, [this]() { mark_stale(); });
    connect(browse, &QPushButton::clicked, this, [this, edit, filter, directory]() {
      // Output must be a new child directory. Choosing the parent does not
      // pre-create the result directory, which is owned by the CLI.
      const QString chosen = directory
          ? QFileDialog::getExistingDirectory(this, QStringLiteral("Choose output parent"), edit->text())
          : QFileDialog::getOpenFileName(this, QStringLiteral("Select input"), edit->text(), filter);
      if (!chosen.isEmpty()) edit->setText(directory
          ? QDir(chosen).filePath(QStringLiteral("coverage-") + QUuid::createUuid().toString(QUuid::WithoutBraces).left(8))
          : chosen);
    });
  };
  path_field(QStringLiteral("Map YAML / 地图"), &map_, QStringLiteral("Map (*.yaml *.yml)"), false);
  path_field(QStringLiteral("Robot profile / 底盘"), &profile_, QStringLiteral("Profile (*.yaml *.yml)"), false);
  path_field(QStringLiteral("Structure / 结构"), &structure_, QStringLiteral("Structure (*.geojson *.json *.yaml *.yml)"), false);
  path_field(QStringLiteral("New output / 新输出目录"), &output_, {}, true);
  map_->setObjectName("coverageMapPath");
  profile_->setObjectName("coverageProfilePath");
  structure_->setObjectName("coverageStructurePath");
  output_->setObjectName("coverageOutputPath");
  map_->setText(map_yaml);
  mode_ = new QComboBox(this);
  mode_->addItem(QStringLiteral("行道遍历 / All selected aisles"), "rows");
  mode_->addItem(QStringLiteral("按幅宽覆盖 / Work area"), "area");
  form->addRow(QStringLiteral("Task / 模式"), mode_);
  width_ = number(this, 0.0, 100.0, 0.0);
  width_->setSpecialValueText(QStringLiteral("Not configured / 未填写"));
  width_->setSuffix(QStringLiteral(" m"));
  form->addRow(QStringLiteral("Operation width / 作业幅宽"), width_);
  clearance_ = number(this, 0.0, 10.0, 0.0);
  clearance_->setSuffix(QStringLiteral(" m"));
  form->addRow(QStringLiteral("Extra clearance / 额外净距"), clearance_);
  backend_ = new QComboBox(this);
  backend_->addItem(QStringLiteral("OpenNav + Smac (ROS)"), "ros");
  backend_->addItem(QStringLiteral("Offline reference / 离线参考后端"), "reference");
  form->addRow(QStringLiteral("Backend / 规划后端"), backend_);
  ros_domain_ = new QSpinBox(this);
  ros_domain_->setRange(-1, 232);
  ros_domain_->setValue(-1);
  ros_domain_->setSpecialValueText(QStringLiteral("Choose isolated domain / 请选择独立域"));
  ros_domain_->setToolTip(QStringLiteral("Must differ from the ROS domain used by the live robot. Reference planning and analysis do not use ROS."));
  form->addRow(QStringLiteral("Offline ROS domain / 离线域"), ros_domain_);
  const auto pose_row = [this, form](const QString &title, QDoubleSpinBox **pose) {
    auto *row = new QWidget(this);
    auto *line = new QHBoxLayout(row);
    line->setContentsMargins(0, 0, 0, 0);
    for (int i = 0; i < 3; ++i) {
      pose[i] = number(row, i == 2 ? -3.141593 : -100000.0, i == 2 ? 3.141593 : 100000.0, 0.0, 6);
      pose[i]->setToolTip(i == 0 ? "x (m)" : i == 1 ? "y (m)" : "body yaw (rad)");
      line->addWidget(pose[i]);
      connect(pose[i], qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this]() { mark_stale(); });
    }
    form->addRow(title, row);
  };
  pose_row(QStringLiteral("Entry x/y/yaw / 入口"), start_);
  return_start_ = new QCheckBox(QStringLiteral("Return to entry / 返回入口"), this);
  return_start_->setChecked(true);
  form->addRow(QString(), return_start_);
  pose_row(QStringLiteral("Exit x/y/yaw / 出口"), end_);
  for (auto *spin : end_) spin->setEnabled(false);
  reverse_ = new QCheckBox(QStringLiteral("Forward preferred; allow reverse / 必要时倒车"), this);
  reverse_->setChecked(true);
  form->addRow(QString(), reverse_);
  load_ = new QPushButton(QStringLiteral("Load request / 加载任务配置"), this);
  export_ = new QPushButton(QStringLiteral("Export reviewed annotations / 导出已审查结构"), this);
  export_->setEnabled(!annotations_.isEmpty() && !annotation_file_.isEmpty());
  form->addRow(load_);
  form->addRow(export_);

  auto *tabs = new QTabWidget(split);
  scene_ = new QGraphicsScene(this);
  view_ = new QGraphicsView(scene_, tabs);
  view_->setObjectName("coveragePreview");
  view_->setDragMode(QGraphicsView::ScrollHandDrag);
  view_->setRenderHint(QPainter::Antialiasing);
  tabs->addTab(view_, QStringLiteral("Map and route / 地图路线"));
  report_ = new QPlainTextEdit(tabs);
  report_->setObjectName("coverageReport");
  report_->setReadOnly(true);
  tabs->addTab(report_, QStringLiteral("Quality report / 质量报告"));
  log_ = new QPlainTextEdit(tabs);
  log_->setObjectName("coverageLog");
  log_->setReadOnly(true);
  log_->setMaximumBlockCount(2000);
  tabs->addTab(log_, QStringLiteral("Job log / 任务日志"));
  split->setStretchFactor(1, 1);
  layout->addWidget(split, 1);
  status_ = new QLabel(QStringLiteral("Choose inputs and generate a new draft."), this);
  status_->setObjectName("coverageStatus");
  status_->setWordWrap(true);
  layout->addWidget(status_);
  progress_ = new QProgressBar(this);
  progress_->setRange(0, 1);
  progress_->setValue(0);
  layout->addWidget(progress_);
  auto *buttons = new QHBoxLayout();
  analyze_ = new QPushButton(QStringLiteral("Analyze clearance / 净空分析"), this);
  plan_ = new QPushButton(QStringLiteral("Generate route / 生成路线"), this);
  plan_->setObjectName("coveragePlan");
  export_route_ = new QPushButton(QStringLiteral("Export route draft / 导出路线草稿"), this);
  export_route_->setObjectName("coverageExportDraft");
  export_route_->setEnabled(false);
  cancel_ = new QPushButton(QStringLiteral("Cancel / 取消"), this);
  cancel_->setEnabled(false);
  auto *fit = new QPushButton(QStringLiteral("Fit map / 全图"), this);
  auto *close = new QPushButton(QStringLiteral("Close / 关闭"), this);
  for (auto *button : {analyze_, plan_, export_route_, cancel_, fit, close}) buttons->addWidget(button);
  layout->addLayout(buttons);
  watcher_ = new QFileSystemWatcher(this);
  connect(watcher_, &QFileSystemWatcher::fileChanged, this, [this](const QString &) { mark_stale(); });
  connect(load_, &QPushButton::clicked, this, &CoverageAuthoringDialog::load_request);
  connect(export_, &QPushButton::clicked, this, &CoverageAuthoringDialog::export_annotations);
  connect(analyze_, &QPushButton::clicked, this, [this]() { run("analyze"); });
  connect(plan_, &QPushButton::clicked, this, [this]() { run("plan"); });
  connect(export_route_, &QPushButton::clicked, this, &CoverageAuthoringDialog::export_route_draft);
  connect(cancel_, &QPushButton::clicked, this, [this]() {
    canceled_ = true;
    runner_.cancel();
    status_->setText(QStringLiteral("Canceled; no result is accepted."));
  });
  connect(close, &QPushButton::clicked, this, &CoverageAuthoringDialog::reject);
  connect(fit, &QPushButton::clicked, this, [this]() { view_->fitInView(scene_->itemsBoundingRect(), Qt::KeepAspectRatio); });
  connect(show_clearance_, &QCheckBox::toggled, this, [this]() {
    if (!stale_ && !current_result_.isEmpty()) draw_preview(current_result_);
  });
  connect(return_start_, &QCheckBox::toggled, this, [this](bool checked) {
    for (auto *spin : end_) spin->setEnabled(!checked);
    mark_stale();
  });
  connect(reverse_, &QCheckBox::toggled, this, [this]() { mark_stale(); });
  for (auto *combo : {mode_, backend_}) connect(combo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() { mark_stale(); });
  connect(ros_domain_, qOverload<int>(&QSpinBox::valueChanged), this, [this]() { mark_stale(); });
  for (auto *spin : {width_, clearance_}) connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this]() { mark_stale(); });
  connect(&runner_, &ExternalToolRunner::output_appended, log_, &QPlainTextEdit::appendPlainText);
  connect(&runner_, &ExternalToolRunner::finished, this, [this](const ToolResult &result) {
    set_running(false);
    if (stale_ || canceled_) return;
    if (!result.ok) {
      status_->setText(QStringLiteral("Job failed; no new result accepted. %1").arg(result.error_summary));
      return;
    }
    if (active_operation_ == "export") {
      status_->setText(QStringLiteral("Route draft saved to %1. This draft is not a READY execution asset.").arg(export_output_));
      return;
    }
    load_result();
  });
}

void CoverageAuthoringDialog::set_running(bool running) {
  analyze_->setEnabled(!running);
  plan_->setEnabled(!running);
  load_->setEnabled(!running);
  export_->setEnabled(!running && !annotations_.isEmpty() && !annotation_file_.isEmpty());
  export_route_->setEnabled(!running && plan_ready_ && !stale_ && !canceled_);
  cancel_->setEnabled(running);
  progress_->setRange(0, running ? 0 : 1);
  progress_->setValue(running ? 0 : 1);
}

void CoverageAuthoringDialog::mark_stale() {
  // Early constructor edits occur before the preview widgets exist.
  if (!status_) return;
  stale_ = true;
  plan_ready_ = false;
  export_route_->setEnabled(false);
  current_result_ = {};
  scene_->clear();
  report_->clear();
  if (runner_.is_running()) runner_.cancel();
  status_->setText(QStringLiteral("Inputs changed. Previous result is stale; generate a new output."));
}

void CoverageAuthoringDialog::load_request() {
  const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Load coverage request"), {},
                                                   QStringLiteral("Request (*.yaml *.yml *.json)"));
  if (path.isEmpty()) return;
  QString error;
  if (!load_request_file(path, &error)) QMessageBox::warning(this, QStringLiteral("Cannot load request"), error);
}

bool CoverageAuthoringDialog::load_request_file(const QString &path, QString *error) {
  QJsonObject request;
  if (!CoverageAuthoringIO::read_document(path, &request, error)) return false;
  const auto start = request.value("start").toArray();
  const auto end = request.contains("end") ? request.value("end").toArray() : start;
  const auto valid_pose = [](const QJsonArray &pose) {
    if (pose.size() != 3) return false;
    for (const auto &value : pose) if (!value.isDouble() || !std::isfinite(value.toDouble())) return false;
    return std::abs(pose[0].toDouble()) <= 100000.0 && std::abs(pose[1].toDouble()) <= 100000.0 &&
           std::abs(pose[2].toDouble()) <= 3.141593;
  };
  if (!valid_pose(start) || !valid_pose(end) ||
      mode_->findData(request.value("mode").toString("rows")) < 0 ||
      backend_->findData(request.value("backend").toString("ros")) < 0) {
    if (error) *error = QStringLiteral("The request needs finite [x,y,yaw] poses, mode rows/area and backend ros/reference.");
    return false;
  }
  request_defaults_ = request;
  map_->setText(absolute_reference(path, request.value("map_yaml").toString()));
  profile_->setText(absolute_reference(path, request.value("robot_profile").toString()));
  structure_->setText(absolute_reference(path, request.value("structure").toString()));
  if (request.contains("site_root")) request_defaults_.insert("site_root", absolute_reference(path, request.value("site_root").toString()));
  mode_->setCurrentIndex(qMax(0, mode_->findData(request.value("mode").toString("rows"))));
  backend_->setCurrentIndex(qMax(0, backend_->findData(request.value("backend").toString("ros"))));
  ros_domain_->setValue(request.value("backend_options").toObject().value("ros_domain_id").toInt(-1));
  width_->setValue(request.value("operation_width_m").toDouble(0.0));
  clearance_->setValue(request.value("clearance_margin_m").toDouble(0.0));
  reverse_->setChecked(request.value("allow_reverse").toBool(true));
  for (int i = 0; i < 3; ++i) {
    start_[i]->setValue(start[i].toDouble());
    end_[i]->setValue(end[i].toDouble());
  }
  return_start_->setChecked(!request.contains("end") || request.value("end") == request.value("start"));
  mark_stale();
  return true;
}

void CoverageAuthoringDialog::export_annotations() {
  const QString output = QFileDialog::getSaveFileName(this, QStringLiteral("New reviewed coverage structure"), {},
                                                      QStringLiteral("GeoJSON (*.geojson)"));
  if (output.isEmpty()) return;
  QJsonArray map_from_reference;
  bool transform_confirmed = false;
  QStringList selected_headlands;
  {
    const bool needs_transform = annotations_.value("reference_frame").toString() != "map";
    QDialog alignment(this);
    alignment.setWindowTitle(QStringLiteral("Review structure export / 确认结构导出"));
    auto *layout = new QVBoxLayout(&alignment);
    auto *explanation = new QLabel(needs_transform ? QStringLiteral(
        "Source frame: %1\nTarget frame: map\n"
        "Enter the reviewed planar transform: p_map = R(yaw) p_source + [dx,dy]. "
        "Zero values mean an explicit identity choice. Original annotations remain unchanged.")
        .arg(annotations_.value("reference_frame").toString())
        : QStringLiteral("Annotations already use map coordinates. Original annotations remain unchanged."), &alignment);
    explanation->setTextFormat(Qt::PlainText);
    explanation->setWordWrap(true);
    layout->addWidget(explanation);
    auto *form = new QFormLayout();
    auto *dx = number(&alignment, -100000.0, 100000.0, 0.0, 6);
    auto *dy = number(&alignment, -100000.0, 100000.0, 0.0, 6);
    auto *yaw = number(&alignment, -3.141593, 3.141593, 0.0, 6);
    form->addRow(QStringLiteral("dx (m)"), dx);
    form->addRow(QStringLiteral("dy (m)"), dy);
    form->addRow(QStringLiteral("yaw (rad)"), yaw);
    layout->addLayout(form);
    for (auto *spin : {dx, dy, yaw}) spin->setEnabled(needs_transform);
    auto *confirmed = new QCheckBox(QStringLiteral("I reviewed this alignment / 我已核对该坐标变换"), &alignment);
    confirmed->setVisible(needs_transform);
    layout->addWidget(confirmed);
    QStringList available_headlands;
    for (const auto &value : annotations_.value("annotations").toArray()) {
      const auto item = value.toObject();
      const QString status = item.value("review_status").toString();
      if (item.value("annotation_type").toString() == "transition_region" &&
          (status == "REVIEWED" || status == "FROZEN")) {
        available_headlands << item.value("annotation_id").toString();
      }
    }
    auto *headlands = new QCheckBox(QStringLiteral(
        "Use %1 reviewed transition regions as headlands (connection only) / 将已审查过渡区用于地头换行")
        .arg(available_headlands.size()), &alignment);
    headlands->setEnabled(!available_headlands.isEmpty());
    headlands->setChecked(false);
    layout->addWidget(headlands);
    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &alignment);
    buttons->button(QDialogButtonBox::Ok)->setEnabled(!needs_transform);
    connect(confirmed, &QCheckBox::toggled, buttons->button(QDialogButtonBox::Ok), &QPushButton::setEnabled);
    connect(buttons, &QDialogButtonBox::accepted, &alignment, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &alignment, &QDialog::reject);
    layout->addWidget(buttons);
    if (alignment.exec() != QDialog::Accepted) return;
    if (needs_transform) {
      map_from_reference = QJsonArray{dx->value(), dy->value(), yaw->value()};
      transform_confirmed = confirmed->isChecked();
    }
    if (headlands->isChecked()) selected_headlands = available_headlands;
  }
  QString error;
  if (!CoverageAuthoringIO::export_structure(annotations_, annotation_file_, map_->text(), output, &error,
                                            map_from_reference, transform_confirmed, selected_headlands)) {
    QMessageBox::warning(this, QStringLiteral("Structure not exported"), error);
    return;
  }
  structure_->setText(output);
}

void CoverageAuthoringDialog::watch_sources(const QString &request_path) {
  const auto old = watcher_->files();
  if (!old.isEmpty()) watcher_->removePaths(old);
  QStringList paths{request_path, map_->text(), profile_->text(), structure_->text()};
  QJsonObject metadata;
  QString error;
  if (CoverageAuthoringIO::read_document(map_->text(), &metadata, &error))
    paths << absolute_reference(map_->text(), metadata.value("image").toString());
  QJsonObject structure;
  if (CoverageAuthoringIO::read_document(structure_->text(), &structure, &error)) {
    const QString source = structure.value("source").toObject().value("annotation_path").toString();
    if (!source.isEmpty()) paths << absolute_reference(structure_->text(), source);
  }
  paths.removeDuplicates();
  for (const auto &path : paths) if (QFileInfo(path).isFile()) watcher_->addPath(path);
}

void CoverageAuthoringDialog::run(const QString &operation) {
  if (runner_.is_running()) return;
  if (operation == "plan" && backend_->currentData().toString() == "ros" &&
      (ros_domain_->value() < 0 || ros_domain_->value() ==
          QProcessEnvironment::systemEnvironment().value("ROS_DOMAIN_ID", "0").toInt())) {
    QMessageBox::warning(this, QStringLiteral("Isolated ROS domain required / 请选择独立离线域"),
        QStringLiteral("Select an offline ROS domain different from the live robot domain. It is never selected silently."));
    return;
  }
  if (mode_->currentData().toString() == "area" && width_->value() <= 0.0) {
    QMessageBox::warning(this, QStringLiteral("Operation width required / 请填写作业幅宽"),
        QStringLiteral("Area coverage requires the actual operation width; it is not inferred from vehicle width."));
    return;
  }
  for (auto *input : {map_, profile_, structure_}) {
    if (!QFileInfo(input->text()).isFile()) {
      QMessageBox::warning(this, QStringLiteral("Input missing"), QStringLiteral("Select existing map, profile and structure files."));
      return;
    }
  }
  const QFileInfo destination(output_->text());
  if (output_->text().trimmed().isEmpty() || destination.exists() || !destination.dir().exists()) {
    QMessageBox::warning(this, QStringLiteral("New output required"), QStringLiteral("Use a new output directory inside an existing parent directory."));
    return;
  }
  if (!ExternalToolRunner::program_available("agt_coverage_authoring") &&
      !ExternalToolRunner::ros2_executable_available("agt_coverage_authoring", "agt_coverage_authoring")) {
    QMessageBox::warning(this, QStringLiteral("Coverage package unavailable"),
        QStringLiteral("Install agt_coverage_authoring and source its environment before starting MapStudio."));
    return;
  }
  QJsonObject request = request_defaults_;
  request.insert("map_yaml", QFileInfo(map_->text()).absoluteFilePath());
  request.insert("robot_profile", QFileInfo(profile_->text()).absoluteFilePath());
  request.insert("structure", QFileInfo(structure_->text()).absoluteFilePath());
  request.insert("mode", mode_->currentData().toString());
  request.insert("backend", backend_->currentData().toString());
  auto backend_options = request.value("backend_options").toObject();
  if (ros_domain_->value() >= 0) backend_options.insert("ros_domain_id", ros_domain_->value());
  else backend_options.remove("ros_domain_id");
  request.insert("backend_options", backend_options);
  if (width_->value() > 0.0) request.insert("operation_width_m", width_->value());
  else request.remove("operation_width_m");
  request.insert("clearance_margin_m", clearance_->value());
  request.insert("allow_reverse", reverse_->isChecked());
  QJsonArray start, end;
  for (int i = 0; i < 3; ++i) {
    start.push_back(start_[i]->value());
    end.push_back(return_start_->isChecked() ? start_[i]->value() : end_[i]->value());
  }
  request.insert("start", start);
  request.insert("end", end);
  const QString request_path = destination.dir().filePath(QStringLiteral("coverage-request-%1.json")
      .arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));
  QSaveFile file(request_path);
  const auto contents = QJsonDocument(request).toJson(QJsonDocument::Indented);
  if (!file.open(QIODevice::WriteOnly) || file.write(contents) != contents.size() || !file.commit()) {
    QMessageBox::warning(this, QStringLiteral("Request not saved"), QStringLiteral("Cannot write request beside the output directory."));
    return;
  }
  active_output_ = destination.absoluteFilePath();
  active_operation_ = operation;
  request_path_ = request_path;
  plan_ready_ = false;
  stale_ = false;
  canceled_ = false;
  current_result_ = {};
  scene_->clear();
  report_->clear();
  log_->clear();
  watch_sources(request_path);
  const QStringList arguments{operation, "--request", request_path, "--output", active_output_};
  ToolInvocation invocation;
  if (ExternalToolRunner::program_available("agt_coverage_authoring")) {
    invocation.label = QStringLiteral("Offline coverage %1").arg(operation);
    invocation.program = "agt_coverage_authoring";
    invocation.arguments = arguments;
  } else {
    invocation = ExternalToolRunner::ros2_run(QStringLiteral("Offline coverage %1").arg(operation),
        "agt_coverage_authoring", "agt_coverage_authoring", arguments);
  }
  set_running(true);
  status_->setText(QStringLiteral("Running %1; changing an input cancels this job.").arg(operation));
  invocation.cancellation_grace_ms = 10000;
  runner_.start(invocation);
}

void CoverageAuthoringDialog::export_route_draft() {
  if (!plan_ready_ || stale_ || canceled_ || runner_.is_running()) return;
  const QString parent = QFileDialog::getExistingDirectory(this, QStringLiteral("Save route draft under"),
                                                           QFileInfo(active_output_).absolutePath());
  if (parent.isEmpty()) return;
  export_output_ = QDir(parent).filePath(QStringLiteral("route-draft-") +
      QUuid::createUuid().toString(QUuid::WithoutBraces).left(8));
  const QStringList arguments{"export", "--request", request_path_, "--plan",
      QDir(active_output_).filePath("plan.json"), "--format", "draft", "--output", export_output_};
  ToolInvocation invocation;
  if (ExternalToolRunner::program_available("agt_coverage_authoring")) {
    invocation.label = "Export offline route draft";
    invocation.program = "agt_coverage_authoring";
    invocation.arguments = arguments;
  } else {
    invocation = ExternalToolRunner::ros2_run("Export offline route draft", "agt_coverage_authoring",
                                             "agt_coverage_authoring", arguments);
  }
  active_operation_ = "export";
  set_running(true);
  status_->setText(QStringLiteral("Exporting the complete draft with current source bindings…"));
  invocation.cancellation_grace_ms = 10000;
  runner_.start(invocation);
}

void CoverageAuthoringDialog::load_result() {
  QJsonObject result;
  QString error;
  const QString path = QDir(active_output_).filePath(active_operation_ == "plan" ? "plan.json" : "report.json");
  if (!CoverageAuthoringIO::read_document(path, &result, &error)) {
    status_->setText(QStringLiteral("No valid result: %1").arg(error));
    return;
  }
  report_->setPlainText(QString::fromUtf8(QJsonDocument(result).toJson(QJsonDocument::Indented)));
  current_result_ = result;
  plan_ready_ = active_operation_ == "plan" && !result.value("points").toArray().isEmpty();
  export_route_->setEnabled(plan_ready_);
  draw_preview(result);
  status_->setText(active_operation_ == "analyze"
      ? QStringLiteral("Clearance analysis ready. Check the quality report before planning.")
      : result.value("complete").toBool()
          ? QStringLiteral("Complete offline candidate. Review quality and runtime compatibility; this does not enable motion.")
          : QStringLiteral("Partial or blocked candidate. Missing targets remain in the report and coverage denominator."));
}

void CoverageAuthoringDialog::draw_preview(const QJsonObject &result) {
  scene_->clear();
  QJsonObject map;
  QString error;
  if (!CoverageAuthoringIO::read_document(map_->text(), &map, &error)) return;
  const QImage image(absolute_reference(map_->text(), map.value("image").toString()));
  const auto origin = map.value("origin").toArray();
  const double resolution = map.value("resolution").toDouble();
  if (image.isNull() || resolution <= 0.0 || origin.size() != 3) return;
  const double yaw = origin[2].toDouble();
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double height = image.height() * resolution;
  auto *raster = scene_->addPixmap(QPixmap::fromImage(image));
  raster->setTransformationMode(Qt::SmoothTransformation);
  const QTransform image_transform(resolution * c, -resolution * s,
                                  resolution * s, resolution * c,
                                  origin[0].toDouble() - s * height,
                                  -origin[1].toDouble() - c * height);
  raster->setTransform(image_transform);
  raster->setZValue(-1.0);
  if (show_clearance_->isChecked()) {
    const QImage heatmap(QDir(active_output_).filePath("clearance.png"));
    if (!heatmap.isNull() && heatmap.size() == image.size()) {
      auto *layer = scene_->addPixmap(QPixmap::fromImage(heatmap));
      layer->setTransform(image_transform);
      layer->setOpacity(0.55);
      layer->setZValue(-0.5);
    }
  }
  const auto points = result.value("points").toArray();
  if (points.size() > 250000) return;
  QPainterPath work, connections, reverse;
  for (int index = 1; index < points.size(); ++index) {
    const auto a = points[index - 1].toObject(), b = points[index].toObject();
    const double ax = a.value("x").toDouble(NAN), ay = a.value("y").toDouble(NAN);
    const double bx = b.value("x").toDouble(NAN), by = b.value("y").toDouble(NAN);
    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(bx) || !std::isfinite(by)) continue;
    QPainterPath &path = b.value("direction").toString() == "R" ? reverse
        : b.value("kind").toString() == "SWATH" ? work : connections;
    path.moveTo(ax, -ay);
    path.lineTo(bx, -by);
  }
  const auto draw = [this](const QPainterPath &path, const QColor &color) {
    QPen pen(color, 2.0);
    pen.setCosmetic(true);
    scene_->addPath(path, pen);
  };
  draw(work, QColor("#159447"));
  draw(connections, QColor("#e88b13"));
  draw(reverse, QColor("#aa44cc"));
  view_->fitInView(scene_->itemsBoundingRect(), Qt::KeepAspectRatio);
}

void CoverageAuthoringDialog::closeEvent(QCloseEvent *event) {
  canceled_ = true;
  runner_.cancel();
  event->accept();
}

void CoverageAuthoringDialog::reject() {
  canceled_ = true;
  runner_.cancel();
  QDialog::reject();
}

}  // namespace agt_map_studio
