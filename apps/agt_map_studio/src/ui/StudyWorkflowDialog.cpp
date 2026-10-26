#include "ui/StudyWorkflowDialog.hpp"

#include <QCloseEvent>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpressionValidator>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTabWidget>
#include <QTemporaryFile>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QUuid>
#include <yaml-cpp/yaml.h>

namespace agt_map_studio {
namespace {
QString string(const YAML::Node &value, const char *key, const QString &fallback = {}) {
  if (!value || !value.IsMap()) return fallback;
  return value[key] ? QString::fromStdString(value[key].as<std::string>()) : fallback;
}
YAML::Node read(const QString &path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly) || file.size() > 32 * 1024 * 1024)
    throw std::runtime_error("Cannot read asset, or asset exceeds 32 MiB");
  return YAML::Load(file.readAll().toStdString());
}
QString ref_path(const QString &owner, const YAML::Node &reference) {
  const QString path = string(reference, "path");
  return path.isEmpty() ? QString() : QDir(QFileInfo(owner).absolutePath()).absoluteFilePath(path);
}
bool valid_id(const QString &id) {
  return QRegularExpression(QStringLiteral("^[A-Za-z0-9_-]{1,80}$")).match(id).hasMatch();
}
QPushButton *button(QHBoxLayout *row, const QString &text, const QString &name,
                    const std::function<void()> &callback) {
  auto *result = new QPushButton(text);
  result->setObjectName(name);
  row->addWidget(result);
  QObject::connect(result, &QPushButton::clicked, result, callback);
  return result;
}
QHBoxLayout *action_row(QVBoxLayout *layout) {
  auto *row = new QHBoxLayout();
  layout->addLayout(row);
  return row;
}
QLineEdit *path_field(QFormLayout *form, const QString &label, bool directory = false) {
  auto *row = new QWidget();
  auto *layout = new QHBoxLayout(row);
  layout->setContentsMargins(0, 0, 0, 0);
  auto *edit = new QLineEdit(row);
  layout->addWidget(edit);
  button(layout, QStringLiteral("…"), {}, [edit, directory, label]() {
    const QString path = directory
        ? QFileDialog::getExistingDirectory(edit, label, edit->text())
        : QFileDialog::getOpenFileName(edit, label, edit->text());
    if (!path.isEmpty()) edit->setText(QFileInfo(path).absoluteFilePath());
  });
  form->addRow(label, row);
  return edit;
}
}  // namespace

StudyWorkflowDialog::StudyWorkflowDialog(QWidget *parent)
    : QDialog(parent), runner_(this), watcher_(this) {
  setObjectName(QStringLiteral("studyWorkflowDialog"));
  setWindowTitle(QStringLiteral("Offline localization studies / 离线重定位研究"));
  resize(1060, 780);
  auto *layout = new QVBoxLayout(this);
  auto *explanation = new QLabel(QStringLiteral(
      "Project → Query Set → frozen revision → offline Study. Same-session reference is not independent ground truth.\n"
      "项目 → 查询点集 → 冻结版本 → 离线研究。同次建图参考不等于独立真值；不启动导航或底盘。"));
  explanation->setWordWrap(true);
  layout->addWidget(explanation);
  auto *tabs = new QTabWidget(this);
  controls_ = tabs;
  layout->addWidget(tabs, 1);

  auto *project_page = new QWidget();
  auto *project_layout = new QVBoxLayout(project_page);
  auto *project_form = new QFormLayout();
  project_layout->addLayout(project_form);
  project_ = path_field(project_form, QStringLiteral("Project YAML / 项目"));
  project_->setObjectName(QStringLiteral("studyProjectPath"));
  source_ = path_field(project_form, QStringLiteral("Source map package / 建图源包"), true);
  blocks_ = path_field(project_form, QStringLiteral("Verified block set / 关键帧块"), true);
  topology_ = path_field(project_form, QStringLiteral("Physical topology YAML / 物理拓扑（可选）"));
  auto *project_buttons = action_row(project_layout);
  button(project_buttons, QStringLiteral("Open / validate / 打开校验"), "studyOpenProject", [this]() {
    load_project_file(project_->text());
  });
  button(project_buttons, QStringLiteral("Create project / 新建项目"), "studyCreateProject", [this]() {
    const auto output = new_file(QStringLiteral("New project / 新项目"), "project.yaml");
    if (output.isEmpty()) return;
    QStringList args{"project-init", "--project", output, "--map-package", source_->text(),
                     "--block-dir", blocks_->text()};
    if (!topology_->text().isEmpty()) args << "--topology" << topology_->text();
    run(args, [this, output]() { project_->setText(output); read_project(); });
  });
  button(project_buttons, QStringLiteral("Save / rebind / 保存依赖"), "studyRebindProject", [this]() {
    QStringList args{"project-set-dependencies", project_->text(), "--map-package", source_->text(),
                     "--block-dir", blocks_->text()};
    if (!topology_->text().isEmpty()) args << "--topology" << topology_->text();
    run(args, [this]() { read_project(); });
  });
  project_assets_ = new QComboBox();
  project_assets_->setObjectName("studyProjectAssets");
  project_form->addRow(QStringLiteral("Project assets / 项目资产"), project_assets_);
  auto *asset_buttons = action_row(project_layout);
  button(asset_buttons, QStringLiteral("Open selected asset / 打开所选"), "studyOpenAsset", [this]() {
    const QString kind = project_assets_->currentData(Qt::UserRole + 1).toString();
    const QString path = project_assets_->currentData().toString();
    if (kind == "query_set") { query_set_->setText(path); validate_queries(); }
    if (kind == "study") { study_->setText(path); validate_study(); }
  });
  button(asset_buttons, QStringLiteral("Show source map / 显示源地图"), "studyShowSource", [this]() {
    emit source_requested(source_->text());
  });
  ros_install_ = path_field(project_form, QStringLiteral("ROS install overlay / 安装目录"), true);
  ros_install_->setText(QDir::home().filePath("ros2_ws/install"));
  native_ = path_field(project_form, QStringLiteral("Native localizer / 离线定位程序（可选）"));
  cancel_grace_ = new QSpinBox();
  cancel_grace_->setRange(1000, 15000);
  cancel_grace_->setSingleStep(1000);
  cancel_grace_->setValue(10000);
  cancel_grace_->setSuffix(" ms");
  project_form->addRow(QStringLiteral("Cancellation cleanup grace / 取消清理时限"), cancel_grace_);
  project_layout->addStretch();
  tabs->addTab(project_page, QStringLiteral("Project / 项目"));

  auto *query_page = new QWidget();
  auto *query_layout = new QVBoxLayout(query_page);
  auto *query_form = new QFormLayout();
  query_layout->addLayout(query_form);
  query_set_ = path_field(query_form, QStringLiteral("Query Set YAML / 查询点集"));
  query_set_->setObjectName(QStringLiteral("studyQuerySetPath"));
  query_id_ = new QLineEdit("greenhouse_queries_v1");
  query_form->addRow(QStringLiteral("New Query Set ID / 新点集ID"), query_id_);
  auto *xy = new QWidget();
  auto *xy_layout = new QHBoxLayout(xy);
  xy_layout->setContentsMargins(0, 0, 0, 0);
  query_x_ = new QDoubleSpinBox();
  query_y_ = new QDoubleSpinBox();
  for (auto *spin : {query_x_, query_y_}) { spin->setRange(-1e7, 1e7); spin->setDecimals(4); xy_layout->addWidget(spin); }
  query_x_->setPrefix("X "); query_y_->setPrefix("Y ");
  button(xy_layout, QStringLiteral("Pick on map / 地图点选"), "studyPickPoint", [this]() { emit pick_requested(source_->text()); });
  query_form->addRow(QStringLiteral("Requested map point (m) / 请求位置"), xy);
  query_scene_ = new QComboBox();
  for (const auto &scene : {"ENTRY", "MIDDLE", "EXIT", "HEADLAND", "OTHER"}) query_scene_->addItem(scene, scene);
  query_form->addRow(QStringLiteral("Scene / 场景"), query_scene_);
  sample_ratios_ = new QComboBox();
  sample_ratios_->addItems({"0,0.25,0.5,0.75,1", "0,0.5,1", "0,1"});
  query_form->addRow(QStringLiteral("Reviewed-row sample ratios / 已确认行采样比例"), sample_ratios_);
  auto *query_create = action_row(query_layout);
  button(query_create, QStringLiteral("New manual set / 新建手工点集"), "studyNewManual", [this]() { create_query_set(false); });
  button(query_create, QStringLiteral("Sample frozen rows / 采样冻结行"), "studySampleRows", [this]() { create_query_set(true); });
  button(query_create, QStringLiteral("Validate / 校验"), "studyValidateQueries", [this]() { validate_queries(); });
  auto *query_edit = action_row(query_layout);
  button(query_edit, QStringLiteral("Add point / 添加点"), "studyAddPoint", [this]() { edit_query("add"); });
  button(query_edit, QStringLiteral("Enable / 启用"), "studyEnableQuery", [this]() { edit_query("enable"); });
  button(query_edit, QStringLiteral("Disable / 停用"), "studyDisableQuery", [this]() { edit_query("disable"); });
  button(query_edit, QStringLiteral("Delete / 删除"), "studyDeleteQuery", [this]() { edit_query("delete"); });
  auto *query_revisions = action_row(query_layout);
  button(query_revisions, QStringLiteral("Freeze revision / 冻结版本"), "studyFreezeQueries", [this]() { revise_query_set(true); });
  button(query_revisions, QStringLiteral("Clone to draft / 克隆草稿"), "studyCloneQueries", [this]() { revise_query_set(false); });
  queries_ = new QTreeWidget(); queries_->setObjectName("studyQueriesTree");
  queries_->setHeaderLabels({"ID", "Enabled / 启用", "Scene / 场景", "Physical row / 物理行", "KF", "Resolved XY / 实际采样位置", "Snap distance / 吸附距离"});
  const int query_column_widths[] = {170, 85, 100, 180, 55, 145, 190};
  for (int column = 0; column < 7; ++column) queries_->setColumnWidth(column, query_column_widths[column]);
  query_layout->addWidget(queries_, 1);
  tabs->addTab(query_page, QStringLiteral("Query Set / 查询点集"));

  auto *study_page = new QWidget();
  auto *study_layout = new QVBoxLayout(study_page);
  auto *study_form = new QFormLayout();
  study_layout->addLayout(study_form);
  study_ = path_field(study_form, QStringLiteral("Study YAML / 研究记录"));
  study_->setObjectName(QStringLiteral("studyManifestPath"));
  study_id_ = new QLineEdit("greenhouse_study_v1");
  study_form->addRow(QStringLiteral("New Study ID / 新研究ID"), study_id_);
  frames_ = new QComboBox();
  frames_->addItems({"1,3,5", "1", "3", "5", "1,3", "1,5", "3,5"});
  study_form->addRow(QStringLiteral("Query accumulation frames / 查询累积帧数"), frames_);
  top_k_ = new QSpinBox(); top_k_->setRange(1, 5); top_k_->setValue(5);
  study_form->addRow(QStringLiteral("Candidate Top-K / 候选数"), top_k_);
  auto *study_buttons = action_row(study_layout);
  button(study_buttons, QStringLiteral("Create / 新建"), "studyCreate", [this]() { create_study(); });
  button(study_buttons, QStringLiteral("Validate / 校验"), "studyValidate", [this]() { validate_study(); });
  button(study_buttons, QStringLiteral("Run offline / 离线运行"), "studyRun", [this]() { run_study(); });
  button(study_buttons, QStringLiteral("Export report / 导出报告"), "studyExport", [this]() { export_study(); });
  auto *filters = action_row(study_layout);
  class_filter_ = new QComboBox();
  class_filter_->addItems({"ALL", "CORRECT", "FALSE_ACCEPT", "REJECTED", "TIMEOUT", "NO_DATA", "REFERENCE_UNKNOWN", "NOT_RUN"});
  scene_filter_ = new QComboBox(); scene_filter_->addItems({"ALL", "ENTRY", "MIDDLE", "EXIT", "HEADLAND", "OTHER"});
  frame_filter_ = new QComboBox(); frame_filter_->addItems({"ALL", "1", "3", "5"});
  for (auto *combo : {class_filter_, scene_filter_, frame_filter_}) {
    filters->addWidget(combo);
    connect(combo, qOverload<int>(&QComboBox::currentIndexChanged), this, [this]() {
      if (!validated_study_path_.isEmpty()) { read_study(); preview_markers(); }
    });
  }
  button(filters, QStringLiteral("Show evidence / 查看证据"), "studyEvidence", [this]() {
    const auto *item = results_->currentItem();
    if (item && !item->data(0, Qt::UserRole).toString().isEmpty())
      emit evidence_requested(item->data(0, Qt::UserRole).toString());
  });
  summary_ = new QLabel(); summary_->setWordWrap(true); study_layout->addWidget(summary_);
  results_ = new QTreeWidget(); results_->setObjectName("studyResultsTree");
  results_->setHeaderLabels({"Query", "Scene", "Frames", "Classification", "Evidence"});
  study_layout->addWidget(results_, 1);
  tabs->addTab(study_page, QStringLiteral("Study / 研究结果"));
  status_ = new QLabel(QStringLiteral("Ready for offline authoring / 可开始离线编制"));
  status_->setObjectName("studyJobStatus"); status_->setWordWrap(true); layout->addWidget(status_);
  log_ = new QPlainTextEdit(); log_->setReadOnly(true); log_->setMaximumHeight(130); layout->addWidget(log_);
  auto *bottom = action_row(layout);
  cancel_ = button(bottom, QStringLiteral("Cancel job / 取消任务"), "studyCancel", [this]() {
    cancel_requested_ = true;
    cancel_->setEnabled(false);
    status_->setText(QStringLiteral("Cancel requested; waiting for worker cleanup / 已请求取消，等待清理"));
    runner_.cancel();
  });
  cancel_->setEnabled(false);
  button(bottom, QStringLiteral("Close / 关闭"), "studyClose", [this]() { close(); });

  connect(&runner_, &ExternalToolRunner::output_appended, log_, &QPlainTextEdit::appendPlainText);
  connect(&runner_, &ExternalToolRunner::finished, this, [this](const ToolResult &result) {
    poll_timer_.stop();
    if (!active_study_path_.isEmpty()) poll_job();
    active_study_path_.clear();
    controls_->setEnabled(true); cancel_->setEnabled(false);
    if (!pending_input_.isEmpty()) { QFile::remove(pending_input_); pending_input_.clear(); }
    auto callback = std::move(completion_); completion_ = {};
    if (cancel_requested_) { status_->setText(QStringLiteral("Canceled; completed evidence retained / 已取消，完成的证据保留")); return; }
    if (!result.ok) { error(result.error_summary); return; }
    if (active_command_.endsWith("-validate")) {
      const auto raw = result.output.toUtf8();
      const int first = raw.indexOf('{'), last = raw.lastIndexOf('}');
      const auto checked = first >= 0 && last > first
          ? QJsonDocument::fromJson(raw.mid(first, last - first + 1)).object() : QJsonObject();
      if (!checked.value("valid").toBool(false) || checked.value("state").toString() != "CURRENT") {
        validated_study_path_.clear(); emit markers_cleared(); results_->clear(); summary_->clear();
        error(QStringLiteral("Asset is STALE/INVALID or validation returned no current identity / 资产已过期或无效\n%1").arg(result.output));
        return;
      }
    }
    status_->setText(QStringLiteral("Command completed; asset validation passed / 操作完成，资产校验通过"));
    if (callback) callback();
  });
  poll_timer_.setInterval(600);
  connect(&poll_timer_, &QTimer::timeout, this, &StudyWorkflowDialog::poll_job);
  connect(&watcher_, &QFileSystemWatcher::fileChanged, this, [this](const QString &path) {
    validated_study_path_.clear();
    emit markers_cleared();
    if (busy() && !active_study_path_.isEmpty() && path != active_study_path_) {
      cancel_requested_ = true;
      runner_.cancel();
    }
    if (!busy()) {
      results_->clear(); summary_->clear();
      status_->setText(QStringLiteral("Inputs changed: validate again / 输入已变化，请重新校验"));
    }
  });
  for (auto *edit : {project_, source_, blocks_, topology_, query_set_, study_}) {
    connect(edit, &QLineEdit::textChanged, this, [this]() {
      validated_study_path_.clear(); results_->clear(); summary_->clear(); emit markers_cleared();
    });
  }
}

void StudyWorkflowDialog::set_inputs(const QString &source, const QString &blocks,
                                     const QString &topology, const QString &native_localizer) {
  if (busy()) return;
  source_->setText(source); blocks_->setText(blocks); topology_->setText(topology); native_->setText(native_localizer);
}
QString StudyWorkflowDialog::source_path() const { return source_->text(); }
void StudyWorkflowDialog::set_picked_point(double x, double y) {
  if (!busy()) { query_x_->setValue(x); query_y_->setValue(y); raise(); }
}
void StudyWorkflowDialog::error(const QString &message) {
  status_->setText(QStringLiteral("Unavailable / rejected / 操作未完成：%1").arg(message));
  log_->appendPlainText(message);
}
QString StudyWorkflowDialog::new_file(const QString &title, const QString &suggestion) {
  if (busy()) return {};
  const QString parent = project_->text().isEmpty() ? QDir::homePath() : QFileInfo(project_->text()).absolutePath();
  const QString path = QFileDialog::getSaveFileName(this, title, QDir(parent).filePath(suggestion), "YAML (*.yaml *.yml)");
  if (QFileInfo::exists(path)) { error(QStringLiteral("Use a new filename; existing revisions are preserved / 请使用新文件名")); return {}; }
  return path.isEmpty() ? QString() : QFileInfo(path).absoluteFilePath();
}
bool StudyWorkflowDialog::run(const QStringList &arguments, std::function<void()> completed,
                               const QString &study_job) {
  if (busy()) { error(QStringLiteral("An offline job is already running / 已有离线任务运行")); return false; }
  ToolInvocation invocation;
  invocation.label = QStringLiteral("Offline study: %1").arg(arguments.value(0));
  invocation.program = QStandardPaths::findExecutable("agt_mapstudio_workflow");
  invocation.arguments = arguments;
  if (invocation.program.isEmpty()) {
    if (!ExternalToolRunner::program_available("ros2") ||
        !ExternalToolRunner::ros2_executable_available("agt_map_localization_benchmark", "agt_mapstudio_workflow")) {
      error(QStringLiteral("Optional agt_mapstudio_workflow is not installed. Install/source the benchmark package; local editing remains available."));
      return false;
    }
    invocation = ExternalToolRunner::ros2_run(invocation.label, "agt_map_localization_benchmark", "agt_mapstudio_workflow", arguments);
  }
  invocation.cancellation_grace_ms = cancel_grace_->value();
  cancel_requested_ = false;
  completion_ = std::move(completed);
  active_study_path_ = study_job;
  active_command_ = arguments.value(0);
  controls_->setEnabled(false); cancel_->setEnabled(true);
  status_->setText(QStringLiteral("RUNNING / 运行中：%1").arg(invocation.label));
  runner_.start(invocation);
  if (!active_study_path_.isEmpty() && runner_.is_running()) poll_timer_.start();
  return true;
}
bool StudyWorkflowDialog::load_project_file(const QString &path) {
  if (busy()) return false;
  const auto absolute = QFileInfo(path).absoluteFilePath();
  return run({"project-validate", absolute}, [this, absolute]() { project_->setText(absolute); read_project(); });
}
void StudyWorkflowDialog::read_project() {
  try {
    const auto document = read(project_->text());
    source_->setText(ref_path(project_->text(), document["source_ref"]));
    blocks_->setText(ref_path(project_->text(), document["block_set_ref"]));
    topology_->setText(ref_path(project_->text(), document["annotation_ref"]));
    project_assets_->clear();
    for (const auto &key : {"query_sets", "studies"}) {
      const QString kind = QString(key) == "studies" ? "study" : "query_set";
      for (const auto &reference : document[key]) {
        const auto path = ref_path(project_->text(), reference);
        project_assets_->addItem(kind + " — " + QFileInfo(path).fileName(), path);
        project_assets_->setItemData(project_assets_->count() - 1, kind, Qt::UserRole + 1);
      }
    }
    watch_inputs();
  } catch (const std::exception &exception) { error(QString::fromUtf8(exception.what())); }
}
void StudyWorkflowDialog::attach_asset(const QString &path, bool study) {
  if (project_->text().isEmpty()) return;
  run({"project-add-assets", project_->text(), study ? "--study" : "--query-set", path}, [this]() { read_project(); });
}
void StudyWorkflowDialog::create_query_set(bool sample_rows) {
  if (!valid_id(query_id_->text())) { error("Query Set ID: ASCII letters, digits, underscore or hyphen, 1–80 characters"); return; }
  const auto output = new_file(QStringLiteral("New Query Set / 新点集"), query_id_->text() + "_draft.yaml");
  if (output.isEmpty()) return;
  QStringList args{sample_rows ? "query-set-sample" : "query-set-manual", "--id", query_id_->text(),
                   "--map-package", source_->text(), "--block-dir", blocks_->text(), "--output", output};
  if (!topology_->text().isEmpty() || sample_rows) args << "--topology" << topology_->text();
  if (sample_rows) args << "--ratios" << sample_ratios_->currentText();
  else {
    QTemporaryFile input(QDir::temp().filePath("agt_study_query_XXXXXX.json"));
    if (!input.open()) { error(input.errorString()); return; }
    const QJsonArray locations{QJsonObject{{"query_id", "Q001"}, {"scene", query_scene_->currentText()},
                                         {"xy", QJsonArray{query_x_->value(), query_y_->value()}}}};
    if (input.write(QJsonDocument(locations).toJson()) < 0 || !input.flush()) { error(input.errorString()); return; }
    pending_input_ = input.fileName(); input.setAutoRemove(false); input.close();
    args << "--locations-json" << pending_input_;
  }
  if (!run(args, [this, output]() { query_set_->setText(output); read_queries(); attach_asset(output, false); })) {
    if (!pending_input_.isEmpty()) { QFile::remove(pending_input_); pending_input_.clear(); }
  }
}
void StudyWorkflowDialog::validate_queries() {
  const auto path = query_set_->text();
  run({"query-set-validate", path}, [this, path]() { query_set_->setText(path); read_queries(); });
}
void StudyWorkflowDialog::read_queries() {
  try {
    const auto document = read(query_set_->text());
    queries_->clear();
    for (const auto &query : document["queries"]) {
      const auto xyz = query["resolved_position_m"];
      const QString position = xyz && xyz.size() >= 2 ? QStringLiteral("%1, %2").arg(xyz[0].as<double>()).arg(xyz[1].as<double>()) : "UNKNOWN";
      auto *item = new QTreeWidgetItem(queries_, {string(query, "query_id"), query["enabled"].as<bool>(true) ? "true" : "false",
          string(query, "scene"), string(query, "row_id", "UNKNOWN"), string(query, "resolved_keyframe_id", "UNKNOWN"),
          position, string(query, "snap_distance_m", "UNKNOWN")});
      item->setData(0, Qt::UserRole, string(query, "query_id"));
    }
    status_->setText(QStringLiteral("Query Set %1 — %2; requested points resolve to source observations / 请求点吸附到真实源观测")
                         .arg(string(document, "query_set_id"), string(document, "status")));
    watch_inputs();
  } catch (const std::exception &exception) { error(QString::fromUtf8(exception.what())); }
}
QString StudyWorkflowDialog::selected_query_id() const {
  return queries_->currentItem() ? queries_->currentItem()->data(0, Qt::UserRole).toString() : QString();
}
void StudyWorkflowDialog::edit_query(const QString &operation) {
  QStringList args;
  if (operation == "add") args = QStringList{"query-set-add-manual", query_set_->text(), "--scene", query_scene_->currentText(),
      "--xy", QString::number(query_x_->value(), 'g', 15), QString::number(query_y_->value(), 'g', 15)};
  else {
    const auto id = selected_query_id();
    if (id.isEmpty()) { error("Select a draft query / 请选择草稿查询点"); return; }
    args = QStringList{operation == "delete" ? "query-set-delete-draft-query" : "query-set-set-enabled", query_set_->text(), "--id", id};
    if (operation != "delete") args << "--enabled" << (operation == "enable" ? "true" : "false");
  }
  const auto path = query_set_->text();
  run(args, [this, path]() { read_queries(); attach_asset(path, false); });
}
void StudyWorkflowDialog::revise_query_set(bool freeze) {
  const auto output = new_file(freeze ? "Freeze Query Set / 冻结点集" : "Clone Query Set / 克隆点集", freeze ? "queries_frozen.yaml" : "queries_draft.yaml");
  if (output.isEmpty()) return;
  run({freeze ? "query-set-freeze" : "query-set-clone-draft", query_set_->text(), "--output", output},
      [this, output]() { query_set_->setText(output); read_queries(); attach_asset(output, false); });
}
void StudyWorkflowDialog::create_study() {
  if (!valid_id(study_id_->text())) { error("Study ID: ASCII letters, digits, underscore or hyphen, 1–80 characters"); return; }
  const auto parent = QFileDialog::getExistingDirectory(this, "New Study parent / 新研究父目录", QFileInfo(project_->text()).absolutePath());
  if (parent.isEmpty()) return;
  const auto directory = QDir(parent).filePath(study_id_->text());
  const auto path = QDir(directory).filePath("study.yaml");
  run({"study-create", "--id", study_id_->text(), "--query-set", query_set_->text(), "--output-dir", directory,
       "--frames", frames_->currentText(), "--candidate-top-k", QString::number(top_k_->value())},
      [this, path]() { study_->setText(path); read_study(); attach_asset(path, true); });
}
void StudyWorkflowDialog::validate_study() {
  const auto path = study_->text();
  validated_study_path_.clear(); emit markers_cleared();
  run({"study-validate", path, "--ros-install", ros_install_->text()}, [this, path]() {
    study_->setText(path); if (read_study()) { validated_study_path_ = path; preview_markers(); }
  });
}
void StudyWorkflowDialog::run_study() {
  if (busy()) return;
  const auto path = QFileInfo(study_->text()).absoluteFilePath();
  QStringList args{"run-study", path, "--ros-install", ros_install_->text()};
  if (!native_->text().isEmpty()) args << "--native-localizer-path" << native_->text();
  validated_study_path_.clear(); emit markers_cleared();
  run(args, [this, path]() { study_->setText(path); validate_study(); }, path);
}
void StudyWorkflowDialog::export_study() {
  const auto parent = QFileDialog::getExistingDirectory(this, "Report parent / 报告父目录", QFileInfo(study_->text()).absolutePath());
  if (parent.isEmpty()) return;
  const auto output = QDir(parent).filePath("report_" + QUuid::createUuid().toString(QUuid::WithoutBraces));
  run({"study-export", study_->text(), "--output-dir", output}, [this, output]() {
    status_->setText(QStringLiteral("Report exported / 报告已导出：%1").arg(output));
  });
}
bool StudyWorkflowDialog::read_study() {
  try {
    const auto document = read(study_->text());
    const auto source = ref_path(study_->text(), document["source_ref"]);
    if (!source.isEmpty()) source_->setText(source);
    blocks_->setText(ref_path(study_->text(), document["block_set_ref"]));
    topology_->setText(ref_path(study_->text(), document["annotation_ref"]));
    query_set_->setText(ref_path(study_->text(), document["query_set_ref"]));
    results_->clear();
    for (const auto &row : document["results"]) {
      const auto classification = string(row, "classification", "NO_DATA");
      const auto scene = string(row, "scene", "UNKNOWN");
      const auto frames = string(row, "query_accumulation_frames");
      if ((class_filter_->currentText() != "ALL" && class_filter_->currentText() != classification) ||
          (scene_filter_->currentText() != "ALL" && scene_filter_->currentText() != scene) ||
          (frame_filter_->currentText() != "ALL" && frame_filter_->currentText() != frames)) continue;
      const auto evidence = ref_path(study_->text(), row["evidence_ref"]);
      auto *item = new QTreeWidgetItem(results_, {string(row, "query_id"), scene, frames, classification, evidence});
      item->setData(0, Qt::UserRole, evidence);
    }
    const auto job = document["job"];
    QString text = QStringLiteral("%1 — %2 / %3; failed %4; reference %5. Labels are empirical; GICP convergence is separate.\n分类为观测结果，GICP收敛不等于定位正确。")
        .arg(string(document, "status"), string(job, "completed", "0"), string(job, "total", "0"),
             string(job, "failed", "0"), string(document, "reference_level", "UNKNOWN"));
    const auto summary_path = QDir(QFileInfo(study_->text()).absolutePath()).filePath("summary.yaml");
    if (QFileInfo::exists(summary_path)) text += "\n" + QString::fromStdString(YAML::Dump(read(summary_path)));
    summary_->setText(text.left(7000));
    watch_inputs();
    return true;
  } catch (const std::exception &exception) { error(QString::fromUtf8(exception.what())); return false; }
}
void StudyWorkflowDialog::preview_markers() {
  if (validated_study_path_.isEmpty()) return;
  emit markers_requested(validated_study_path_, source_->text(), class_filter_->currentText(), scene_filter_->currentText(), frame_filter_->currentText());
}
void StudyWorkflowDialog::poll_job() {
  if (active_study_path_.isEmpty()) return;
  try {
    const auto document = read(active_study_path_);
    const auto job = document["job"];
    status_->setText(QStringLiteral("%1: %2 / %3; failed %4; query %5 — %6")
        .arg(string(document, "status"), string(job, "completed", "0"), string(job, "total", "0"),
             string(job, "failed", "0"), string(job, "current_query"), active_study_path_));
  } catch (const std::exception &) { /* The worker may be atomically replacing its manifest. */ }
}
void StudyWorkflowDialog::watch_inputs() {
  if (!watcher_.files().isEmpty()) watcher_.removePaths(watcher_.files());
  QStringList paths{project_->text(), topology_->text(), query_set_->text(), study_->text(),
                    QDir(source_->text()).filePath("manifest.yaml"), QDir(source_->text()).filePath("checksums.sha256"),
                    QDir(source_->text()).filePath("map.pcd"), QDir(source_->text()).filePath("poses_timed.txt"),
                    QDir(source_->text()).filePath("metadata.yaml"),
                    QDir(blocks_->text()).filePath("manifest.yaml"), QDir(blocks_->text()).filePath("checksums.sha256")};
  for (const auto &path : paths) if (!path.isEmpty() && QFileInfo(path).isFile()) watcher_.addPath(path);
}
void StudyWorkflowDialog::closeEvent(QCloseEvent *event) {
  // Keep this dialog and its runner alive until a running worker has had its
  // configured cleanup grace. No QProcess child is destroyed mid-cleanup.
  if (busy()) { cancel_requested_ = true; runner_.cancel(); if (busy()) { event->ignore(); return; } }
  emit markers_cleared();
  event->accept();
}

}  // namespace agt_map_studio
