// These are Qt/process lifecycle tests. The fixture command deliberately does
// not implement or validate localization algorithms/assets; real workflow CLI
// and native localization acceptance remain separate checks.
#include "ui/StudyWorkflowDialog.hpp"

#include <QApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStandardPaths>
#include <QTemporaryDir>
#include <QThread>
#include <gtest/gtest.h>

namespace {
QApplication &application() {
  static int argc = 1;
  static char name[] = "study-dialog-test";
  static char *argv[] = {name, nullptr};
  static QApplication app(argc, argv);
  return app;
}
bool wait_idle(agt_map_studio::StudyWorkflowDialog &dialog) {
  QElapsedTimer timer; timer.start();
  while (dialog.busy() && timer.elapsed() < 6000) {
    application().processEvents(); QThread::msleep(5);
  }
  application().processEvents();
  return !dialog.busy();
}
void write(const QString &path, const QByteArray &bytes) {
  QFile file(path); ASSERT_TRUE(file.open(QIODevice::WriteOnly));
  ASSERT_EQ(file.write(bytes), bytes.size());
}
struct Environment {
  QByteArray path = qgetenv("PATH"), ament = qgetenv("AMENT_PREFIX_PATH");
  ~Environment() { qputenv("PATH", path); qputenv("AMENT_PREFIX_PATH", ament); }
};
void install_fixture(const QTemporaryDir &directory) {
  const auto interpreter = QStandardPaths::findExecutable("python3");
  ASSERT_FALSE(interpreter.isEmpty());
  const auto file = directory.filePath("agt_mapstudio_workflow");
  write(file, "#!" + interpreter.toUtf8() + "\n"
      "import json,sys,time\n"
      "from pathlib import Path\n"
      "Path(__file__+'.args').write_text(json.dumps(sys.argv[1:]))\n"
      "if sys.argv[1]=='run-study': time.sleep(.25)\n"
      "print(json.dumps({'valid':True,'state':'STALE' if 'stale' in sys.argv[2] else 'CURRENT','status':'PASS'}))\n");
  ASSERT_TRUE(QFile::setPermissions(file, QFile::ReadOwner | QFile::WriteOwner | QFile::ExeOwner));
  qputenv("PATH", directory.path().toUtf8()); qputenv("AMENT_PREFIX_PATH", "");
}
}  // namespace

TEST(StudyWorkflowDialog, OptionalCommandAbsenceLeavesEditingAvailable) {
  application(); Environment environment; QTemporaryDir directory;
  qputenv("PATH", directory.path().toUtf8()); qputenv("AMENT_PREFIX_PATH", "");
  agt_map_studio::StudyWorkflowDialog dialog;
  EXPECT_FALSE(dialog.load_project_file(directory.filePath("project.yaml")));
  EXPECT_FALSE(dialog.busy());
  EXPECT_TRUE(dialog.findChild<QPushButton *>("studyCreateProject")->isEnabled());
  EXPECT_TRUE(dialog.findChild<QLabel *>("studyJobStatus")->text().contains("not installed"));
}

TEST(StudyWorkflowDialog, PathsAreSingleArgumentsAndBusyOpenCannotReplaceCurrentJob) {
  application(); Environment environment; QTemporaryDir directory;
  install_fixture(directory);
  const auto path = directory.filePath("project with spaces ; literal.yaml");
  write(path, "source_ref: {path: source}\nblock_set_ref: {path: blocks}\nquery_sets: []\nstudies: []\n");
  agt_map_studio::StudyWorkflowDialog dialog;
  ASSERT_TRUE(dialog.load_project_file(path));
  EXPECT_FALSE(dialog.load_project_file(directory.filePath("other.yaml")));
  ASSERT_TRUE(wait_idle(dialog));
  QFile args(directory.filePath("agt_mapstudio_workflow.args")); ASSERT_TRUE(args.open(QIODevice::ReadOnly));
  const auto argv = QJsonDocument::fromJson(args.readAll()).array();
  ASSERT_EQ(argv.size(), 2);
  EXPECT_EQ(argv[1].toString(), path);
  EXPECT_EQ(dialog.findChild<QLineEdit *>("studyProjectPath")->text(), path);
}

TEST(StudyWorkflowDialog, StaleWithZeroExitNeverBecomesValidatedPreview) {
  application(); Environment environment; QTemporaryDir directory; install_fixture(directory);
  agt_map_studio::StudyWorkflowDialog dialog;
  int previews = 0;
  QObject::connect(&dialog, &agt_map_studio::StudyWorkflowDialog::markers_requested,
                   &dialog, [&]() { ++previews; });
  dialog.findChild<QLineEdit *>("studyManifestPath")->setText(directory.filePath("stale.yaml"));
  dialog.findChild<QPushButton *>("studyValidate")->click();
  ASSERT_TRUE(wait_idle(dialog));
  EXPECT_EQ(previews, 0);
  EXPECT_TRUE(dialog.findChild<QLabel *>("studyJobStatus")->text().contains("STALE/INVALID"));
}

TEST(StudyWorkflowDialog, RunningStudyKeepsCapturedPathWhenSelectionChanges) {
  application(); Environment environment; QTemporaryDir directory; install_fixture(directory);
  const auto path = directory.filePath("study.yaml");
  write(path, "status: QUEUED\nsource_ref: {path: source}\nblock_set_ref: {path: blocks}\nquery_set_ref: {path: queries.yaml}\njob: {completed: 0, total: 1, failed: 0}\nresults: []\n");
  agt_map_studio::StudyWorkflowDialog dialog;
  auto *selected = dialog.findChild<QLineEdit *>("studyManifestPath");
  selected->setText(path);
  dialog.findChild<QPushButton *>("studyRun")->click();
  ASSERT_TRUE(dialog.busy());
  EXPECT_EQ(dialog.active_study_path(), path);
  selected->setText(directory.filePath("different-selection.yaml"));
  EXPECT_EQ(dialog.active_study_path(), path);
  EXPECT_FALSE(dialog.findChild<QPushButton *>("studyRun")->isEnabled());
  ASSERT_TRUE(wait_idle(dialog));
  EXPECT_EQ(selected->text(), path);
  EXPECT_TRUE(dialog.active_study_path().isEmpty());
}

TEST(StudyWorkflowDialog, MapPickRequestCarriesTheSelectedProjectSource) {
  application();
  agt_map_studio::StudyWorkflowDialog dialog;
  dialog.set_inputs("/source/current-project", "/blocks", "/topology.yaml", {});
  QString requested;
  QObject::connect(&dialog, &agt_map_studio::StudyWorkflowDialog::pick_requested,
                   &dialog, [&](const QString &source) { requested = source; });
  dialog.findChild<QPushButton *>("studyPickPoint")->click();
  EXPECT_EQ(requested, "/source/current-project");
  EXPECT_EQ(dialog.source_path(), requested);
}
