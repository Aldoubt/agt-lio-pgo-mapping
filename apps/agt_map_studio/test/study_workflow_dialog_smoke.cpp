// Explicit integration acceptance: installed *real* agt_mapstudio_workflow CLI
// and disposable verified source/block/topology fixtures required. No native
// localization worker is run; this verifies authoring and asset validation.
#include "ui/StudyWorkflowDialog.hpp"
#include <QApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QFileInfo>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QTabWidget>
#include <QThread>
#include <QTimer>
#include <QTreeWidget>
#include <iostream>
#include <stdexcept>

using agt_map_studio::StudyWorkflowDialog;
namespace {
void check(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
  std::cout << "PASS " << message << std::endl;
}
void wait(StudyWorkflowDialog &dialog) {
  QElapsedTimer elapsed; elapsed.start();
  while (dialog.busy() && elapsed.elapsed() < 45000) {
    QApplication::processEvents(); QThread::msleep(10);
  }
  QApplication::processEvents();
  check(!dialog.busy(), "real CLI completed within deadline");
  const auto status = dialog.findChild<QLabel *>("studyJobStatus")->text();
  if (status.startsWith("Unavailable")) throw std::runtime_error((status + "\n" + dialog.findChild<QPlainTextEdit *>()->toPlainText()).toStdString());
}
void choose_file_and_click(StudyWorkflowDialog &dialog, const char *button, const QString &path,
                           bool directory = false) {
  QTimer::singleShot(100, &dialog, [path, directory]() {
    for (auto *widget : QApplication::topLevelWidgets()) {
      if (auto *picker = qobject_cast<QFileDialog *>(widget)) {
        if (directory) picker->setDirectory(path);
        else {
          picker->setDirectory(QFileInfo(path).absolutePath());
          picker->selectFile(QFileInfo(path).fileName());
          if (auto *name = picker->findChild<QLineEdit *>("fileNameEdit")) name->setText(QFileInfo(path).fileName());
        }
        QMetaObject::invokeMethod(picker, "accept", Qt::QueuedConnection);
        return;
      }
    }
  });
  dialog.findChild<QPushButton *>(button)->click();
  wait(dialog);
}
}
int main(int argc, char **argv) {
  QApplication app(argc, argv);
  app.setQuitOnLastWindowClosed(false);
  QApplication::setAttribute(Qt::AA_DontUseNativeDialogs);
  if (argc != 3) { std::cerr << "usage: smoke PROJECT_YAML SCREENSHOT_PNG\n"; return 2; }
  try {
    const QString project = QFileInfo(argv[1]).absoluteFilePath();
    const QString directory = QFileInfo(project).absolutePath();
    StudyWorkflowDialog dialog; dialog.show();
    check(dialog.load_project_file(project), "start real project validator"); wait(dialog);
    dialog.findChild<QTabWidget *>()->setCurrentIndex(1);
    dialog.set_picked_point(4.2, .2);
    const auto manual = QDir(directory).filePath("gui_manual.yaml");
    choose_file_and_click(dialog, "studyNewManual", manual);
    check(QFileInfo::exists(manual), "GUI manual Query Set created by real CLI");
    check(dialog.findChild<QTreeWidget *>("studyQueriesTree")->topLevelItemCount() == 1,
          "requested map point resolves to one actual source observation");
    const auto frozen = QDir(directory).filePath("gui_manual_frozen.yaml");
    choose_file_and_click(dialog, "studyFreezeQueries", frozen);
    check(QFileInfo::exists(frozen), "GUI Query Set frozen into a new revision");
    dialog.findChild<QTabWidget *>()->setCurrentIndex(2);
    choose_file_and_click(dialog, "studyCreate", directory, true);
    const auto study = QDir(directory).filePath("greenhouse_study_v1/study.yaml");
    check(QFileInfo::exists(study), "GUI immutable Study created from frozen Query Set");
    dialog.findChild<QPushButton *>("studyValidate")->click(); wait(dialog);
    choose_file_and_click(dialog, "studyExport", directory, true);
    const auto reports = QDir(directory).entryList({"report_*"}, QDir::Dirs | QDir::NoDotAndDotDot);
    check(reports.size() == 1, "GUI report exported into a new directory");
    const auto report = QDir(directory).filePath(reports.front());
    for (const auto &name : {"queries.csv", "candidates.csv", "study_manifest.yaml"})
      check(QFileInfo::exists(QDir(report).filePath(name)), "existing report artifact written by real CLI");
    check(!QFileInfo::exists(QDir(report).filePath("summary.csv")),
          "queued Study has no fabricated execution summary");
    dialog.findChild<QTabWidget *>()->setCurrentIndex(1);
    const auto rows = QDir(directory).filePath("gui_rows.yaml");
    choose_file_and_click(dialog, "studySampleRows", rows);
    check(QFileInfo::exists(rows), "GUI reviewed-row Query Set uses exact frozen annotation");
    check(dialog.findChild<QTreeWidget *>("studyQueriesTree")->topLevelItemCount() == 5,
          "row entry/quarters/middle/exit remain in the Query Set");
    dialog.findChild<QPushButton *>("studyValidateQueries")->click(); wait(dialog);
    check(dialog.grab().save(argv[2]), "offscreen authoring screenshot saved");
    std::cout << "PASS real CLI authoring integration; native localization NOT_RUN\n";
    dialog.close();
  } catch (const std::exception &error) { std::cerr << "FAIL " << error.what() << '\n'; return 1; }
  return 0;
}
