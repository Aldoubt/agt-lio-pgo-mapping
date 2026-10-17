// Standalone, real Qt Widgets/X11 software-acceptance harness.
// Only temporary fixtures in S1/runs; no ROS publishers or production assets.
#include "ui/MainWindow.hpp"
#include "occupancy/OccupancyViewer.hpp"
#include "selection/SelectionManager.h"
#include "workflow/WorkflowSession.hpp"

#include <QAction>
#include <QApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QLabel>
#include <QKeySequence>
#include <QMessageBox>
#include <QStatusBar>
#include <QSaveFile>
#include <QTimer>
#include <QUuid>
#include <QtTest/QTest>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace agt_map_studio;
namespace {
void check(bool condition, const QString &what) {
  if (!condition) throw std::runtime_error(what.toStdString());
  std::cout << "PASS " << what.toStdString() << std::endl;
}
void snap(QWidget &window, const QString &dir, const QString &name) {
  QApplication::processEvents();
  QTest::qWait(350);
  const QString file = QDir(dir).filePath(name);
  check(window.grab().save(file, "PNG"), QStringLiteral("Screenshot: %1").arg(file));
}
QAction *save_action(MainWindow &w) {
  for (QAction *a : w.findChildren<QAction*>())
    if (a->shortcut() == QKeySequence::Save) return a;
  return nullptr;
}
void save_with_ctrl_s(MainWindow &w, const QString &session_file) {
  check(save_action(w) != nullptr, QStringLiteral("Ctrl+S action available"));
  w.activateWindow();
  QApplication::processEvents();
  QTest::keyClick(&w, Qt::Key_S, Qt::ControlModifier);
  QApplication::processEvents();
  check(QFileInfo::exists(session_file), QStringLiteral("Ctrl+S wrote Studio session"));
}
int active_count(const QString &session_file) {
  WorkflowSession session;
  QString error;
  check(session.load(session_file, &error), QStringLiteral("Read saved session metadata: %1").arg(error));
  check(session.occupancy_draft_matches_disk(), QStringLiteral("Local 2D draft integrity"));
  const YAML::Node yaml = YAML::LoadFile(session.occupancy_draft_path().toStdString());
  int count=0;
  for (const auto &operation: yaml["operations"])
    if (!operation["undone"].as<bool>(false)) ++count;
  return count;
}
void close_with_discard_prompt(MainWindow &w) {
  QTimer::singleShot(200, []() {
    for (QWidget *widget : QApplication::topLevelWidgets()) {
      if (auto *message = qobject_cast<QMessageBox *>(widget)) {
        message->done(QMessageBox::No);
      }
    }
  });
  w.close();
  check(!w.isVisible(), QStringLiteral("Qt MapStudio window fully closed"));
}
void send_2d_edits(OccupancyViewer *v) {
  check(v && v->has_map(), QStringLiteral("Actual Qt OccupancyViewer has loaded PGM"));
  v->set_mode(OccupancyInteractionMode::Erase);
  // Actual Qt mouse press/release on the displayed 20x20 occupancy canvas.
  // Source fixture obstacle occupies grid [7..10]x[7..10].
  const double z = v->zoom();
  const QPointF origin((v->width()-v->map().width()*z)*0.5,
                       (v->height()-v->map().height()*z)*0.5);
  const auto pixel = [&](int x,int y) {
    return (origin+QPointF((x+0.5)*z,(v->map().height()-y-0.5)*z)).toPoint();
  };
  QTest::mousePress(v, Qt::LeftButton, Qt::NoModifier, pixel(7,7));
  QTest::mouseRelease(v, Qt::LeftButton, Qt::NoModifier, pixel(10,10));
  v->set_mode(OccupancyInteractionMode::Obstacle);
  v->obstacle_line_requested(0.5, 0.5, 1.5, 0.5, 0.2);
  v->set_mode(OccupancyInteractionMode::Forbidden);
  v->forbidden_polygon_requested({QPointF(2.5,2.5), QPointF(3.0,2.5), QPointF(3.0,3.0)});
  v->set_mode(OccupancyInteractionMode::View);
}
void alter_temporarily(const QString &file, MainWindow &w, const QString &session_file,
                       const QString &expected_text) {
  QFile f(file);
  check(f.open(QIODevice::ReadOnly), QStringLiteral("Read fixture for fail injection"));
  const QByteArray original=f.readAll(); f.close();
  check(f.open(QIODevice::WriteOnly), QStringLiteral("Open fixture for transient tamper"));
  f.write(original + "\n# transient-tamper\n");f.close();
  QString error;
  const bool restored = w.open_session(session_file, &error);
  check(f.open(QIODevice::WriteOnly), QStringLiteral("Restore original fixture content"));
  f.write(original); f.close();
  check(!restored && error.contains(expected_text),
        QStringLiteral("Fail-closed mutated source: %1").arg(error));
  auto *view=w.findChild<OccupancyViewer*>();
  check(view && view->has_map(),QStringLiteral("Existing 2D GUI remains loaded after rejection"));
}
} // namespace

int main(int argc,char **argv) {
  QApplication app(argc, argv);
  app.setApplicationName("AGT S1 Qt GUI acceptance");
  if (argc!=4) { std::cerr << "usage: <create|reopen|corrupt|interrupt|three_d> <fixture_dir> <screenshot_dir>\n"; return 2; }
  const QString mode=QString::fromLocal8Bit(argv[1]);
  const QString D=QString::fromLocal8Bit(argv[2]);
  const QString shots=QString::fromLocal8Bit(argv[3]);
  const QString session_file=QDir(D).filePath("map_studio/studio_session.yaml");
  try {
    MainWindow window{QString()};
    window.resize(1280, 780);
    window.show(); QApplication::processEvents();
    QString error;
    if (mode=="create") {
      check(window.open_pcd(QDir(D).filePath("map.pcd"), &error), QStringLiteral("GUI PCD load: %1").arg(error));
      check(window.open_occupancy_map(QDir(D).filePath("map.yaml"), &error), QStringLiteral("GUI PGM load: %1").arg(error));
      snap(window, shots, "02_opened_pcd_pgm.png");
      send_2d_edits(window.findChild<OccupancyViewer*>());
      snap(window, shots, "03_edited_pgm.png");
      check(QMetaObject::invokeMethod(&window,"undo_edit"),QStringLiteral("GUI Undo slot called"));
      check(QMetaObject::invokeMethod(&window,"redo_edit"),QStringLiteral("GUI Redo slot called"));
      save_with_ctrl_s(window, session_file);
      check(active_count(session_file)>=3, QStringLiteral("2D rectangle/obstacle/keepout edits saved"));
      snap(window, shots, "04_ctrl_s_saved_draft.png");
      close_with_discard_prompt(window);
    } else if (mode=="reopen") {
      check(window.open_session(session_file,&error),QStringLiteral("New process restores Studio DRAFT: %1").arg(error));
      check(active_count(session_file)>=3,QStringLiteral("Reloaded edit history remains intact"));
      check(window.findChild<OccupancyViewer*>()->has_map(),QStringLiteral("Reloaded occupancy map visibly present"));
      snap(window,shots,"05_new_process_reopened.png");
      check(QMetaObject::invokeMethod(&window,"undo_edit"),QStringLiteral("Undo after process restart"));
      check(window.statusBar()->currentMessage().contains("2D edits:"),QStringLiteral("Undo reports 2D change"));
      check(QMetaObject::invokeMethod(&window,"redo_edit"),QStringLiteral("Redo after process restart"));
      save_with_ctrl_s(window,session_file);
      check(active_count(session_file)>=3,QStringLiteral("Undo/Redo after restart preserves active edits"));
      snap(window,shots,"06_undo_redo_after_reopen.png");
      close_with_discard_prompt(window);
    } else if (mode=="corrupt") {
      check(window.open_session(session_file,&error),QStringLiteral("Load valid baseline before rejection tests: %1").arg(error));
      const int baseline_count=active_count(session_file);
      alter_temporarily(QDir(D).filePath("map.pgm"), window,session_file,"Base map");
      alter_temporarily(QDir(D).filePath("map.pcd"), window,session_file,"Source PCD");
      // A syntactically invalid 2D draft with a correctly updated digest tests
      // model preflight, not only checksum verification. Must not mutate GUI.
      WorkflowSession baseline;
      check(baseline.load(session_file,&error),QStringLiteral("Read current session for malformed draft fixture"));
      const QString invalid_dir=QDir(D).filePath("rejection_case");
      check(QDir().mkpath(invalid_dir),QStringLiteral("Create isolated malformed draft directory"));
      const QString invalid_draft=QDir(invalid_dir).filePath(QFileInfo(baseline.occupancy_draft_path()).fileName());
      QFile corrupt(invalid_draft);
      check(corrupt.open(QIODevice::WriteOnly),QStringLiteral("Open malformed draft fixture"));
      corrupt.write("version: 1\noperations: [broken\n");corrupt.close();
      baseline.set_occupancy_draft_sha256(WorkflowSession::sha256_file(invalid_draft));
      baseline.set_work_dir(invalid_dir);
      check(baseline.save(&error),QStringLiteral("Create consistent-checksum corrupted session: %1").arg(error));
      error.clear();
      check(!window.open_session(baseline.session_file(),&error) && error.contains("Invalid Studio DRAFT"),
            QStringLiteral("Malformed checksum-matching draft rejected BEFORE UI mutation: %1").arg(error));
      save_with_ctrl_s(window,session_file);
      check(active_count(session_file)==baseline_count, QStringLiteral("Existing editing state remains intact after all failed loads"));
      snap(window, shots,"07_source_and_draft_rejections.png");
      close_with_discard_prompt(window);
    } else if (mode=="interrupt") {
      WorkflowSession existing;
      check(existing.load(session_file,&error),QStringLiteral("Read uninterrupted session before fault simulation"));
      const int baseline_count=active_count(session_file);
      QFile session_bytes(session_file);
      check(session_bytes.open(QIODevice::ReadOnly),QStringLiteral("Read persisted session"));
      const QByteArray initial=session_bytes.readAll();session_bytes.close();
      const QString old_draft=existing.occupancy_draft_path();
      check(existing.occupancy_draft_matches_disk(),QStringLiteral("Original referenced draft valid"));
      const QString orphan=QDir(existing.work_dir()).filePath(
          QStringLiteral("occupancy_draft_%1.yaml").arg(QUuid::createUuid().toString(QUuid::WithoutBraces)));
      check(QFile::copy(old_draft,orphan),QStringLiteral("Write uncommitted new draft generation (simulated crash point)"));
      check(QFileInfo::exists(old_draft),QStringLiteral("Original draft still exists after uncommitted generation"));
      QSaveFile pending(session_file);
      check(pending.open(QIODevice::WriteOnly),QStringLiteral("Begin uncommitted atomic session write"));
      pending.write("schema_version: INTERRUPTED\n");
      pending.cancelWriting();
      check(session_bytes.open(QIODevice::ReadOnly),QStringLiteral("Read session after interrupted write"));
      const QByteArray after=session_bytes.readAll();session_bytes.close();
      check(after==initial,QStringLiteral("Atomic session metadata remains unchanged after cancelled write"));
      WorkflowSession retained;
      check(retained.load(session_file,&error) && retained.occupancy_draft_matches_disk(),
            QStringLiteral("Retained old session references verified old draft"));
      check(retained.occupancy_draft_path()==old_draft,
            QStringLiteral("Uncommitted generation cannot change recovered draft identity"));
      check(window.open_session(session_file,&error),
            QStringLiteral("GUI remains recoverable after simulated interrupted save"));
      check(active_count(session_file)==baseline_count,
            QStringLiteral("Interrupted save does not change recovered operations"));
      snap(window,shots,"09_interrupted_save_recovery.png");
      close_with_discard_prompt(window);
    } else if (mode=="three_d") {
      WorkflowSession base;
      check(base.load(session_file,&error),QStringLiteral("Load 2D session as starting point for 3D test"));
      const QString temp_dir=QDir(D).filePath("three_d_case");
      check(QDir().mkpath(temp_dir),QStringLiteral("Make 3D intent fixture directory"));
      const QString current2d=QFileInfo(base.occupancy_draft_path()).fileName();
      if (!base.occupancy_draft_sha256().isEmpty()) {
        QFile::remove(QDir(temp_dir).filePath(current2d));
        check(QFile::copy(base.occupancy_draft_path(), QDir(temp_dir).filePath(current2d)),
              QStringLiteral("Copy 2D draft into 3D intent fixture"));
      }
      SelectionManager selection;selection.reset(100);
      AxisAlignedBoundingBox bbox;
      bbox.valid=true; bbox.min=Eigen::Vector3f(0,0,0); bbox.max=Eigen::Vector3f(.3f,.3f,.2f);
      selection.select_points({0,1},bbox);
      check(selection.delete_selected(),QStringLiteral("Create genuine 3D deletion intent"));
      const QString rules=QDir(temp_dir).filePath("refinement.yaml");
      check(selection.write_refinement_rules(rules,base.source_pcd(),&error),
            QStringLiteral("Save 3D refinement rules: %1").arg(error));
      base.set_work_dir(temp_dir);
      base.set_refinement_rules_filename("refinement.yaml");
      base.set_refinement_fingerprint(selection.active_fingerprint());
      base.set_refinement_draft_sha256(WorkflowSession::sha256_file(rules));
      check(base.save(&error),QStringLiteral("Persist 3D-not-applied Studio DRAFT: %1").arg(error));
      check(window.open_session(base.session_file(),&error),QStringLiteral("GUI loads saved 3D intent: %1").arg(error));
      bool visible_marker=false;
      for (QLabel *l:window.findChildren<QLabel*>())
        if (l->text().contains("manual reapplication needed")) visible_marker=true;
      check(visible_marker,QStringLiteral("3D explicitly marked manual reapplication needed"));
      snap(window,shots,"08_3d_draft_unapplied.png");
      close_with_discard_prompt(window);
    } else throw std::runtime_error("Unknown mode");
  } catch(const std::exception &e) {std::cerr << "FAIL " << e.what() << std::endl;return 1;}
  return 0;
}
