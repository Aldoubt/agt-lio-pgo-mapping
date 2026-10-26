// Optional integration smoke: an installed reference backend is required.
// QT_QPA_PLATFORM=offscreen coverage_authoring_dialog_smoke \
//   /path/to/examples/rows.yaml /new/output [/path/to/screenshot.png]
#include "ui/CoverageAuthoringDialog.hpp"
#include "ui/CoverageAuthoringIO.hpp"

#include <QApplication>
#include <QDir>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTimer>

#include <iostream>

int main(int argc, char **argv) {
  QApplication application(argc, argv);
  if (argc < 3 || argc > 4) {
    std::cerr << "usage: coverage_authoring_dialog_smoke REQUEST NEW_OUTPUT [SCREENSHOT]\n";
    return 2;
  }
  agt_map_studio::CoverageAuthoringDialog dialog{QString(), QJsonObject(), QString()};
  QString error;
  QJsonObject request;
  if (!agt_map_studio::CoverageAuthoringIO::read_document(argv[1], &request, &error) ||
      request.value("backend").toString() != "reference" ||
      !dialog.load_request_file(argv[1], &error)) {
    std::cerr << "reference request required: " << error.toStdString() << '\n';
    return 2;
  }
  dialog.findChild<QLineEdit *>("coverageOutputPath")->setText(argv[2]);
  dialog.show();
  QTimer::singleShot(50, [&]() { dialog.findChild<QPushButton *>("coveragePlan")->click(); });
  QTimer poll;
  QObject::connect(&poll, &QTimer::timeout, [&]() {
    if (!dialog.findChild<QPushButton *>("coverageExportDraft")->isEnabled()) return;
    QJsonObject result;
    if (!agt_map_studio::CoverageAuthoringIO::read_document(QDir(argv[2]).filePath("plan.json"), &result, &error) ||
        !result.value("complete").toBool()) {
      std::cerr << "expected complete synthetic task: " << error.toStdString() << '\n';
      dialog.close();
      application.exit(3);
      return;
    }
    if (argc == 4 && !dialog.grab().save(argv[3])) {
      std::cerr << "cannot save screenshot\n";
      dialog.close();
      application.exit(3);
      return;
    }
    // A changed source selection must invalidate display and export, even
    // though the immutable output on disk remains untouched.
    dialog.findChild<QLineEdit *>("coverageMapPath")->setText(QStringLiteral("changed-input.yaml"));
    const bool stale_cleared = !dialog.findChild<QPushButton *>("coverageExportDraft")->isEnabled() &&
        dialog.findChild<QPlainTextEdit *>("coverageReport")->toPlainText().isEmpty() &&
        dialog.findChild<QGraphicsView *>("coveragePreview")->scene()->items().isEmpty();
    std::cout << (stale_cleared ? "PASS" : "FAIL") << ": planning subprocess, preview, input invalidation\n";
    dialog.close();
    application.exit(stale_cleared ? 0 : 3);
  });
  poll.start(100);
  QTimer::singleShot(150000, [&]() {
    std::cerr << dialog.findChild<QPlainTextEdit *>("coverageLog")->toPlainText().toStdString();
    dialog.close();
    application.exit(4);
  });
  return application.exec();
}
