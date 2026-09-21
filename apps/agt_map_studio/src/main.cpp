#include "ui/MainWindow.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <QApplication>
#include <QCommandLineOption>
#include <QCommandLineParser>
#include <QFileInfo>
#include <QMessageBox>

#include <exception>

int main(int argc, char **argv) {
  QApplication application(argc, argv);
  application.setApplicationName(QStringLiteral("agt_map_studio"));
  application.setApplicationVersion(QStringLiteral("0.3.0"));

  QCommandLineParser parser;
  parser.setApplicationDescription(
      QStringLiteral("AGT Map Studio: 3D point deletion, 2D navigation-map patching and map package publishing."));
  parser.addHelpOption();
  parser.addVersionOption();
  const QCommandLineOption pcd_option(
      {QStringLiteral("p"), QStringLiteral("pcd")},
      QStringLiteral("Open a PCD file on startup."), QStringLiteral("path"));
  parser.addOption(pcd_option);
  const QCommandLineOption map_option(
      {QStringLiteral("m"), QStringLiteral("map")},
      QStringLiteral("Open a Nav2 map.yaml on startup."), QStringLiteral("path"));
  parser.addOption(map_option);
  const QCommandLineOption package_option(
      {QStringLiteral("k"), QStringLiteral("package")},
      QStringLiteral("Open a mapping package or a maps/<id>/<version> map package directory."),
      QStringLiteral("dir"));
  parser.addOption(package_option);
  const QCommandLineOption session_option(
      {QStringLiteral("s"), QStringLiteral("session")},
      QStringLiteral("Restore a studio_session.yaml."), QStringLiteral("path"));
  parser.addOption(session_option);
  const QCommandLineOption review_package_option(
      QStringLiteral("review-package"),
      QStringLiteral("Open a mapping package in lightweight 2D review mode (PCD is not rendered)."),
      QStringLiteral("dir"));
  parser.addOption(review_package_option);
  const QCommandLineOption review_map_option(
      QStringLiteral("review-map"), QStringLiteral("Base map.yaml for lightweight 2D review."),
      QStringLiteral("path"));
  parser.addOption(review_map_option);
  const QCommandLineOption review_output_option(
      QStringLiteral("review-output"), QStringLiteral("Fixed output directory for the confirmed map."),
      QStringLiteral("dir"));
  parser.addOption(review_output_option);
  parser.addPositionalArgument(QStringLiteral("pcd"),
                               QStringLiteral("Optional positional PCD path."));
  parser.process(application);

  QString config_path;
  try {
    const std::string share =
        ament_index_cpp::get_package_share_directory("agt_map_studio");
    config_path = QString::fromStdString(share + "/config/config.yaml");
  } catch (const std::exception &) {
    // Running directly from a build tree is supported; defaults remain valid.
  }

  agt_map_studio::MainWindow window(config_path);
  const QString review_package = parser.value(review_package_option);
  const QString review_map = parser.value(review_map_option);
  const QString review_output = parser.value(review_output_option);
  const bool review_requested =
      !review_package.isEmpty() || !review_map.isEmpty() || !review_output.isEmpty();
  if (review_requested) {
    if (review_package.isEmpty() || review_map.isEmpty() || review_output.isEmpty()) {
      window.show();
      QMessageBox::critical(
          &window, QStringLiteral("Invalid review arguments"),
          QStringLiteral("--review-package, --review-map and --review-output must be used together."));
    } else {
      QString error;
      if (!window.open_mapping_review(QFileInfo(review_package).absoluteFilePath(),
                                      QFileInfo(review_map).absoluteFilePath(),
                                      QFileInfo(review_output).absoluteFilePath(), &error)) {
        window.show();
        QMessageBox::critical(&window, QStringLiteral("Open review failed"), error);
      }
    }
  }
  QString pcd_path = parser.value(pcd_option);
  if (pcd_path.isEmpty() && !parser.positionalArguments().isEmpty()) {
    pcd_path = parser.positionalArguments().first();
  }
  if (!review_requested && !pcd_path.isEmpty()) {
    QString error;
    if (!window.open_pcd(QFileInfo(pcd_path).absoluteFilePath(), &error)) {
      window.show();
      QMessageBox::critical(&window, QStringLiteral("Open PCD failed"), error);
    }
  }
  const QString package_path = parser.value(package_option);
  if (!review_requested && !package_path.isEmpty()) {
    QString error;
    if (!window.open_mapping_package(QFileInfo(package_path).absoluteFilePath(), &error)) {
      window.show();
      QMessageBox::critical(&window, QStringLiteral("Open package failed"), error);
    }
  }
  const QString session_path = parser.value(session_option);
  if (!review_requested && !session_path.isEmpty()) {
    QString error;
    if (!window.open_session(QFileInfo(session_path).absoluteFilePath(), &error)) {
      window.show();
      QMessageBox::critical(&window, QStringLiteral("Open session failed"), error);
    }
  }
  const QString map_path = parser.value(map_option);
  if (!review_requested && !map_path.isEmpty()) {
    QString error;
    if (!window.open_occupancy_map(QFileInfo(map_path).absoluteFilePath(), &error)) {
      window.show();
      QMessageBox::critical(&window, QStringLiteral("Open Occupancy Map failed"), error);
    }
  }
  window.show();
  return application.exec();
}
