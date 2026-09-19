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
  application.setApplicationVersion(QStringLiteral("0.1.0"));

  QCommandLineParser parser;
  parser.setApplicationDescription(
      QStringLiteral("Offline PCD point-cloud viewer for AGT map packages."));
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
  QString pcd_path = parser.value(pcd_option);
  if (pcd_path.isEmpty() && !parser.positionalArguments().isEmpty()) {
    pcd_path = parser.positionalArguments().first();
  }
  if (!pcd_path.isEmpty()) {
    QString error;
    if (!window.open_pcd(QFileInfo(pcd_path).absoluteFilePath(), &error)) {
      window.show();
      QMessageBox::critical(&window, QStringLiteral("Open PCD failed"), error);
    }
  }
  const QString map_path = parser.value(map_option);
  if (!map_path.isEmpty()) {
    QString error;
    if (!window.open_occupancy_map(QFileInfo(map_path).absoluteFilePath(), &error)) {
      window.show();
      QMessageBox::critical(&window, QStringLiteral("Open Occupancy Map failed"), error);
    }
  }
  window.show();
  return application.exec();
}
