#include "tools/ExternalToolRunner.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcessEnvironment>
#include <QStandardPaths>
#include <QTextStream>

namespace agt_map_studio {

ExternalToolRunner::ExternalToolRunner(QObject *parent) : QObject(parent) {}

ToolInvocation ExternalToolRunner::ros2_run(const QString &label, const QString &package,
                                            const QString &executable,
                                            const QStringList &arguments) {
  ToolInvocation invocation;
  invocation.label = label;
  invocation.program = QStringLiteral("ros2");
  invocation.arguments << QStringLiteral("run") << package << executable << arguments;
  return invocation;
}

bool ExternalToolRunner::program_available(const QString &program) {
  if (QFileInfo(program).isAbsolute()) return QFileInfo(program).isExecutable();
  return !QStandardPaths::findExecutable(program).isEmpty();
}

bool ExternalToolRunner::ros2_executable_available(const QString &package,
                                                   const QString &executable) {
  const QString prefixes = QProcessEnvironment::systemEnvironment().value(
      QStringLiteral("AMENT_PREFIX_PATH"));
  for (const QString &prefix : prefixes.split(':', Qt::SkipEmptyParts)) {
    const QString candidate = QDir(prefix).filePath(
        QStringLiteral("lib/%1/%2").arg(package, executable));
    if (QFileInfo(candidate).isExecutable()) return true;
  }
  return false;
}

void ExternalToolRunner::append_log(const QString &text) const {
  if (current_.log_path.isEmpty()) return;
  QFile file(current_.log_path);
  if (!file.open(QIODevice::Append | QIODevice::Text)) return;
  QTextStream stream(&file);
  stream << text;
}

void ExternalToolRunner::start(const ToolInvocation &invocation) {
  if (process_) return;
  current_ = invocation;
  buffer_.clear();
  process_ = new QProcess(this);
  process_->setProcessChannelMode(QProcess::MergedChannels);
  if (!invocation.working_directory.isEmpty()) {
    process_->setWorkingDirectory(invocation.working_directory);
  }
  connect(process_, &QProcess::readyReadStandardOutput, this, [this]() {
    const QString chunk = QString::fromUtf8(process_->readAllStandardOutput());
    buffer_ += chunk;
    append_log(chunk);
    emit output_appended(chunk);
  });
  connect(process_, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this,
          [this](int code, QProcess::ExitStatus status) {
            ToolResult result;
            result.exit_code = status == QProcess::NormalExit ? code : -1;
            result.ok = status == QProcess::NormalExit && code == 0;
            result.output = buffer_;
            if (!result.ok) {
              const QStringList lines = buffer_.split('\n', Qt::SkipEmptyParts);
              result.error_summary = lines.isEmpty()
                  ? QStringLiteral("%1 exited with %2").arg(current_.label).arg(result.exit_code)
                  : lines.mid(qMax(0, lines.size() - 6)).join('\n');
            }
            append_log(QStringLiteral("\n[%1] exit=%2\n").arg(current_.label).arg(result.exit_code));
            process_->deleteLater();
            process_ = nullptr;
            emit finished(result);
          });
  connect(process_, &QProcess::errorOccurred, this, [this](QProcess::ProcessError error) {
    if (error != QProcess::FailedToStart || !process_) return;
    ToolResult result;
    result.error_summary = QStringLiteral("Cannot start %1 (%2). Source the ROS 2 / workspace "
                                          "setup before launching the studio.")
                               .arg(current_.program, current_.label);
    result.output = buffer_;
    append_log(result.error_summary + '\n');
    process_->deleteLater();
    process_ = nullptr;
    emit finished(result);
  });
  const QString command_line = invocation.program + ' ' + invocation.arguments.join(' ');
  append_log(QStringLiteral("\n$ %1\n").arg(command_line));
  emit started(invocation.label);
  emit output_appended(QStringLiteral("$ %1\n").arg(command_line));
  process_->start(invocation.program, invocation.arguments);
}

void ExternalToolRunner::cancel() {
  if (!process_) return;
  process_->terminate();
  if (!process_->waitForFinished(3000)) process_->kill();
}

ToolResult ExternalToolRunner::run_blocking(const ToolInvocation &invocation, int timeout_ms) {
  QProcess process;
  process.setProcessChannelMode(QProcess::MergedChannels);
  if (!invocation.working_directory.isEmpty()) {
    process.setWorkingDirectory(invocation.working_directory);
  }
  process.start(invocation.program, invocation.arguments);
  ToolResult result;
  if (!process.waitForStarted(5000)) {
    result.error_summary = QStringLiteral("Cannot start %1").arg(invocation.program);
    return result;
  }
  if (!process.waitForFinished(timeout_ms)) {
    process.kill();
    result.error_summary = QStringLiteral("%1 timed out").arg(invocation.label);
    return result;
  }
  result.output = QString::fromUtf8(process.readAllStandardOutput());
  result.exit_code = process.exitStatus() == QProcess::NormalExit ? process.exitCode() : -1;
  result.ok = result.exit_code == 0;
  if (!result.ok) result.error_summary = result.output.right(600);
  return result;
}

}  // namespace agt_map_studio
