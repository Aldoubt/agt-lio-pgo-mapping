#pragma once

#include <QObject>
#include <QProcess>
#include <QString>
#include <QStringList>

#include <functional>

namespace agt_map_studio {

// One asynchronous external command with captured output. The Studio never
// imports navigation Python; it only executes installed `ros2 run` tools.
struct ToolInvocation {
  QString label;
  QString program;
  QStringList arguments;
  QString working_directory;
  QString log_path;  // optional: append combined output here
};

struct ToolResult {
  bool ok = false;
  int exit_code = -1;
  QString output;
  QString error_summary;
};

class ExternalToolRunner : public QObject {
  Q_OBJECT

public:
  explicit ExternalToolRunner(QObject *parent = nullptr);

  // Build a `ros2 run <package> <executable> args...` invocation.
  static ToolInvocation ros2_run(const QString &label, const QString &package,
                                 const QString &executable, const QStringList &arguments);
  // Whether `program` can be found on PATH (or is an absolute executable).
  static bool program_available(const QString &program);
  // Whether an installed ROS executable resolves via `ros2 pkg prefix`. Cheap
  // heuristic: looks for lib/<package>/<executable> under AMENT_PREFIX_PATH.
  static bool ros2_executable_available(const QString &package, const QString &executable);

  bool is_running() const { return process_ != nullptr; }
  const ToolInvocation &current() const { return current_; }
  void start(const ToolInvocation &invocation);
  void cancel();

  // Synchronous convenience for short read-only probes.
  static ToolResult run_blocking(const ToolInvocation &invocation, int timeout_ms = 60000);

signals:
  void started(const QString &label);
  void output_appended(const QString &chunk);
  void finished(const ToolResult &result);

private:
  void append_log(const QString &text) const;

  QProcess *process_ = nullptr;
  ToolInvocation current_;
  QString buffer_;
};

}  // namespace agt_map_studio
