#include "tools/ExternalToolRunner.hpp"

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QTimer>

TEST(ExternalToolRunnerTest, CancellationAllowsOwnedPlannerCleanupBeyondLegacyThreeSeconds) {
  int argc = 1;
  char name[] = "tool-cancellation-test";
  char *argv[] = {name, nullptr};
  QCoreApplication application(argc, argv);
  agt_map_studio::ExternalToolRunner runner;
  agt_map_studio::ToolInvocation command;
  command.label = "bounded cleanup fixture";
  command.program = "python3";
  command.arguments = QStringList{"-c", "import signal,time,sys\n"
      "def stop(*args):\n time.sleep(3.2)\n print('CLEANUP_COMPLETE',flush=True)\n sys.exit(0)\n"
      "signal.signal(signal.SIGTERM,stop)\nprint('READY_FOR_CANCEL',flush=True)\n"
      "while True: time.sleep(.1)\n"};
  command.cancellation_grace_ms = 10000;
  bool requested = false, finished = false;
  agt_map_studio::ToolResult outcome;
  QObject::connect(&runner, &agt_map_studio::ExternalToolRunner::output_appended,
      &application, [&](const QString &text) {
        // Ignore the echoed invocation; wait for the actual child handshake.
        if (!requested && text.trimmed() == "READY_FOR_CANCEL") {
          requested = true;
          QTimer::singleShot(0, &application, [&]() { runner.cancel(); });
        }
      });
  QObject::connect(&runner, &agt_map_studio::ExternalToolRunner::finished,
      &application, [&](const agt_map_studio::ToolResult &result) {
        finished = true;
        outcome = result;
        application.quit();
      });
  QTimer::singleShot(15000, &application, [&]() { runner.cancel(); application.quit(); });
  runner.start(command);
  application.exec();
  EXPECT_TRUE(requested);
  EXPECT_TRUE(finished);
  EXPECT_TRUE(outcome.ok) << outcome.error_summary.toStdString();
  EXPECT_NE(outcome.output.indexOf("CLEANUP_COMPLETE"), -1);
}
