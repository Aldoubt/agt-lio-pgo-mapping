#include "workflow/WorkflowSession.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QUuid>
#include <yaml-cpp/yaml.h>
#include <QTemporaryDir>

#include <gtest/gtest.h>

namespace agt_map_studio {
namespace {

QString write_file(const QTemporaryDir &dir, const QString &name, const QByteArray &data) {
  const QString path = dir.filePath(name);
  QFile file(path);
  EXPECT_TRUE(file.open(QIODevice::WriteOnly));
  file.write(data);
  file.close();
  return path;
}

}  // namespace

TEST(WorkflowSessionTest, StagesStartMissingAndBecomeFresh) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString pcd = write_file(dir, "map.pcd", "not really a pcd");
  WorkflowSession session;
  session.set_work_dir(dir.filePath("work"));
  session.reset(pcd, QString());
  EXPECT_FALSE(session.source_pcd_sha256().isEmpty());
  EXPECT_EQ(session.state(WorkflowSession::Navigation), StageState::Missing);
  EXPECT_EQ(session.effective_pcd(), pcd);

  const QString nav_dir = dir.filePath("nav");
  QDir().mkpath(nav_dir);
  session.mark_done(WorkflowSession::Navigation, nav_dir, "abc");
  EXPECT_EQ(session.state(WorkflowSession::Navigation), StageState::Fresh);
}

TEST(WorkflowSessionTest, ThreeDEditsInvalidateDownstreamStages) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString pcd = write_file(dir, "map.pcd", "source");
  WorkflowSession session;
  session.set_work_dir(dir.filePath("work"));
  session.reset(pcd, QString());
  const QString reloc = dir.filePath("reloc");
  const QString nav = dir.filePath("nav");
  QDir().mkpath(reloc);
  QDir().mkpath(nav);
  session.mark_done(WorkflowSession::Relocalization, reloc, session.source_pcd_sha256());
  session.mark_done(WorkflowSession::Navigation, nav, "pgm");
  EXPECT_EQ(session.state(WorkflowSession::Relocalization), StageState::Fresh);

  // The user deletes points: refine is now required and downstream is stale.
  session.set_refinement_fingerprint("edit-1");
  EXPECT_EQ(session.state(WorkflowSession::Refine), StageState::Missing);
  EXPECT_EQ(session.effective_pcd(), pcd) << "unapplied edits never change the effective PCD";

  const QString refined = dir.filePath("refined");
  QDir().mkpath(refined);
  write_file(dir, "refined/map.pcd", "refined-cloud");
  session.mark_done(WorkflowSession::Refine, refined, "refined-sha");
  EXPECT_EQ(session.state(WorkflowSession::Refine), StageState::Fresh);
  EXPECT_EQ(session.effective_pcd_sha256(), "refined-sha");
  EXPECT_EQ(session.state(WorkflowSession::Relocalization), StageState::Stale);
  EXPECT_EQ(session.state(WorkflowSession::Navigation), StageState::Stale);
  EXPECT_FALSE(session.blocking_reasons_for_publish().isEmpty());

  // Editing again after refine makes refine itself stale.
  session.set_refinement_fingerprint("edit-2");
  EXPECT_EQ(session.state(WorkflowSession::Refine), StageState::Stale);
  EXPECT_EQ(session.effective_pcd(), pcd);
}

TEST(WorkflowSessionTest, TwoDEditsOnlyInvalidatePatchAndPublish) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString pcd = write_file(dir, "map.pcd", "source");
  WorkflowSession session;
  session.set_work_dir(dir.filePath("work"));
  session.reset(pcd, QString());
  const QString nav = dir.filePath("nav");
  QDir().mkpath(nav);
  session.mark_done(WorkflowSession::Relocalization, nav, session.source_pcd_sha256());
  session.mark_done(WorkflowSession::Navigation, nav, "pgm");
  session.set_patch_fingerprint("p1");
  EXPECT_EQ(session.state(WorkflowSession::Navigation), StageState::Fresh);
  EXPECT_EQ(session.state(WorkflowSession::Patch), StageState::Missing);
  EXPECT_TRUE(session.effective_navigation_dir().isEmpty());
  const QString patched = dir.filePath("patched");
  QDir().mkpath(patched);
  session.mark_done(WorkflowSession::Patch, patched, "pgm2");
  EXPECT_EQ(session.effective_navigation_dir(), patched);
  session.publish_target().map_id = "m";
  session.publish_target().map_version = "v1";
  EXPECT_TRUE(session.blocking_reasons_for_publish().isEmpty());
}

TEST(WorkflowSessionTest, SaveAndLoadRoundTrip) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString pcd = write_file(dir, "map.pcd", "source");
  WorkflowSession session;
  session.set_work_dir(dir.filePath("work"));
  session.reset(pcd, QString());
  session.converter().resolution = 0.05;
  session.publish_target().map_id = "demo";
  session.publish_target().map_version = "v9";
  const QString nav = dir.filePath("nav");
  QDir().mkpath(nav);
  session.mark_done(WorkflowSession::Navigation, nav, "pgm", "note");
  QString error;
  ASSERT_TRUE(session.save(&error)) << error.toStdString();

  WorkflowSession restored;
  ASSERT_TRUE(restored.load(session.session_file(), &error)) << error.toStdString();
  EXPECT_EQ(restored.source_pcd(), pcd);
  EXPECT_DOUBLE_EQ(restored.converter().resolution, 0.05);
  EXPECT_EQ(restored.publish_target().map_id, "demo");
  EXPECT_EQ(restored.record(WorkflowSession::Navigation).path, nav);
  EXPECT_EQ(restored.state(WorkflowSession::Navigation), StageState::Fresh);
}

TEST(WorkflowSessionTest, DraftIdentityRoundTripAndModificationFailsClosed) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString yaml = write_file(dir, "map.yaml", "image: map.pgm\nresolution: 0.1\n");
  const QString image = write_file(dir, "map.pgm", "P5\n1 1\n255\n\x00");
  const QString source = write_file(dir, "source.pcd", "frozen-source");
  WorkflowSession session;
  session.set_work_dir(dir.filePath("draft"));
  session.reset(source, QString());
  session.set_occupancy_base(yaml, image);
  EXPECT_TRUE(session.source_matches_disk());
  EXPECT_TRUE(session.occupancy_base_matches_disk());
  QDir().mkpath(session.work_dir());
  write_file(dir, "draft/occupancy_draft.yaml", "version: 1\noperations: []\n");
  session.set_occupancy_draft_sha256(WorkflowSession::sha256_file(session.occupancy_draft_path()));
  QString error;
  ASSERT_TRUE(session.save(&error)) << error.toStdString();
  WorkflowSession reopened;
  ASSERT_TRUE(reopened.load(session.session_file(), &error)) << error.toStdString();
  EXPECT_EQ(reopened.occupancy_base_yaml(), QFileInfo(yaml).absoluteFilePath());
  EXPECT_EQ(reopened.occupancy_draft_sha256(), session.occupancy_draft_sha256());
  EXPECT_TRUE(reopened.source_matches_disk());
  EXPECT_TRUE(reopened.occupancy_base_matches_disk());
  EXPECT_TRUE(reopened.occupancy_draft_matches_disk());
  const QString rules = write_file(dir, "draft/refinement.yaml", "version: 1\noperations: []\n");
  session.set_refinement_draft_sha256(WorkflowSession::sha256_file(rules));
  ASSERT_TRUE(session.save(&error)) << error.toStdString();
  ASSERT_TRUE(reopened.load(session.session_file(), &error)) << error.toStdString();
  EXPECT_TRUE(reopened.refinement_draft_matches_disk());
  write_file(dir, "draft/refinement.yaml", "modified: true\n");
  EXPECT_FALSE(reopened.refinement_draft_matches_disk());

  write_file(dir, "draft/occupancy_draft.yaml", "changed: true\n");
  EXPECT_FALSE(reopened.occupancy_draft_matches_disk());
  write_file(dir, "map.pgm", "changed image");
  EXPECT_FALSE(reopened.occupancy_base_matches_disk());
  write_file(dir, "source.pcd", "changed pcd");
  EXPECT_FALSE(reopened.source_matches_disk());
}

TEST(WorkflowSessionTest, StandaloneOccupancyDraftAndNoReadyRouteIdentity) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString yaml = write_file(dir, "map.yaml", "image: map.pgm\n");
  const QString image = write_file(dir, "map.pgm", "P5\n");
  WorkflowSession session;
  session.set_work_dir(dir.filePath("work"));
  session.set_occupancy_base(yaml, image);
  QString error;
  ASSERT_TRUE(session.save(&error)) << error.toStdString();
  WorkflowSession reopened;
  ASSERT_TRUE(reopened.load(session.session_file(), &error)) << error.toStdString();
  EXPECT_TRUE(reopened.source_pcd().isEmpty());
  EXPECT_TRUE(reopened.has_occupancy_base());
  EXPECT_TRUE(reopened.occupancy_base_matches_disk());
  EXPECT_TRUE(reopened.occupancy_draft_matches_disk());
  EXPECT_FALSE(reopened.blocking_reasons_for_publish().isEmpty());
  QFile saved(session.session_file());
  ASSERT_TRUE(saved.open(QIODevice::ReadOnly));
  EXPECT_TRUE(saved.readAll().contains("DRAFT_NOT_READY_ROUTE"));
}

TEST(WorkflowSessionTest, VersionedDraftRevisionsDoNotOverwriteCommittedGeneration) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  WorkflowSession session;
  session.set_work_dir(dir.filePath("work"));
  ASSERT_TRUE(QDir().mkpath(session.work_dir()));
  const QString first = session.occupancy_draft_path();
  write_file(dir, "work/occupancy_draft.yaml", "operations: []\n");
  const QString first_sha = WorkflowSession::sha256_file(first);
  session.set_occupancy_draft_sha256(first_sha);
  QString error;
  ASSERT_TRUE(session.save(&error)) << error.toStdString();
  const QString old_session_hash = WorkflowSession::sha256_file(session.session_file());

  const QString uuid = QUuid::createUuid().toString(QUuid::WithoutBraces);
  const QString name = QStringLiteral("occupancy_draft_%1.yaml").arg(uuid);
  write_file(dir, QStringLiteral("work/%1").arg(name), "operations: [new]\n");
  // Simulate a process dying after writing a new draft but before committing
  // the session. The old session must still resolve its original revision.
  WorkflowSession reopened;
  ASSERT_TRUE(reopened.load(session.session_file(), &error)) << error.toStdString();
  EXPECT_EQ(reopened.occupancy_draft_path(), first);
  EXPECT_TRUE(reopened.occupancy_draft_matches_disk());
  EXPECT_EQ(WorkflowSession::sha256_file(session.session_file()), old_session_hash);
  EXPECT_EQ(WorkflowSession::sha256_file(first), first_sha);

  session.set_occupancy_draft_filename(name);
  session.set_occupancy_draft_sha256(WorkflowSession::sha256_file(session.occupancy_draft_path()));
  ASSERT_TRUE(session.save(&error)) << error.toStdString();
  ASSERT_TRUE(reopened.load(session.session_file(), &error)) << error.toStdString();
  EXPECT_EQ(reopened.occupancy_draft_path(), session.occupancy_draft_path());
  EXPECT_TRUE(reopened.occupancy_draft_matches_disk());
  EXPECT_EQ(WorkflowSession::sha256_file(first), first_sha);
}

TEST(WorkflowSessionTest, SessionRejectsUnsafeDraftRelativeFileNames) {
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  WorkflowSession session;
  session.set_work_dir(dir.filePath("work"));
  QString error;
  ASSERT_TRUE(session.save(&error)) << error.toStdString();
  YAML::Node root = YAML::LoadFile(session.session_file().toStdString());
  root["occupancy_draft"]["draft_file"] = "../../../../etc/passwd";
  const QString tampered = write_file(dir, "unsafe.yaml", QByteArray(YAML::Dump(root).c_str()));
  WorkflowSession rejected;
  EXPECT_FALSE(rejected.load(tampered, &error));
  EXPECT_TRUE(error.contains("Invalid Studio-local draft filename")) << error.toStdString();
}

}  // namespace agt_map_studio
