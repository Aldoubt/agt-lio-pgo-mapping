#include "annotation/ResearchAnnotationModel.hpp"

#include <gtest/gtest.h>

#include <QTemporaryDir>

using agt_map_studio::AxisAlignedBoundingBox;
using agt_map_studio::ResearchAnnotationModel;
using agt_map_studio::SelectionGeometry;

namespace {

QJsonObject document() {
  return QJsonObject{
      {"schema_id", "agt.research_annotations"}, {"schema_version", 1},
      {"dataset_id", "dataset-test"}, {"annotation_version", "annotation-test-v1"},
      {"review_status", "DRAFT"}, {"reference_frame", "map_reference/camera_init"},
      {"source_hashes", QJsonObject{{"session-a", QString(64, 'a')}, {"session-b", QString(64, 'b')}}},
      {"annotations", QJsonArray{}},
  };
}

QVector<QPointF> square() { return {{0, 0}, {4, 0}, {4, 3}, {0, 3}}; }

}  // namespace

TEST(ResearchAnnotationModelTest, PolygonCreateEditUndoRedoSaveReloadPreservesCoordinates) {
  ResearchAnnotationModel model;
  QString error;
  ASSERT_TRUE(model.load_document(document(), &error)) << error.toStdString();
  QString id;
  ASSERT_TRUE(model.add_polygon("greenhouse_boundary", square(), {"session-a", "session-b"}, {}, {}, &id, &error))
      << error.toStdString();
  EXPECT_TRUE(model.is_dirty());
  ASSERT_EQ(model.annotation_ids().size(), 1);
  ASSERT_TRUE(model.move_polygon_vertex(id, 1, QPointF(5, 0), &error)) << error.toStdString();
  auto geometry = model.annotations().first().toObject().value("geometry").toObject();
  EXPECT_DOUBLE_EQ(geometry.value("coordinates_xy_m").toArray()[1].toArray()[0].toDouble(), 5.0);
  ASSERT_TRUE(model.undo());
  geometry = model.annotations().first().toObject().value("geometry").toObject();
  EXPECT_DOUBLE_EQ(geometry.value("coordinates_xy_m").toArray()[1].toArray()[0].toDouble(), 4.0);
  ASSERT_TRUE(model.redo());
  ASSERT_TRUE(model.delete_polygon_vertex(id, 0, &error)) << error.toStdString();
  EXPECT_EQ(model.annotations().first().toObject().value("geometry").toObject()
                .value("coordinates_xy_m").toArray().size(), 3);
  ASSERT_TRUE(model.undo());
  EXPECT_EQ(model.annotations().first().toObject().value("geometry").toObject()
                .value("coordinates_xy_m").toArray().size(), 4);
  QTemporaryDir dir;
  ASSERT_TRUE(dir.isValid());
  const QString path = dir.filePath("annotation.json");
  ASSERT_TRUE(model.save_file(path, &error)) << error.toStdString();
  EXPECT_FALSE(model.is_dirty());
  ResearchAnnotationModel reloaded;
  ASSERT_TRUE(reloaded.load_file(path, &error)) << error.toStdString();
  EXPECT_EQ(reloaded.document(), model.document());
  EXPECT_EQ(reloaded.reference_frame(), "map_reference/camera_init");
}

TEST(ResearchAnnotationModelTest, InvalidSelfIntersectingPolygonIsRejected) {
  ResearchAnnotationModel model;
  ASSERT_TRUE(model.load_document(document()));
  QString error;
  const QVector<QPointF> bowtie{{0, 0}, {3, 3}, {0, 3}, {3, 0}};
  EXPECT_FALSE(model.add_polygon("greenhouse_boundary", bowtie, {"session-a"}, {}, {}, nullptr, &error));
  EXPECT_NE(error.indexOf("self-intersect"), -1);
  EXPECT_TRUE(model.annotations().isEmpty());
}

TEST(ResearchAnnotationModelTest, SavesAisleObstacleAndTraversableLabelsUsingCustomV1Extension) {
  ResearchAnnotationModel model;
  ASSERT_TRUE(model.load_document(document()));
  const QStringList labels{"Aisle", "Obstacle Region", "Traversable Region"};
  for (const auto &label : labels) {
    QString id;
    QString error;
    const QString notes = QStringLiteral("MapStudio UI type: %1").arg(label);
    ASSERT_TRUE(model.add_polygon("custom", square(), {"session-a", "session-b"}, {}, notes, &id, &error))
        << label.toStdString() << ": " << error.toStdString();
  }

  const auto annotations = model.annotations();
  ASSERT_EQ(annotations.size(), labels.size());
  for (int i = 0; i < annotations.size(); ++i) {
    EXPECT_EQ(annotations[i].toObject().value("annotation_type").toString(), "custom");
    EXPECT_EQ(annotations[i].toObject().value("notes").toString(),
              QStringLiteral("MapStudio UI type: %1").arg(labels[i]));
    EXPECT_EQ(annotations[i].toObject().value("geometry").toObject().value("kind").toString(),
              "polygon_xy");
  }
}

TEST(ResearchAnnotationModelTest, FreezeRequiresEveryFeatureReviewedAndThenLocksEdits) {
  ResearchAnnotationModel model;
  ASSERT_TRUE(model.load_document(document()));
  QString id;
  QString error;
  ASSERT_TRUE(model.add_polygon("navigation_interior", square(), {"session-a"}, {}, {}, &id, &error));
  EXPECT_FALSE(model.freeze("reviewer", "2026-10-07T00:00:00Z", &error));
  ASSERT_TRUE(model.set_annotation_review_status(id, "REVIEWED", &error)) << error.toStdString();
  EXPECT_EQ(model.review_status(), "REVIEWED");
  ASSERT_TRUE(model.freeze("researcher", "2026-10-07T00:00:00Z", &error)) << error.toStdString();
  EXPECT_EQ(model.review_status(), "FROZEN");
  EXPECT_FALSE(model.add_polygon("harvested_region", square(), {"session-a"}, {}, {}, nullptr, &error));
  EXPECT_NE(error.indexOf("immutable"), -1);
}

TEST(ResearchAnnotationModelTest, ReusesSelectionManagerGeometryForStableStructure) {
  ResearchAnnotationModel model;
  ASSERT_TRUE(model.load_document(document()));
  SelectionGeometry geometry;
  geometry.rule_type = "remove_polygon";
  geometry.polygon_xy = {{0, 0}, {3, 0}, {3, 2}, {0, 2}};
  geometry.has_z_range = true;
  geometry.z_min = 0.2;
  geometry.z_max = 2.8;
  QString id;
  QString error;
  ASSERT_TRUE(model.add_3d_selection("fixed_frame_region", geometry, {"session-a"}, "reviewed frame", &id, &error))
      << error.toStdString();
  const auto shape = model.annotations().first().toObject().value("geometry").toObject();
  EXPECT_EQ(shape.value("kind").toString(), "selection_3d");
  EXPECT_EQ(shape.value("shape").toString(), "polygon_prism");
  EXPECT_DOUBLE_EQ(shape.value("z_range_m").toArray()[0].toDouble(), 0.2);
}

TEST(ResearchAnnotationModelTest, CandidateIdentityPreventsDuplicateImports) {
  ResearchAnnotationModel model;
  ASSERT_TRUE(model.load_document(document()));
  QString id;
  ASSERT_TRUE(model.add_polygon("greenhouse_boundary", square(), {"session-a", "session-b"},
                                "greenhouse_boundary_candidate", {}, &id));
  EXPECT_TRUE(model.has_candidate("greenhouse_boundary_candidate"));
  EXPECT_FALSE(model.has_candidate("navigation_interior"));
}

TEST(ResearchAnnotationModelTest, AddsProvisionalAisleAndEndpointsAsOneUndoableV1Batch) {
  ResearchAnnotationModel model;
  ASSERT_TRUE(model.load_document(document()));
  agt_map_studio::AisleAnnotationProposal proposal;
  proposal.proposal_id = QStringLiteral("AUTO-A001");
  proposal.polygon_xy_m = {{0.5, 1.0}, {3.5, 1.0}, {3.5, 2.0}, {0.5, 2.0}};
  proposal.start_xy_m = QPointF(0.5, 1.5);
  proposal.end_xy_m = QPointF(3.5, 1.5);
  proposal.start_z_m = 0.1;
  proposal.end_z_m = 0.2;
  proposal.width_m = 1.0;
  proposal.length_m = 3.0;
  proposal.confidence = 0.8;
  QStringList created_ids;
  QString error;

  ASSERT_TRUE(model.add_aisle_proposals({proposal}, "session-a",
      QStringLiteral("generator: aisle-density-profile/v1"), &created_ids, &error)) << error.toStdString();
  ASSERT_EQ(created_ids.size(), 3);
  ASSERT_EQ(model.annotations().size(), 3);
  const auto aisle = model.annotations()[0].toObject();
  const auto start = model.annotations()[1].toObject();
  const auto end = model.annotations()[2].toObject();
  EXPECT_EQ(aisle.value("source_candidate_id").toString(), "AUTO-A001");
  EXPECT_EQ(aisle.value("review_status").toString(), "DRAFT");
  EXPECT_EQ(aisle.value("candidate_status").toString(), "PROVISIONAL");
  EXPECT_EQ(aisle.value("source_session_ids").toArray()[0].toString(), "session-a");
  EXPECT_EQ(start.value("source_candidate_id").toString(), "AUTO-A001-START");
  EXPECT_EQ(end.value("source_candidate_id").toString(), "AUTO-A001-END");
  EXPECT_EQ(start.value("geometry").toObject().value("kind").toString(), "point_xyz");
  EXPECT_EQ(end.value("geometry").toObject().value("kind").toString(), "point_xyz");
  EXPECT_EQ(start.value("candidate_status").toString(), "PROVISIONAL");
  EXPECT_EQ(end.value("candidate_status").toString(), "PROVISIONAL");
  EXPECT_FALSE(model.add_aisle_proposals({proposal}, "session-a", QStringLiteral("duplicate"), nullptr, &error));
  EXPECT_NE(error.indexOf(QStringLiteral("duplicate aisle proposal ID")), -1);
  EXPECT_EQ(model.annotations().size(), 3);

  ASSERT_TRUE(model.undo());
  EXPECT_TRUE(model.annotations().isEmpty());
  ASSERT_TRUE(model.redo());
  EXPECT_EQ(model.annotations().size(), 3);
}
