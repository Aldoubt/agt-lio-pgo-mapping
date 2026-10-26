#include "ui/CoverageAuthoringIO.hpp"

#include <gtest/gtest.h>

#include <QFile>
#include <QCryptographicHash>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryDir>

using agt_map_studio::CoverageAuthoringIO;

namespace {

QJsonObject fixture() {
  const QJsonObject boundary{
      {"annotation_id", "boundary"}, {"annotation_type", "greenhouse_boundary"},
      {"review_status", "REVIEWED"},
      {"geometry", QJsonObject{{"kind", "polygon_xy"},
          {"coordinates_xy_m", QJsonArray{QJsonArray{0, 0}, QJsonArray{10, 0}, QJsonArray{10, 10}, QJsonArray{0, 10}}}}}};
  const QJsonObject aisle{
      {"annotation_id", "corridor"}, {"annotation_type", "custom"}, {"review_status", "REVIEWED"},
      {"coverage", QJsonObject{{"role", "aisle"}, {"id", "row_1"}, {"endpoint_ids", QJsonArray{"a", "b"}}}},
      {"notes", "Display only; stale endpoint [100, 100] must never be consumed"},
      {"geometry", QJsonObject{{"kind", "polygon_xy"},
          {"coordinates_xy_m", QJsonArray{QJsonArray{1, 1}, QJsonArray{9, 1}, QJsonArray{9, 3}, QJsonArray{1, 3}}}}}};
  const auto endpoint = [](const QString &id, const QString &end, double x) {
    return QJsonObject{{"annotation_id", id}, {"annotation_type", "custom"}, {"review_status", "REVIEWED"},
        {"coverage", QJsonObject{{"role", "aisle_endpoint"}, {"aisle_id", "row_1"}, {"endpoint", end}}},
        {"geometry", QJsonObject{{"kind", "point_xyz"}, {"xyz_m", QJsonArray{x, 2.0, 0.0}}}}};
  };
  return {{"schema_id", "agt.research_annotations"}, {"schema_version", 1}, {"reference_frame", "map"},
          {"annotations", QJsonArray{boundary, aisle, endpoint("a", "start", 1.0), endpoint("b", "end", 9.0)}}};
}

void write(const QString &path, const QJsonObject &object) {
  QFile file(path);
  ASSERT_TRUE(file.open(QIODevice::WriteOnly));
  file.write(QJsonDocument(object).toJson());
}

}  // namespace

TEST(CoverageAuthoringIOTest, UsesExplicitIdentityAndCurrentReviewedGeometry) {
  auto source = fixture();
  auto entries = source.value("annotations").toArray();
  auto end = entries[3].toObject();
  end.insert("geometry", QJsonObject{{"kind", "point_xyz"}, {"xyz_m", QJsonArray{8.0, 2.0, 0.0}}});
  entries[3] = end;
  source.insert("annotations", entries);
  QJsonObject result;
  QString error;
  ASSERT_TRUE(CoverageAuthoringIO::structure_from_annotations(source, &result, &error)) << error.toStdString();
  EXPECT_EQ(result.value("frame_id").toString(), "map");
  const auto features = result.value("features").toArray();
  ASSERT_EQ(features.size(), 2);
  const auto row = features[1].toObject();
  EXPECT_EQ(row.value("properties").toObject().value("id").toString(), "row_1");
  const auto coordinates = row.value("geometry").toObject().value("coordinates").toArray();
  EXPECT_DOUBLE_EQ(coordinates[1].toArray()[0].toDouble(), 8.0);
  EXPECT_EQ(row.value("properties").toObject().value("corridor").toObject().value("type").toString(), "Polygon");
}

TEST(CoverageAuthoringIOTest, DoesNotRelabelResearchFrameOrAcceptUnreviewedEndpoints) {
  auto source = fixture();
  QJsonObject result;
  QString error;
  source.insert("reference_frame", "map_reference/camera_init");
  EXPECT_FALSE(CoverageAuthoringIO::structure_from_annotations(source, &result, &error));
  EXPECT_TRUE(error.contains("transform"));
  source = fixture();
  auto entries = source.value("annotations").toArray();
  auto endpoint = entries[2].toObject();
  endpoint.insert("review_status", "DRAFT");
  entries[2] = endpoint;
  source.insert("annotations", entries);
  EXPECT_FALSE(CoverageAuthoringIO::structure_from_annotations(source, &result, &error));
  EXPECT_TRUE(error.contains("unreviewed"));
}

TEST(CoverageAuthoringIOTest, DoesNotInferLegacyAislesFromNotes) {
  auto source = fixture();
  auto entries = source.value("annotations").toArray();
  auto aisle = entries[1].toObject();
  aisle.remove("coverage");
  aisle.insert("notes", "MapStudio UI type: Aisle\nproposal_id: row_1");
  entries[1] = aisle;
  source.insert("annotations", entries);
  QJsonObject result;
  ASSERT_TRUE(CoverageAuthoringIO::structure_from_annotations(source, &result, nullptr));
  EXPECT_EQ(result.value("features").toArray().size(), 1);
}

TEST(CoverageAuthoringIOTest, NavigationInteriorIsNotImplicitlyARequiredWorkArea) {
  auto source = fixture();
  auto entries = source.value("annotations").toArray();
  auto region = entries[0].toObject();
  region.insert("annotation_id", "nav_region");
  region.insert("annotation_type", "navigation_interior");
  entries.append(region);
  source.insert("annotations", entries);
  QJsonObject result;
  ASSERT_TRUE(CoverageAuthoringIO::structure_from_annotations(source, &result, nullptr));
  EXPECT_EQ(result.value("features").toArray().size(), 2);
}

TEST(CoverageAuthoringIOTest, RejectsDeletedOrCrossLinkedEndpoints) {
  auto source = fixture();
  auto entries = source.value("annotations").toArray();
  entries.removeAt(3);
  source.insert("annotations", entries);
  QJsonObject result;
  EXPECT_FALSE(CoverageAuthoringIO::structure_from_annotations(source, &result, nullptr));
  source = fixture();
  entries = source.value("annotations").toArray();
  auto endpoint = entries[3].toObject();
  auto metadata = endpoint.value("coverage").toObject();
  metadata.insert("aisle_id", "different_row");
  endpoint.insert("coverage", metadata);
  entries[3] = endpoint;
  source.insert("annotations", entries);
  EXPECT_FALSE(CoverageAuthoringIO::structure_from_annotations(source, &result, nullptr));
}

TEST(CoverageAuthoringIOTest, ExportBindsSavedSourceAndNeverOverwrites) {
  QTemporaryDir directory;
  ASSERT_TRUE(directory.isValid());
  const auto source = fixture();
  const QString source_path = directory.filePath("annotation.json");
  const QString map_path = directory.filePath("map.yaml");
  const QString image_path = directory.filePath("map.pgm");
  const QString output = directory.filePath("structure.geojson");
  write(source_path, source);
  write(map_path, QJsonObject{{"image", "map.pgm"}});
  QFile image(image_path);
  ASSERT_TRUE(image.open(QIODevice::WriteOnly));
  image.write("P2\n1 1\n255\n254\n");
  image.close();
  QString error;
  ASSERT_TRUE(CoverageAuthoringIO::export_structure(source, source_path, map_path, output, &error)) << error.toStdString();
  QJsonObject exported;
  ASSERT_TRUE(CoverageAuthoringIO::read_document(output, &exported, &error));
  EXPECT_TRUE(exported.value("source").toObject().value("annotation_sha256").toString().startsWith("sha256:"));
  EXPECT_TRUE(exported.value("source").toObject().value("map_image_sha256").toString().startsWith("sha256:"));
  EXPECT_TRUE(exported.value("source").toObject().value("map_yaml_sha256").toString().startsWith("sha256:"));
  EXPECT_FALSE(CoverageAuthoringIO::export_structure(source, source_path, map_path, output, &error));
  auto changed = source;
  changed.insert("annotation_version", "changed");
  EXPECT_FALSE(CoverageAuthoringIO::export_structure(changed, source_path, map_path, directory.filePath("new.geojson"), &error));
  EXPECT_TRUE(error.contains("Save"));
}

TEST(CoverageAuthoringIOTest, ReviewedTransformProjectsCopyAndRetainsOriginalProvenance) {
  QTemporaryDir directory;
  ASSERT_TRUE(directory.isValid());
  auto source = fixture();
  source.insert("reference_frame", "map_reference/camera_init");
  const QString source_path = directory.filePath("annotation.json");
  const QString map_path = directory.filePath("map.yaml");
  write(source_path, source);
  write(map_path, QJsonObject{{"image", "map.pgm"}});
  QFile image(directory.filePath("map.pgm"));
  ASSERT_TRUE(image.open(QIODevice::WriteOnly));
  image.write("P2\n1 1\n255\n254\n");
  image.close();
  QFile original(source_path);
  ASSERT_TRUE(original.open(QIODevice::ReadOnly));
  const QByteArray original_bytes = original.readAll();
  original.close();
  const QJsonArray transform{10.0, -2.0, 1.5707963267948966};
  QString error;
  const QString output = directory.filePath("projected.geojson");
  EXPECT_FALSE(CoverageAuthoringIO::export_structure(source, source_path, map_path, output, &error));
  EXPECT_FALSE(CoverageAuthoringIO::export_structure(source, source_path, map_path, output, &error, transform, false));
  ASSERT_TRUE(CoverageAuthoringIO::export_structure(source, source_path, map_path, output, &error, transform, true))
      << error.toStdString();
  QJsonObject exported;
  ASSERT_TRUE(CoverageAuthoringIO::read_document(output, &exported, &error));
  const auto row = exported.value("features").toArray()[1].toObject();
  const auto points = row.value("geometry").toObject().value("coordinates").toArray();
  EXPECT_NEAR(points[0].toArray()[0].toDouble(), 8.0, 1e-9);
  EXPECT_NEAR(points[0].toArray()[1].toDouble(), -1.0, 1e-9);
  EXPECT_NEAR(points[1].toArray()[0].toDouble(), 8.0, 1e-9);
  EXPECT_NEAR(points[1].toArray()[1].toDouble(), 7.0, 1e-9);
  const auto provenance = exported.value("source").toObject();
  EXPECT_EQ(provenance.value("original_frame").toString(), "map_reference/camera_init");
  EXPECT_EQ(provenance.value("map_from_reference").toArray(), transform);
  EXPECT_EQ(provenance.value("transform_review").toString(), "CONFIRMED");
  EXPECT_EQ(provenance.value("annotation_sha256").toString(), QStringLiteral("sha256:") +
      QString::fromLatin1(QCryptographicHash::hash(original_bytes, QCryptographicHash::Sha256).toHex()));
  ASSERT_TRUE(original.open(QIODevice::ReadOnly));
  EXPECT_EQ(original.readAll(), original_bytes);
  EXPECT_EQ(source.value("reference_frame").toString(), "map_reference/camera_init");
}

TEST(CoverageAuthoringIOTest, HeadlandSelectionIsExplicitAndDoesNotExpandRequiredWorkArea) {
  QTemporaryDir directory;
  auto source = fixture();
  auto entries = source.value("annotations").toArray();
  auto transition = entries[0].toObject();
  transition.insert("annotation_id", "turning_space");
  transition.insert("annotation_type", "transition_region");
  entries.append(transition);
  source.insert("annotations", entries);
  const QString source_path = directory.filePath("annotations.json"), map_path = directory.filePath("map.yaml");
  write(source_path, source);
  write(map_path, QJsonObject{{"image", "map.pgm"}});
  QFile image(directory.filePath("map.pgm"));
  ASSERT_TRUE(image.open(QIODevice::WriteOnly));
  image.write("P2\n1 1\n255\n254\n");
  image.close();
  QString error;
  const QString output = directory.filePath("headland.geojson");
  ASSERT_TRUE(CoverageAuthoringIO::export_structure(source, source_path, map_path, output, &error,
      {}, false, {"turning_space"})) << error.toStdString();
  QJsonObject exported;
  ASSERT_TRUE(CoverageAuthoringIO::read_document(output, &exported, &error));
  const auto features = exported.value("features").toArray();
  ASSERT_EQ(features.size(), 3);
  EXPECT_EQ(features[2].toObject().value("properties").toObject().value("feature_type").toString(), "headland");
  EXPECT_EQ(exported.value("source").toObject().value("headland_annotation_ids").toArray(), QJsonArray{"turning_space"});
  EXPECT_FALSE(CoverageAuthoringIO::export_structure(source, source_path, map_path, directory.filePath("bad.geojson"), &error,
      {1.0, 0.0, 0.0}, true));
  EXPECT_TRUE(error.contains("nonidentity"));
}
