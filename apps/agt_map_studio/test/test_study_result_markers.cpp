#include "ui/StudyResultMarkers.hpp"

#include <QTemporaryDir>
#include <QFile>

#include <gtest/gtest.h>

namespace agt_map_studio {
namespace {

bool write_file(const QString &path, const QByteArray &contents) {
  QFile file(path);
  return file.open(QIODevice::WriteOnly | QIODevice::Truncate) &&
         file.write(contents) == contents.size();
}

}  // namespace

TEST(StudyResultMarkers, LoadsCategorizedQueryPositionsAndAppliesFilters) {
  QTemporaryDir directory;
  ASSERT_TRUE(directory.isValid());
  const QString queries_path = directory.filePath(QStringLiteral("queries.yaml"));
  const QString study_path = directory.filePath(QStringLiteral("study.yaml"));
  ASSERT_TRUE(write_file(queries_path, R"YAML(queries:
  - query_id: Q_CORRECT
    resolved_position_m: [1.0, 2.0, 0.1]
  - query_id: Q_FALSE_ACCEPT
    resolved_position_m: [3.0, 4.0, 0.2]
  - query_id: Q_REJECTED
    resolved_position_m: [5.0, 6.0, 0.3]
  - query_id: Q_TIMEOUT
    resolved_position_m: [7.0, 8.0, 0.4]
  - query_id: Q_NO_DATA
    resolved_position_m: [9.0, 10.0, 0.5]
)YAML"));
  ASSERT_TRUE(write_file(study_path, R"YAML(query_set_ref:
  kind: filesystem
  path: queries.yaml
results:
  - {query_id: Q_CORRECT, classification: CORRECT, scene: ENTRY, query_accumulation_frames: 1}
  - {query_id: Q_FALSE_ACCEPT, classification: FALSE_ACCEPT, scene: MIDDLE, query_accumulation_frames: 3}
  - {query_id: Q_REJECTED, classification: REJECTED, scene: EXIT, query_accumulation_frames: 5}
  - {query_id: Q_TIMEOUT, classification: TIMEOUT, scene: HEADLAND, query_accumulation_frames: 1}
  - {query_id: Q_NO_DATA, classification: NO_DATA, scene: OTHER, query_accumulation_frames: 3}
  - {query_id: Q_NOT_RUN, classification: NOT_RUN, scene: OTHER, query_accumulation_frames: 3}
)YAML"));

  StudyResultMarkerCloud markers;
  QString error;
  ASSERT_TRUE(load_study_result_markers(study_path, QStringLiteral("ALL"),
                                        QStringLiteral("ALL"), QStringLiteral("ALL"),
                                        &markers, &error)) << error.toStdString();
  EXPECT_EQ(markers.cloud.point_count(), 5U);
  EXPECT_EQ(markers.colors.size(), 5U);
  EXPECT_FLOAT_EQ(markers.cloud.xyz[0], 1.0F);
  EXPECT_FLOAT_EQ(markers.cloud.xyz[3], 3.0F);
  EXPECT_NE(markers.colors[0], markers.colors[1]);
  EXPECT_NE(markers.colors[1], markers.colors[2]);
  EXPECT_NE(markers.colors[2], markers.colors[3]);
  EXPECT_NE(markers.colors[3], markers.colors[4]);

  ASSERT_TRUE(load_study_result_markers(study_path, QStringLiteral("FALSE_ACCEPT"),
                                        QStringLiteral("MIDDLE"), QStringLiteral("3"),
                                        &markers, &error)) << error.toStdString();
  ASSERT_EQ(markers.cloud.point_count(), 1U);
  EXPECT_FLOAT_EQ(markers.cloud.xyz[0], 3.0F);
  EXPECT_FLOAT_EQ(markers.cloud.xyz[1], 4.0F);
  EXPECT_FLOAT_EQ(markers.cloud.xyz[2], 0.2F);
}

TEST(StudyResultMarkers, RejectsUnsupportedReferenceKind) {
  QTemporaryDir directory;
  ASSERT_TRUE(directory.isValid());
  const QString study_path = directory.filePath(QStringLiteral("study.yaml"));
  ASSERT_TRUE(write_file(study_path, R"YAML(query_set_ref:
  kind: package
  path: queries.yaml
results: []
)YAML"));

  StudyResultMarkerCloud markers;
  QString error;
  EXPECT_FALSE(load_study_result_markers(study_path, QStringLiteral("ALL"),
                                         QStringLiteral("ALL"), QStringLiteral("ALL"),
                                         &markers, &error));
  EXPECT_NE(error.indexOf(QStringLiteral("no supported Query Set reference")), -1);
}

}  // namespace agt_map_studio
