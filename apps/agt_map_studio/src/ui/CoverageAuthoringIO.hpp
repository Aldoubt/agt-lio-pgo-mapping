#pragma once

#include <QJsonObject>
#include <QJsonArray>
#include <QString>
#include <QStringList>

namespace agt_map_studio {

// Bridges reviewed annotation geometry to the authoring input. This does not
// validate vehicle traversability or produce a READY Route.
class CoverageAuthoringIO {
public:
  static bool structure_from_annotations(const QJsonObject &annotations,
                                         QJsonObject *geojson, QString *error);
  static bool export_structure(const QJsonObject &annotations,
                               const QString &source_file, const QString &map_yaml,
                               const QString &output,
                               QString *error,
                               const QJsonArray &map_from_reference = {},
                               bool transform_review_confirmed = false,
                               const QStringList &headland_annotation_ids = {});
  static bool read_document(const QString &path, QJsonObject *document, QString *error);
};

}  // namespace agt_map_studio
