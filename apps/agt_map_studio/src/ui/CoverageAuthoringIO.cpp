#include "ui/CoverageAuthoringIO.hpp"

#include <QCryptographicHash>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QJsonArray>
#include <QJsonDocument>
#include <QTemporaryFile>
#include <QSet>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <exception>
#include <stdexcept>

namespace agt_map_studio {
namespace {

QJsonValue yaml_json(const YAML::Node &value) {
  if (!value || value.IsNull()) return QJsonValue();
  if (value.IsSequence()) {
    QJsonArray array;
    for (const auto &entry : value) array.push_back(yaml_json(entry));
    return array;
  }
  if (value.IsMap()) {
    QJsonObject object;
    for (const auto &entry : value) {
      const QString key = QString::fromStdString(entry.first.as<std::string>());
      if (object.contains(key)) throw std::runtime_error("duplicate document key");
      object.insert(key, yaml_json(entry.second));
    }
    return object;
  }
  const std::string scalar = value.Scalar();
  if (value.Tag() != "!") {
    if (scalar == "true") return true;
    if (scalar == "false") return false;
    bool ok = false;
    const double number = QString::fromStdString(scalar).toDouble(&ok);
    if (ok && std::isfinite(number)) return number;
  }
  return QString::fromStdString(scalar);
}

bool reviewed(const QJsonObject &item) {
  const QString status = item.value("review_status").toString();
  return status == "REVIEWED" || status == "FROZEN";
}

bool coordinate(const QJsonValue &value, int count) {
  const auto array = value.toArray();
  if (array.size() != count) return false;
  for (const auto &part : array) {
    if (!part.isDouble() || !std::isfinite(part.toDouble())) return false;
  }
  return true;
}

QJsonObject polygon(const QJsonObject &item) {
  const auto geometry = item.value("geometry").toObject();
  if (geometry.value("kind").toString() != "polygon_xy") return {};
  QJsonArray ring = geometry.value("coordinates_xy_m").toArray();
  if (ring.size() < 3) return {};
  for (const auto &point : ring) if (!coordinate(point, 2)) return {};
  if (ring.first() != ring.last()) ring.push_back(ring.first());
  return {{"type", "Polygon"}, {"coordinates", QJsonArray{ring}}};
}

bool transform_copy(QJsonObject *document, const QJsonArray &transform,
                    bool confirmed, QString *error) {
  const QString frame = document->value("reference_frame").toString();
  if (frame == "map" && transform.isEmpty()) return true;
  if (!confirmed || !coordinate(transform, 3) || frame.isEmpty()) {
    if (error) *error = QStringLiteral("A non-map reference frame requires an explicit reviewed finite [dx,dy,yaw] transform.");
    return false;
  }
  if (frame == "map" && (transform[0].toDouble() != 0.0 || transform[1].toDouble() != 0.0 || transform[2].toDouble() != 0.0)) {
    if (error) *error = QStringLiteral("Annotations already in map cannot receive a nonidentity source-to-map transform.");
    return false;
  }
  const double dx = transform[0].toDouble(), dy = transform[1].toDouble();
  const double c = std::cos(transform[2].toDouble()), s = std::sin(transform[2].toDouble());
  const auto xy = [dx, dy, c, s](const QJsonArray &point) {
    return QJsonArray{dx + c * point[0].toDouble() - s * point[1].toDouble(),
                      dy + s * point[0].toDouble() + c * point[1].toDouble()};
  };
  auto entries = document->value("annotations").toArray();
  for (int index = 0; index < entries.size(); ++index) {
    auto item = entries[index].toObject();
    auto geometry = item.value("geometry").toObject();
    const QString kind = geometry.value("kind").toString();
    if (kind == "polygon_xy" || kind == "polyline_xy") {
      QJsonArray points;
      for (const auto &value : geometry.value("coordinates_xy_m").toArray()) {
        if (!coordinate(value, 2)) {
          if (error) *error = QStringLiteral("Cannot transform malformed XY annotation geometry.");
          return false;
        }
        points.push_back(xy(value.toArray()));
      }
      geometry.insert("coordinates_xy_m", points);
    } else if (kind == "point_xyz") {
      const auto point = geometry.value("xyz_m").toArray();
      if (!coordinate(point, 3)) {
        if (error) *error = QStringLiteral("Cannot transform malformed endpoint geometry.");
        return false;
      }
      QJsonArray result = xy(point);
      result.push_back(point[2]);
      geometry.insert("xyz_m", result);
    }
    item.insert("geometry", geometry);
    entries[index] = item;
  }
  document->insert("annotations", entries);
  document->insert("reference_frame", "map");
  return true;
}

}  // namespace

bool CoverageAuthoringIO::read_document(const QString &path, QJsonObject *document,
                                       QString *error) {
  QFile file(path);
  if (!document || !file.open(QIODevice::ReadOnly) || file.size() > 32 * 1024 * 1024) {
    if (error) *error = QStringLiteral("Cannot read document (maximum 32 MiB): %1").arg(path);
    return false;
  }
  const QByteArray contents = file.readAll();
  QJsonParseError parse_error;
  const auto json = QJsonDocument::fromJson(contents, &parse_error);
  if (parse_error.error == QJsonParseError::NoError && json.isObject()) {
    *document = json.object();
    return true;
  }
  try {
    const auto result = yaml_json(YAML::Load(contents.toStdString()));
    if (!result.isObject()) throw std::runtime_error("document must be an object");
    *document = result.toObject();
    return true;
  } catch (const std::exception &exception) {
    if (error) *error = QStringLiteral("Invalid document %1: %2").arg(path, exception.what());
    return false;
  }
}

bool CoverageAuthoringIO::structure_from_annotations(const QJsonObject &document,
                                                    QJsonObject *geojson,
                                                    QString *error) {
  const auto fail = [error](const QString &message) {
    if (error) *error = message;
    return false;
  };
  if (!geojson || document.value("schema_id").toString() != "agt.research_annotations" ||
      document.value("schema_version").toInt() != 1 ||
      document.value("reference_frame").toString() != "map") {
    return fail(QStringLiteral("Coverage export requires Research Annotation V1 in frame 'map'. "
        "A research reference frame needs an explicit reviewed transform; it cannot be relabeled."));
  }
  QJsonObject by_id;
  const auto annotations = document.value("annotations").toArray();
  for (const auto &value : annotations) {
    const auto item = value.toObject();
    const QString id = item.value("annotation_id").toString();
    if (id.isEmpty() || by_id.contains(id)) return fail(QStringLiteral("Missing or duplicate annotation ID."));
    by_id.insert(id, item);
  }
  int boundaries = 0;
  QSet<QString> physical_ids;
  QJsonArray features;
  for (const auto &value : annotations) {
    const auto item = value.toObject();
    const auto coverage = item.value("coverage").toObject();
    const QString role = coverage.value("role").toString();
    const QString type = item.value("annotation_type").toString();
    QString feature_type;
    if (type == "greenhouse_boundary") feature_type = "field_boundary";
    else if (role == "keepout" || role == "work_area" || role == "headland") feature_type = role;
    else if (role == "aisle") feature_type = "aisle";
    else continue;
    if (!reviewed(item)) return fail(QStringLiteral("Review annotation %1 before coverage export.")
        .arg(item.value("annotation_id").toString()));
    const QString id = coverage.value("id").toString(item.value("annotation_id").toString());
    if (id.isEmpty() || physical_ids.contains(id)) return fail(QStringLiteral("Duplicate physical feature ID: %1").arg(id));
    physical_ids.insert(id);
    QJsonObject properties{{"id", id}, {"feature_type", feature_type}, {"review_status", "CONFIRMED"},
                           {"source_annotation_id", item.value("annotation_id")}};
    QJsonObject geometry = polygon(item);
    if (feature_type == "aisle") {
      const auto endpoints = coverage.value("endpoint_ids").toArray();
      if (geometry.isEmpty() || endpoints.size() != 2 || endpoints[0] == endpoints[1]) {
        return fail(QStringLiteral("Aisle %1 needs a polygon and two explicit endpoint references.").arg(id));
      }
      QJsonArray line;
      for (int index = 0; index < 2; ++index) {
        const auto endpoint = by_id.value(endpoints[index].toString()).toObject();
        const auto metadata = endpoint.value("coverage").toObject();
        const auto shape = endpoint.value("geometry").toObject();
        if (!reviewed(endpoint) || metadata.value("role").toString() != "aisle_endpoint" ||
            metadata.value("aisle_id").toString() != id ||
            metadata.value("endpoint").toString() != (index == 0 ? "start" : "end") ||
            shape.value("kind").toString() != "point_xyz" || !coordinate(shape.value("xyz_m"), 3)) {
          return fail(QStringLiteral("Aisle %1 has a missing, unreviewed or mismatched endpoint.").arg(id));
        }
        const auto xyz = shape.value("xyz_m").toArray();
        line.push_back(QJsonArray{xyz[0], xyz[1]});
      }
      if (line[0] == line[1]) return fail(QStringLiteral("Aisle %1 has zero length.").arg(id));
      properties.insert("corridor", geometry);
      properties.insert("allowed_direction", coverage.value("allowed_direction").toString("bidirectional"));
      geometry = {{"type", "LineString"}, {"coordinates", line}};
    }
    if (geometry.isEmpty()) return fail(QStringLiteral("Feature %1 needs a finite XY polygon.").arg(id));
    if (feature_type == "field_boundary") ++boundaries;
    features.push_back(QJsonObject{{"type", "Feature"}, {"properties", properties}, {"geometry", geometry}});
  }
  if (boundaries != 1) return fail(QStringLiteral("Coverage export requires exactly one reviewed greenhouse boundary."));
  *geojson = {{"type", "FeatureCollection"}, {"frame_id", "map"},
              {"review", QJsonObject{{"status", "CONFIRMED"}}}, {"features", features}};
  return true;
}

bool CoverageAuthoringIO::export_structure(const QJsonObject &annotations,
                                          const QString &source_file,
                                          const QString &map_yaml,
                                          const QString &output, QString *error,
                                          const QJsonArray &map_from_reference,
                                          bool transform_review_confirmed,
                                          const QStringList &headland_annotation_ids) {
  QFile source_input(source_file);
  if (!source_input.open(QIODevice::ReadOnly) || source_input.size() > 32 * 1024 * 1024) {
    if (error) *error = QStringLiteral("Cannot read the saved annotation JSON: %1").arg(source_file);
    return false;
  }
  const QByteArray source_bytes = source_input.readAll();
  QJsonParseError parse_error;
  const auto source_json = QJsonDocument::fromJson(source_bytes, &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !source_json.isObject()) {
    if (error) *error = QStringLiteral("The source must be a saved Research Annotation JSON document.");
    return false;
  }
  const QJsonObject source = source_json.object();
  if (source != annotations) {
    if (error) *error = QStringLiteral("Save the reviewed annotations before exporting coverage geometry.");
    return false;
  }
  QJsonObject geometry;
  QJsonObject projected = annotations;
  if (!headland_annotation_ids.isEmpty()) {
    QSet<QString> pending;
    for (const auto &id : headland_annotation_ids) pending.insert(id);
    if (pending.size() != headland_annotation_ids.size()) {
      if (error) *error = QStringLiteral("Duplicate selected headland annotation.");
      return false;
    }
    auto entries = projected.value("annotations").toArray();
    for (int index = 0; index < entries.size(); ++index) {
      auto item = entries[index].toObject();
      const QString id = item.value("annotation_id").toString();
      if (!pending.contains(id)) continue;
      if (item.value("annotation_type").toString() != "transition_region" ||
          !reviewed(item) || polygon(item).isEmpty()) {
        if (error) *error = QStringLiteral("Selected headland %1 must be a reviewed transition polygon.").arg(id);
        return false;
      }
      auto role = item.value("coverage").toObject();
      role.insert("role", "headland");
      item.insert("coverage", role);
      entries[index] = item;
      pending.remove(id);
    }
    if (!pending.isEmpty()) {
      if (error) *error = QStringLiteral("A selected headland annotation is missing.");
      return false;
    }
    projected.insert("annotations", entries);
  }
  if (!transform_copy(&projected, map_from_reference, transform_review_confirmed, error) ||
      !structure_from_annotations(projected, &geometry, error)) return false;
  QJsonObject map;
  if (!read_document(map_yaml, &map, error)) return false;
  const QString image_ref = map.value("image").toString();
  const QString image_path = QFileInfo(image_ref).isAbsolute() ? image_ref : QFileInfo(map_yaml).dir().filePath(image_ref);
  const auto digest = [](const QString &path) {
    QFile input(path);
    if (!input.open(QIODevice::ReadOnly)) return QString();
    QCryptographicHash hash(QCryptographicHash::Sha256);
    if (!hash.addData(&input)) return QString();
    return QStringLiteral("sha256:") + QString::fromLatin1(hash.result().toHex());
  };
  const QString yaml_hash = digest(map_yaml), image_hash = image_ref.isEmpty() ? QString() : digest(image_path);
  if (yaml_hash.isEmpty() || image_hash.isEmpty()) {
    if (error) *error = QStringLiteral("A readable map YAML and referenced image are required to bind the reviewed structure.");
    return false;
  }
  QJsonObject provenance{
      {"annotation_path", QFileInfo(source_file).absoluteFilePath()},
      {"annotation_sha256", QStringLiteral("sha256:") + QString::fromLatin1(
          QCryptographicHash::hash(source_bytes, QCryptographicHash::Sha256).toHex())},
      {"original_frame", annotations.value("reference_frame")},
      {"map_yaml_sha256", yaml_hash}, {"map_image_sha256", image_hash}};
  if (!map_from_reference.isEmpty()) {
    provenance.insert("map_from_reference", map_from_reference);
    provenance.insert("transform_review", "CONFIRMED");
  }
  if (!headland_annotation_ids.isEmpty()) provenance.insert("headland_annotation_ids", QJsonArray::fromStringList(headland_annotation_ids));
  geometry.insert("source", provenance);
  if (QFileInfo::exists(output)) {
    if (error) *error = QStringLiteral("Choose a new structure file; existing exports are not overwritten.");
    return false;
  }
  QTemporaryFile destination(QFileInfo(output).dir().filePath(".coverage-structure-XXXXXX"));
  const auto data = QJsonDocument(geometry).toJson(QJsonDocument::Indented);
  if (!destination.open() || destination.write(data) != data.size() || !destination.flush()) {
    if (error) *error = QStringLiteral("Cannot write coverage structure: %1").arg(output);
    return false;
  }
  destination.close();
  // QFile::rename refuses an existing destination, including a competing
  // writer arriving after the earlier user-facing existence check.
  if (!QFile::rename(destination.fileName(), output)) {
    if (error) *error = QStringLiteral("Cannot publish new coverage structure without replacing a file: %1").arg(output);
    return false;
  }
  return true;
}

}  // namespace agt_map_studio
