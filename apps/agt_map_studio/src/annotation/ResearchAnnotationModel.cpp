#include "annotation/ResearchAnnotationModel.hpp"

#include <QDateTime>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QLineF>
#include <QSaveFile>
#include <QUuid>

#include <algorithm>
#include <cmath>

namespace agt_map_studio {
namespace {

bool on_segment(const QPointF &a, const QPointF &b, const QPointF &p) {
  const double cross = (b.x() - a.x()) * (p.y() - a.y()) - (b.y() - a.y()) * (p.x() - a.x());
  if (std::abs(cross) > 1e-9) return false;
  return p.x() >= std::min(a.x(), b.x()) - 1e-9 && p.x() <= std::max(a.x(), b.x()) + 1e-9 &&
         p.y() >= std::min(a.y(), b.y()) - 1e-9 && p.y() <= std::max(a.y(), b.y()) + 1e-9;
}

int orientation(const QPointF &a, const QPointF &b, const QPointF &c) {
  const double value = (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
  if (std::abs(value) <= 1e-9) return 0;
  return value > 0.0 ? 1 : 2;
}

bool intersects(const QPointF &a, const QPointF &b, const QPointF &c, const QPointF &d) {
  const int o1 = orientation(a, b, c);
  const int o2 = orientation(a, b, d);
  const int o3 = orientation(c, d, a);
  const int o4 = orientation(c, d, b);
  if (o1 != o2 && o3 != o4) return true;
  return (o1 == 0 && on_segment(a, b, c)) || (o2 == 0 && on_segment(a, b, d)) ||
         (o3 == 0 && on_segment(c, d, a)) || (o4 == 0 && on_segment(c, d, b));
}

QJsonArray point_array(const QPointF &point) {
  return {point.x(), point.y()};
}

bool is_sha256(const QString &value) {
  if (value.size() != 64) return false;
  for (const QChar character : value) {
    const QChar lower = character.toLower();
    if (!((lower >= QLatin1Char('0') && lower <= QLatin1Char('9')) ||
          (lower >= QLatin1Char('a') && lower <= QLatin1Char('f')))) return false;
  }
  return true;
}

bool finite_number_array(const QJsonArray &array, int expected_size) {
  if (array.size() != expected_size) return false;
  for (const auto &value : array) {
    if (!value.isDouble() || !std::isfinite(value.toDouble())) return false;
  }
  return true;
}

}  // namespace

bool ResearchAnnotationModel::load_file(const QString &path, QString *error) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    if (error) *error = QStringLiteral("Cannot read annotation JSON: %1").arg(path);
    return false;
  }
  QJsonParseError parse_error;
  const auto doc = QJsonDocument::fromJson(file.readAll(), &parse_error);
  if (parse_error.error != QJsonParseError::NoError || !doc.isObject()) {
    if (error) *error = QStringLiteral("Invalid annotation JSON: %1").arg(parse_error.errorString());
    return false;
  }
  return load_document(doc.object(), error);
}

bool ResearchAnnotationModel::load_document(const QJsonObject &document, QString *error) {
  if (document.value("schema_id").toString() != QStringLiteral("agt.research_annotations") ||
      document.value("schema_version").toInt(-1) != 1 ||
      document.value("dataset_id").toString().isEmpty() ||
      document.value("annotation_version").toString().isEmpty() ||
      document.value("reference_frame").toString().isEmpty() ||
      !valid_status(document.value("review_status").toString()) ||
      !document.value("source_hashes").isObject() || document.value("source_hashes").toObject().size() < 2 ||
      !document.value("annotations").isArray()) {
    if (error) *error = QStringLiteral("Unsupported or incomplete Research Annotation V1 document");
    return false;
  }
  std::vector<QString> ids;
  for (const auto &value : document.value("annotations").toArray()) {
    if (!value.isObject()) {
      if (error) *error = QStringLiteral("annotation entries must be JSON objects");
      return false;
    }
    const QJsonObject item = value.toObject();
    const QString id = item.value("annotation_id").toString();
    const QString type = item.value("annotation_type").toString();
    if (id.isEmpty() || !valid_annotation_type(type) || !valid_status(item.value("review_status").toString()) ||
        !item.value("geometry").isObject() || !item.value("source_session_ids").isArray() ||
        item.value("source_session_ids").toArray().isEmpty()) {
      if (error) *error = QStringLiteral("annotation entry has missing or unsupported fields");
      return false;
    }
    if (std::find(ids.begin(), ids.end(), id) != ids.end()) {
      if (error) *error = QStringLiteral("duplicate annotation_id: %1").arg(id);
      return false;
    }
    ids.push_back(id);
    const QJsonObject geometry = item.value("geometry").toObject();
    const QJsonArray feature_sessions = item.value("source_session_ids").toArray();
    QStringList seen_sessions;
    for (const auto &session_value : feature_sessions) {
      const QString session_id = session_value.toString();
      if (session_id.isEmpty() || seen_sessions.contains(session_id) ||
          !document.value("source_hashes").toObject().contains(session_id)) {
        if (error) *error = QStringLiteral("annotation has duplicate or unbound source session: %1").arg(session_id);
        return false;
      }
      seen_sessions.push_back(session_id);
    }
    if (geometry.value("kind").toString() == QStringLiteral("polygon_xy")) {
      QVector<QPointF> points;
      for (const auto &vertex : geometry.value("coordinates_xy_m").toArray()) {
        const auto pair = vertex.toArray();
        if (pair.size() != 2 || !pair[0].isDouble() || !pair[1].isDouble()) {
          if (error) *error = QStringLiteral("polygon coordinate must be [x,y]");
          return false;
        }
        points.push_back(QPointF(pair[0].toDouble(), pair[1].toDouble()));
      }
      if (!valid_polygon(points, error)) return false;
    } else if (geometry.value("kind").toString() == QStringLiteral("polyline_xy")) {
      const QJsonArray coordinates = geometry.value("coordinates_xy_m").toArray();
      if (coordinates.size() < 2) {
        if (error) *error = QStringLiteral("polyline requires at least two coordinates");
        return false;
      }
      for (const auto &coordinate : coordinates) {
        if (!finite_number_array(coordinate.toArray(), 2)) {
          if (error) *error = QStringLiteral("polyline coordinate must be finite [x,y]");
          return false;
        }
      }
    } else if (geometry.value("kind").toString() == QStringLiteral("point_xyz")) {
      if (!finite_number_array(geometry.value("xyz_m").toArray(), 3)) {
        if (error) *error = QStringLiteral("point coordinate must be finite [x,y,z]");
        return false;
      }
    } else if (geometry.value("kind").toString() == QStringLiteral("selection_3d")) {
      const QString shape = geometry.value("shape").toString();
      if (shape == "aabb") {
        const QJsonArray minimum = geometry.value("min_xyz_m").toArray();
        const QJsonArray maximum = geometry.value("max_xyz_m").toArray();
        if (!finite_number_array(minimum, 3) || !finite_number_array(maximum, 3)) {
          if (error) *error = QStringLiteral("AABB ROI requires finite min/max XYZ");
          return false;
        }
        for (int i = 0; i < 3; ++i) if (minimum[i].toDouble() > maximum[i].toDouble()) {
          if (error) *error = QStringLiteral("AABB ROI bounds are inverted");
          return false;
        }
      } else if (shape == "polygon_prism") {
        QVector<QPointF> points;
        for (const auto &vertex : geometry.value("coordinates_xy_m").toArray()) {
          const QJsonArray pair = vertex.toArray();
          if (!finite_number_array(pair, 2)) {
            if (error) *error = QStringLiteral("3D polygon ROI coordinate must be finite [x,y]");
            return false;
          }
          points.push_back(QPointF(pair[0].toDouble(), pair[1].toDouble()));
        }
        const QJsonArray z_range = geometry.value("z_range_m").toArray();
        if (!valid_polygon(points, error) || !finite_number_array(z_range, 2) ||
            z_range[0].toDouble() > z_range[1].toDouble()) {
          if (error && error->isEmpty()) *error = QStringLiteral("3D polygon ROI requires a valid polygon and Z range");
          return false;
        }
      } else if (shape == "sphere") {
        if (!finite_number_array(geometry.value("center_xyz_m").toArray(), 3) ||
            !geometry.value("radius_m").isDouble() || geometry.value("radius_m").toDouble() <= 0.0 ||
            !std::isfinite(geometry.value("radius_m").toDouble())) {
          if (error) *error = QStringLiteral("sphere ROI requires a finite center and positive radius");
          return false;
        }
      } else {
        if (error) *error = QStringLiteral("unsupported 3D selection shape");
        return false;
      }
    } else {
      if (error) *error = QStringLiteral("unsupported annotation geometry kind");
      return false;
    }
  }
  for (const auto &value : document.value("source_hashes").toObject()) {
    if (!is_sha256(value.toString())) {
      if (error) *error = QStringLiteral("source_hashes values must be SHA-256 hex digests");
      return false;
    }
  }
  document_ = document;
  undo_stack_.clear();
  redo_stack_.clear();
  dirty_ = false;
  return true;
}

bool ResearchAnnotationModel::save_file(const QString &path, QString *error) {
  if (document_.isEmpty()) {
    if (error) *error = QStringLiteral("No research annotation document is loaded");
    return false;
  }
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly)) {
    if (error) *error = QStringLiteral("Cannot write annotation JSON: %1").arg(path);
    return false;
  }
  const auto payload = QJsonDocument(document_).toJson(QJsonDocument::Indented);
  if (file.write(payload) != payload.size() || !file.commit()) {
    if (error) *error = QStringLiteral("Failed to atomically save annotation JSON: %1").arg(path);
    return false;
  }
  dirty_ = false;
  return true;
}

bool ResearchAnnotationModel::add_polygon(const QString &annotation_type,
                                           const QVector<QPointF> &vertices_xy_m,
                                           const QStringList &source_session_ids,
                                           const QString &source_candidate_id,
                                           const QString &notes,
                                           QString *new_id, QString *error) {
  if (!can_edit(error) || !valid_annotation_type(annotation_type) || source_session_ids.isEmpty()) {
    if (error && error->isEmpty()) *error = QStringLiteral("invalid annotation type or missing source session");
    return false;
  }
  if (!valid_polygon(vertices_xy_m, error)) return false;
  for (const auto &session_id : source_session_ids) {
    const QJsonObject hashes = document_.value("source_hashes").toObject();
    if (!hashes.contains(session_id) || hashes.value(session_id).toString().size() != 64) {
      if (error) *error = QStringLiteral("annotation source session/hash is not bound: %1").arg(session_id);
      return false;
    }
  }
  push_undo();
  QJsonArray coordinates;
  for (const auto &point : vertices_xy_m) coordinates.push_back(point_array(point));
  QJsonObject item{
      {"annotation_id", QStringLiteral("annotation-%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces))},
      {"annotation_type", annotation_type},
      {"geometry", QJsonObject{{"kind", "polygon_xy"}, {"coordinates_xy_m", coordinates}}},
      {"review_status", "DRAFT"},
      {"source_session_ids", QJsonArray::fromStringList(source_session_ids)},
      {"notes", notes},
  };
  if (!source_candidate_id.isEmpty()) item.insert("source_candidate_id", source_candidate_id);
  QJsonArray entries = annotations();
  entries.push_back(item);
  document_.insert("annotations", entries);
  set_status_if_all_reviewed();
  dirty_ = true;
  if (new_id) *new_id = item.value("annotation_id").toString();
  return true;
}

bool ResearchAnnotationModel::add_polyline(const QVector<QPointF> &vertices_xy_m,
                                           const QStringList &source_session_ids,
                                           const QString &notes,
                                           QString *new_id, QString *error) {
  if (!can_edit(error) || source_session_ids.isEmpty() || vertices_xy_m.size() < 2) {
    if (error && error->isEmpty()) *error = QStringLiteral("polyline requires a writable project, two points and source sessions");
    return false;
  }
  for (int i = 0; i < vertices_xy_m.size(); ++i) {
    const auto &point = vertices_xy_m[i];
    if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
        (i > 0 && QLineF(vertices_xy_m[i - 1], point).length() <= 1e-9)) {
      if (error) *error = QStringLiteral("polyline coordinates must be finite and adjacent vertices distinct");
      return false;
    }
  }
  const QJsonObject hashes = document_.value("source_hashes").toObject();
  for (const auto &session_id : source_session_ids) {
    if (!hashes.contains(session_id) || hashes.value(session_id).toString().size() != 64) {
      if (error) *error = QStringLiteral("annotation source session/hash is not bound: %1").arg(session_id);
      return false;
    }
  }
  QJsonArray coordinates;
  for (const auto &point : vertices_xy_m) coordinates.push_back(point_array(point));
  QJsonObject item{
      {"annotation_id", QStringLiteral("annotation-%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces))},
      {"annotation_type", "custom"},
      {"geometry", QJsonObject{{"kind", "polyline_xy"}, {"coordinates_xy_m", coordinates}}},
      {"review_status", "DRAFT"},
      {"source_session_ids", QJsonArray::fromStringList(source_session_ids)},
      {"notes", notes},
  };
  push_undo();
  QJsonArray entries = annotations();
  entries.push_back(item);
  document_.insert("annotations", entries);
  set_status_if_all_reviewed();
  dirty_ = true;
  if (new_id) *new_id = item.value("annotation_id").toString();
  return true;
}

bool ResearchAnnotationModel::add_point(const QPointF &point_xy_m, double z_m,
                                         const QStringList &source_session_ids,
                                         const QString &notes,
                                         QString *new_id, QString *error) {
  if (!can_edit(error) || source_session_ids.isEmpty() || !std::isfinite(point_xy_m.x()) ||
      !std::isfinite(point_xy_m.y()) || !std::isfinite(z_m)) {
    if (error && error->isEmpty()) *error = QStringLiteral("point requires a writable project, finite XYZ and source sessions");
    return false;
  }
  const QJsonObject hashes = document_.value("source_hashes").toObject();
  for (const auto &session_id : source_session_ids) {
    if (!hashes.contains(session_id) || hashes.value(session_id).toString().size() != 64) {
      if (error) *error = QStringLiteral("annotation source session/hash is not bound: %1").arg(session_id);
      return false;
    }
  }
  QJsonObject item{
      {"annotation_id", QStringLiteral("annotation-%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces))},
      {"annotation_type", "custom"},
      {"geometry", QJsonObject{{"kind", "point_xyz"}, {"xyz_m", QJsonArray{point_xy_m.x(), point_xy_m.y(), z_m}}}},
      {"review_status", "DRAFT"},
      {"source_session_ids", QJsonArray::fromStringList(source_session_ids)},
      {"notes", notes},
  };
  push_undo();
  QJsonArray entries = annotations();
  entries.push_back(item);
  document_.insert("annotations", entries);
  set_status_if_all_reviewed();
  dirty_ = true;
  if (new_id) *new_id = item.value("annotation_id").toString();
  return true;
}

bool ResearchAnnotationModel::add_aisle_proposals(
    const QVector<AisleAnnotationProposal> &proposals,
    const QString &source_session_id,
    const QString &generation_notes,
    QStringList *new_ids, QString *error) {
  if (!can_edit(error) || proposals.isEmpty()) {
    if (error && error->isEmpty()) *error = QStringLiteral("at least one aisle proposal is required");
    return false;
  }
  const QJsonObject hashes = document_.value("source_hashes").toObject();
  if (!hashes.contains(source_session_id) || !is_sha256(hashes.value(source_session_id).toString())) {
    if (error) *error = QStringLiteral("aisle proposals require a bound source session/hash: %1").arg(source_session_id);
    return false;
  }
  QStringList ids;
  for (const auto &proposal : proposals) {
    if (proposal.proposal_id.trimmed().isEmpty() || !valid_polygon(proposal.polygon_xy_m, error) ||
        !std::isfinite(proposal.start_xy_m.x()) || !std::isfinite(proposal.start_xy_m.y()) ||
        !std::isfinite(proposal.end_xy_m.x()) || !std::isfinite(proposal.end_xy_m.y()) ||
        !std::isfinite(proposal.start_z_m) || !std::isfinite(proposal.end_z_m) ||
        !std::isfinite(proposal.width_m) || proposal.width_m <= 0.0 ||
        !std::isfinite(proposal.length_m) || proposal.length_m <= 0.0 ||
        !std::isfinite(proposal.confidence) || proposal.confidence < 0.0 || proposal.confidence > 1.0) {
      if (error && error->isEmpty()) *error = QStringLiteral("aisle proposal has invalid identity, geometry, or diagnostics");
      return false;
    }
    if (ids.contains(proposal.proposal_id) || has_candidate(proposal.proposal_id)) {
      if (error) *error = QStringLiteral("duplicate aisle proposal ID: %1").arg(proposal.proposal_id);
      return false;
    }
    ids.push_back(proposal.proposal_id);
  }

  push_undo();
  QJsonArray entries = annotations();
  QStringList created_ids;
  for (const auto &proposal : proposals) {
    const QString aisle_id = QStringLiteral("annotation-%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces));
    const QString start_id = QStringLiteral("annotation-%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces));
    const QString end_id = QStringLiteral("annotation-%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces));
    QJsonArray polygon_points;
    for (const auto &point : proposal.polygon_xy_m) polygon_points.push_back(point_array(point));
    const QString common = generation_notes +
        QStringLiteral("\nproposal_id: %1\nwidth_m: %2\nlength_m: %3\nconfidence: %4")
            .arg(proposal.proposal_id)
            .arg(proposal.width_m, 0, 'f', 3)
            .arg(proposal.length_m, 0, 'f', 3)
            .arg(proposal.confidence, 0, 'f', 3);
    QJsonObject aisle{
        {"annotation_id", aisle_id},
        {"annotation_type", "custom"},
        {"geometry", QJsonObject{{"kind", "polygon_xy"}, {"coordinates_xy_m", polygon_points}}},
        {"review_status", "DRAFT"},
        {"candidate_status", "PROVISIONAL"},
        {"source_candidate_id", proposal.proposal_id},
        {"source_session_ids", QJsonArray{source_session_id}},
        {"notes", QStringLiteral("MapStudio UI type: Aisle\n%1").arg(common)},
    };
    const auto endpoint = [&](const QString &id, const QString &suffix,
                              const QPointF &xy, double z, const QString &side) {
      return QJsonObject{
          {"annotation_id", id},
          {"annotation_type", "custom"},
          {"geometry", QJsonObject{{"kind", "point_xyz"}, {"xyz_m", QJsonArray{xy.x(), xy.y(), z}}}},
          {"review_status", "DRAFT"},
          {"candidate_status", "PROVISIONAL"},
          {"source_candidate_id", proposal.proposal_id + suffix},
          {"source_session_ids", QJsonArray{source_session_id}},
          {"notes", QStringLiteral("MapStudio UI type: Aisle End\nproposal_id: %1\nend: %2\n%3")
              .arg(proposal.proposal_id, side, generation_notes)},
      };
    };
    entries.push_back(aisle);
    entries.push_back(endpoint(start_id, QStringLiteral("-START"), proposal.start_xy_m,
                               proposal.start_z_m, QStringLiteral("START")));
    entries.push_back(endpoint(end_id, QStringLiteral("-END"), proposal.end_xy_m,
                               proposal.end_z_m, QStringLiteral("END")));
    created_ids << aisle_id << start_id << end_id;
  }
  document_.insert("annotations", entries);
  set_status_if_all_reviewed();
  dirty_ = true;
  if (new_ids) *new_ids = created_ids;
  return true;
}

bool ResearchAnnotationModel::add_3d_selection(const QString &annotation_type,
                                                const SelectionGeometry &geometry,
                                                const QStringList &source_session_ids,
                                                const QString &notes,
                                                QString *new_id, QString *error) {
  if (!can_edit(error) || source_session_ids.isEmpty() ||
      !(annotation_type == "fixed_frame_region" || annotation_type == "fixed_column_region" ||
        annotation_type == "stable_structure_roi" || annotation_type == "corner" || annotation_type == "frame_edge")) {
    if (error && error->isEmpty()) *error = QStringLiteral("invalid stable-structure type or missing source session");
    return false;
  }
  for (const auto &session_id : source_session_ids) {
    if (!document_.value("source_hashes").toObject().contains(session_id)) {
      if (error) *error = QStringLiteral("unknown annotation source session: %1").arg(session_id);
      return false;
    }
  }
  QJsonObject geometry_json{{"kind", "selection_3d"}};
  if (geometry.rule_type == "remove_polygon") {
    if (geometry.polygon_xy.size() < 3 || !geometry.has_z_range ||
        !std::isfinite(geometry.z_min) || !std::isfinite(geometry.z_max) ||
        geometry.z_min > geometry.z_max) {
      if (error) *error = QStringLiteral("3D polygon selection requires at least three vertices and a Z window");
      return false;
    }
    QVector<QPointF> polygon;
    for (const auto &point : geometry.polygon_xy) {
      if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
        if (error) *error = QStringLiteral("3D polygon selection contains a non-finite vertex");
        return false;
      }
      polygon.push_back(QPointF(point.x(), point.y()));
    }
    if (!valid_polygon(polygon, error)) return false;
    QJsonArray points;
    for (const auto &point : geometry.polygon_xy) points.push_back(QJsonArray{point.x(), point.y()});
    geometry_json.insert("shape", "polygon_prism");
    geometry_json.insert("coordinates_xy_m", points);
    geometry_json.insert("z_range_m", QJsonArray{geometry.z_min, geometry.z_max});
  } else if (geometry.rule_type == "remove_sphere") {
    if (!(geometry.radius > 0.0) || !std::isfinite(geometry.radius) ||
        !geometry.center.allFinite()) {
      if (error) *error = QStringLiteral("sphere selection radius must be positive and finite");
      return false;
    }
    geometry_json.insert("shape", "sphere");
    geometry_json.insert("center_xyz_m", QJsonArray{geometry.center.x(), geometry.center.y(), geometry.center.z()});
    geometry_json.insert("radius_m", geometry.radius);
  } else if (geometry.rule_type == "remove_box") {
    if (!geometry.box.valid || !geometry.box.min.allFinite() || !geometry.box.max.allFinite() ||
        (geometry.box.min.array() > geometry.box.max.array()).any()) {
      if (error) *error = QStringLiteral("3D box selection is invalid");
      return false;
    }
    geometry_json.insert("shape", "aabb");
    geometry_json.insert("min_xyz_m", QJsonArray{geometry.box.min.x(), geometry.box.min.y(), geometry.box.min.z()});
    geometry_json.insert("max_xyz_m", QJsonArray{geometry.box.max.x(), geometry.box.max.y(), geometry.box.max.z()});
  } else {
    if (error) *error = QStringLiteral("unsupported existing 3D selection geometry type");
    return false;
  }
  push_undo();
  QJsonObject item{
      {"annotation_id", QStringLiteral("annotation-%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces))},
      {"annotation_type", annotation_type}, {"geometry", geometry_json},
      {"review_status", "DRAFT"}, {"source_session_ids", QJsonArray::fromStringList(source_session_ids)},
      {"notes", notes},
  };
  QJsonArray entries = annotations();
  entries.push_back(item);
  document_.insert("annotations", entries);
  set_status_if_all_reviewed();
  dirty_ = true;
  if (new_id) *new_id = item.value("annotation_id").toString();
  return true;
}

bool ResearchAnnotationModel::move_polygon_vertex(const QString &annotation_id, int vertex,
                                                  const QPointF &point, QString *error) {
  const int index = annotation_index(annotation_id);
  if (!can_edit(error) || index < 0 || !std::isfinite(point.x()) || !std::isfinite(point.y())) {
    if (error && error->isEmpty()) *error = QStringLiteral("invalid annotation or coordinate");
    return false;
  }
  QJsonArray entries = annotations();
  QJsonObject item = entries[index].toObject();
  QJsonObject geometry = item.value("geometry").toObject();
  if (geometry.value("kind").toString() != "polygon_xy") {
    if (error) *error = QStringLiteral("selected annotation is not a polygon");
    return false;
  }
  QJsonArray points = geometry.value("coordinates_xy_m").toArray();
  if (vertex < 0 || vertex >= points.size()) {
    if (error) *error = QStringLiteral("polygon vertex index is out of range");
    return false;
  }
  QVector<QPointF> proposed;
  for (const auto &value : points) {
    const auto pair = value.toArray();
    proposed.push_back(QPointF(pair[0].toDouble(), pair[1].toDouble()));
  }
  proposed[vertex] = point;
  if (!valid_polygon(proposed, error)) return false;
  push_undo();
  points[vertex] = point_array(point);
  geometry.insert("coordinates_xy_m", points);
  item.insert("geometry", geometry);
  item.insert("review_status", "DRAFT");
  item.remove("reviewer");
  item.remove("reviewed_at");
  entries[index] = item;
  document_.insert("annotations", entries);
  set_status_if_all_reviewed();
  dirty_ = true;
  return true;
}

bool ResearchAnnotationModel::replace_polygon(const QString &annotation_id,
                                               const QVector<QPointF> &vertices_xy_m,
                                               QString *error) {
  const int index = annotation_index(annotation_id);
  if (!can_edit(error) || index < 0 || !valid_polygon(vertices_xy_m, error)) return false;
  QJsonArray entries = annotations();
  QJsonObject item = entries[index].toObject();
  QJsonObject geometry = item.value("geometry").toObject();
  if (geometry.value("kind").toString() != "polygon_xy") {
    if (error) *error = QStringLiteral("selected annotation is not a polygon");
    return false;
  }
  push_undo();
  QJsonArray points;
  for (const auto &point : vertices_xy_m) points.push_back(point_array(point));
  geometry.insert("coordinates_xy_m", points);
  item.insert("geometry", geometry);
  item.insert("review_status", "DRAFT");
  item.remove("reviewer");
  item.remove("reviewed_at");
  entries[index] = item;
  document_.insert("annotations", entries);
  set_status_if_all_reviewed();
  dirty_ = true;
  return true;
}

bool ResearchAnnotationModel::replace_xy_geometry(const QString &annotation_id,
                                                   const QVector<QPointF> &vertices_xy_m,
                                                   QString *error) {
  const int index = annotation_index(annotation_id);
  if (!can_edit(error) || index < 0) {
    if (error && error->isEmpty()) *error = QStringLiteral("invalid annotation");
    return false;
  }
  QJsonArray entries = annotations();
  QJsonObject item = entries[index].toObject();
  QJsonObject geometry = item.value("geometry").toObject();
  const QString kind = geometry.value("kind").toString();
  if (kind == "polygon_xy") {
    if (!valid_polygon(vertices_xy_m, error)) return false;
    QJsonArray coordinates;
    for (const auto &point : vertices_xy_m) coordinates.push_back(point_array(point));
    geometry.insert("coordinates_xy_m", coordinates);
  } else if (kind == "polyline_xy") {
    if (vertices_xy_m.size() < 2) {
      if (error) *error = QStringLiteral("polyline must retain at least two vertices");
      return false;
    }
    QJsonArray coordinates;
    for (int i = 0; i < vertices_xy_m.size(); ++i) {
      const auto &point = vertices_xy_m[i];
      if (!std::isfinite(point.x()) || !std::isfinite(point.y()) ||
          (i > 0 && QLineF(vertices_xy_m[i - 1], point).length() <= 1e-9)) {
        if (error) *error = QStringLiteral("polyline coordinates must be finite and adjacent vertices distinct");
        return false;
      }
      coordinates.push_back(point_array(point));
    }
    geometry.insert("coordinates_xy_m", coordinates);
  } else if (kind == "point_xyz") {
    if (vertices_xy_m.size() != 1 || !std::isfinite(vertices_xy_m[0].x()) ||
        !std::isfinite(vertices_xy_m[0].y())) {
      if (error) *error = QStringLiteral("point annotation has exactly one finite XY coordinate");
      return false;
    }
    QJsonArray xyz = geometry.value("xyz_m").toArray();
    if (xyz.size() != 3 || !std::isfinite(xyz[2].toDouble())) {
      if (error) *error = QStringLiteral("point annotation has invalid Z coordinate");
      return false;
    }
    xyz[0] = vertices_xy_m[0].x();
    xyz[1] = vertices_xy_m[0].y();
    geometry.insert("xyz_m", xyz);
  } else {
    if (error) *error = QStringLiteral("selected annotation has no editable XY geometry");
    return false;
  }
  push_undo();
  item.insert("geometry", geometry);
  item.insert("review_status", "DRAFT");
  item.remove("reviewer");
  item.remove("reviewed_at");
  entries[index] = item;
  document_.insert("annotations", entries);
  set_status_if_all_reviewed();
  dirty_ = true;
  return true;
}

bool ResearchAnnotationModel::delete_polygon_vertex(const QString &annotation_id, int vertex,
                                                     QString *error) {
  const int index = annotation_index(annotation_id);
  if (!can_edit(error) || index < 0) {
    if (error && error->isEmpty()) *error = QStringLiteral("invalid annotation");
    return false;
  }
  QJsonArray entries = annotations();
  QJsonObject item = entries[index].toObject();
  QJsonObject geometry = item.value("geometry").toObject();
  const QString kind = geometry.value("kind").toString();
  if (kind != "polygon_xy" && kind != "polyline_xy") {
    if (error) *error = QStringLiteral("selected annotation has no removable line vertex");
    return false;
  }
  QJsonArray points = geometry.value("coordinates_xy_m").toArray();
  const int minimum = kind == "polygon_xy" ? 3 : 2;
  if (vertex < 0 || vertex >= points.size() || points.size() <= minimum) {
    if (error) *error = kind == "polygon_xy"
        ? QStringLiteral("cannot delete this vertex; polygon must retain at least three vertices")
        : QStringLiteral("cannot delete this vertex; polyline must retain at least two vertices");
    return false;
  }
  QVector<QPointF> proposed;
  for (const auto &value : points) {
    const auto pair = value.toArray();
    proposed.push_back(QPointF(pair[0].toDouble(), pair[1].toDouble()));
  }
  proposed.removeAt(vertex);
  if (kind == "polygon_xy" && !valid_polygon(proposed, error)) return false;
  if (kind == "polyline_xy") {
    for (int i = 0; i < proposed.size(); ++i) {
      if (!std::isfinite(proposed[i].x()) || !std::isfinite(proposed[i].y()) ||
          (i > 0 && QLineF(proposed[i - 1], proposed[i]).length() <= 1e-9)) {
        if (error) *error = QStringLiteral("polyline coordinates must remain finite and distinct");
        return false;
      }
    }
  }
  push_undo();
  points.removeAt(vertex);
  geometry.insert("coordinates_xy_m", points);
  item.insert("geometry", geometry);
  item.insert("review_status", "DRAFT");
  item.remove("reviewer");
  item.remove("reviewed_at");
  entries[index] = item;
  document_.insert("annotations", entries);
  set_status_if_all_reviewed();
  dirty_ = true;
  return true;
}

bool ResearchAnnotationModel::delete_annotation(const QString &annotation_id, QString *error) {
  if (!can_edit(error)) return false;
  QJsonArray entries = annotations();
  const int index = annotation_index(annotation_id);
  if (index < 0) {
    if (error) *error = QStringLiteral("annotation does not exist: %1").arg(annotation_id);
    return false;
  }
  push_undo();
  entries.removeAt(index);
  document_.insert("annotations", entries);
  set_status_if_all_reviewed();
  dirty_ = true;
  return true;
}

bool ResearchAnnotationModel::set_annotation_review_status(const QString &annotation_id,
                                                            const QString &status,
                                                            QString *error,
                                                            const QString &reviewer) {
  if (!can_edit(error) || !valid_status(status) || status == "FROZEN") {
    if (error && error->isEmpty()) *error = QStringLiteral("feature status must be DRAFT or REVIEWED before freezing the document");
    return false;
  }
  QJsonArray entries = annotations();
  const int index = annotation_index(annotation_id);
  if (index < 0) {
    if (error) *error = QStringLiteral("annotation does not exist: %1").arg(annotation_id);
    return false;
  }
  push_undo();
  QJsonObject item = entries[index].toObject();
  item.insert("review_status", status);
  if (status == "REVIEWED" && !reviewer.trimmed().isEmpty()) {
    item.insert("reviewer", reviewer.trimmed());
    item.insert("reviewed_at", QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
  } else if (status == "DRAFT") {
    item.remove("reviewer");
    item.remove("reviewed_at");
  }
  entries[index] = item;
  document_.insert("annotations", entries);
  if (status == "REVIEWED" && !reviewer.trimmed().isEmpty()) {
    document_.insert("reviewer", reviewer.trimmed());
    document_.insert("reviewed_at", QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
  }
  set_status_if_all_reviewed();
  dirty_ = true;
  return true;
}

bool ResearchAnnotationModel::freeze(const QString &reviewer, const QString &reviewed_at,
                                      QString *error) {
  if (document_.isEmpty() || is_frozen() || reviewer.trimmed().isEmpty()) {
    if (error) *error = QStringLiteral("document must be loaded, editable and have a reviewer name");
    return false;
  }
  QJsonArray entries = annotations();
  if (entries.isEmpty() || std::any_of(entries.begin(), entries.end(), [](const QJsonValue &value) {
        return value.toObject().value("review_status").toString() != "REVIEWED";
      })) {
    if (error) *error = QStringLiteral("every annotation must be REVIEWED before freezing");
    return false;
  }
  for (int i = 0; i < entries.size(); ++i) {
    QJsonObject item = entries[i].toObject();
    item.insert("review_status", "FROZEN");
    entries[i] = item;
  }
  document_.insert("annotations", entries);
  document_.insert("review_status", "FROZEN");
  document_.insert("reviewer", reviewer.trimmed());
  document_.insert("reviewed_at", reviewed_at.isEmpty() ? QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs) : reviewed_at);
  undo_stack_.clear();
  redo_stack_.clear();
  dirty_ = true;
  return true;
}

bool ResearchAnnotationModel::undo() {
  if (undo_stack_.empty() || is_frozen()) return false;
  redo_stack_.push_back(document_);
  document_ = undo_stack_.back();
  undo_stack_.pop_back();
  dirty_ = true;
  return true;
}

bool ResearchAnnotationModel::redo() {
  if (redo_stack_.empty() || is_frozen()) return false;
  undo_stack_.push_back(document_);
  document_ = redo_stack_.back();
  redo_stack_.pop_back();
  dirty_ = true;
  return true;
}

QStringList ResearchAnnotationModel::session_ids() const {
  return document_.value("source_hashes").toObject().keys();
}

bool ResearchAnnotationModel::has_candidate(const QString &source_candidate_id) const {
  for (const auto &value : annotations()) {
    if (value.toObject().value("source_candidate_id").toString() == source_candidate_id) return true;
  }
  return false;
}

QStringList ResearchAnnotationModel::annotation_ids() const {
  QStringList result;
  for (const auto &value : annotations()) result.push_back(value.toObject().value("annotation_id").toString());
  return result;
}

int ResearchAnnotationModel::annotation_index(const QString &annotation_id) const {
  const QJsonArray entries = annotations();
  for (int i = 0; i < entries.size(); ++i) {
    if (entries[i].toObject().value("annotation_id").toString() == annotation_id) return i;
  }
  return -1;
}

QVector<AnnotationPolygonOverlay> ResearchAnnotationModel::polygon_overlays(
    const QString &selected_annotation_id) const {
  QVector<AnnotationPolygonOverlay> result;
  for (const auto &value : annotations()) {
    const QJsonObject item = value.toObject();
    const QJsonObject geometry = item.value("geometry").toObject();
    const QString kind = geometry.value("kind").toString();
    if (kind != "polygon_xy" && kind != "polyline_xy" && kind != "point_xyz") continue;
    AnnotationPolygonOverlay overlay;
    overlay.annotation_id = item.value("annotation_id").toString();
    overlay.geometry_kind = kind;
    overlay.annotation_type = item.value("annotation_type").toString();
    const QString notes = item.value("notes").toString();
    const QString marker = QStringLiteral("MapStudio UI type: ");
    const int marker_index = notes.indexOf(marker);
    if (marker_index >= 0) {
      const QString display_type = notes.mid(marker_index + marker.size()).section('\n', 0, 0).trimmed();
      if (!display_type.isEmpty()) overlay.annotation_type = display_type;
    }
    overlay.color = type_color(overlay.annotation_type);
    overlay.selected = overlay.annotation_id == selected_annotation_id;
    if (kind == "point_xyz") {
      const QJsonArray point = geometry.value("xyz_m").toArray();
      if (point.size() == 3) overlay.vertices_xy_m.push_back(QPointF(point[0].toDouble(), point[1].toDouble()));
    } else {
      for (const auto &vertex : geometry.value("coordinates_xy_m").toArray()) {
        const auto point = vertex.toArray();
        if (point.size() == 2) overlay.vertices_xy_m.push_back(QPointF(point[0].toDouble(), point[1].toDouble()));
      }
    }
    result.push_back(overlay);
  }
  return result;
}

bool ResearchAnnotationModel::can_edit(QString *error) const {
  if (document_.isEmpty()) {
    if (error) *error = QStringLiteral("Open a Research Annotation project first");
    return false;
  }
  if (is_frozen()) {
    if (error) *error = QStringLiteral("FROZEN annotation is immutable; create a new draft version to revise it");
    return false;
  }
  return true;
}

bool ResearchAnnotationModel::push_undo() {
  undo_stack_.push_back(document_);
  if (undo_stack_.size() > 100) undo_stack_.erase(undo_stack_.begin());
  redo_stack_.clear();
  return true;
}

void ResearchAnnotationModel::set_status_if_all_reviewed() {
  const QJsonArray entries = annotations();
  const bool all_reviewed = !entries.isEmpty() && std::all_of(entries.begin(), entries.end(), [](const QJsonValue &value) {
    return value.toObject().value("review_status").toString() == "REVIEWED";
  });
  document_.insert("review_status", all_reviewed ? "REVIEWED" : "DRAFT");
  if (!all_reviewed) {
    document_.insert("reviewer", QJsonValue::Null);
    document_.insert("reviewed_at", QJsonValue::Null);
  }
}

bool ResearchAnnotationModel::valid_status(const QString &status) {
  return status == "DRAFT" || status == "REVIEWED" || status == "FROZEN";
}

bool ResearchAnnotationModel::valid_annotation_type(const QString &type) {
  static const QStringList types = {
      "greenhouse_boundary", "navigation_interior", "harvested_region", "transition_region",
      "dense_vegetation_region", "external_background", "fixed_frame_region",
      "fixed_column_region", "stable_structure_roi", "corner", "frame_edge", "custom"};
  return types.contains(type);
}

bool ResearchAnnotationModel::valid_polygon(const QVector<QPointF> &points, QString *error) {
  if (points.size() < 3) {
    if (error) *error = QStringLiteral("polygon needs at least three vertices");
    return false;
  }
  double twice_area = 0.0;
  for (int i = 0; i < points.size(); ++i) {
    const auto &a = points[i];
    const auto &b = points[(i + 1) % points.size()];
    if (!std::isfinite(a.x()) || !std::isfinite(a.y()) || QLineF(a, b).length() <= 1e-9) {
      if (error) *error = QStringLiteral("polygon has non-finite or duplicate adjacent vertices");
      return false;
    }
    twice_area += a.x() * b.y() - b.x() * a.y();
  }
  for (int i = 0; i < points.size(); ++i) {
    const int i2 = (i + 1) % points.size();
    for (int j = i + 1; j < points.size(); ++j) {
      const int j2 = (j + 1) % points.size();
      if (i == j || i2 == j || j2 == i) continue;
      if (intersects(points[i], points[i2], points[j], points[j2])) {
        if (error) *error = QStringLiteral("polygon edges self-intersect");
        return false;
      }
    }
  }
  if (std::abs(twice_area) <= 1e-8) {
    if (error) *error = QStringLiteral("polygon area must be positive");
    return false;
  }
  return true;
}

QColor ResearchAnnotationModel::type_color(const QString &type) {
  if (type == "greenhouse_boundary") return QColor(125, 44, 191);
  if (type == "navigation_interior") return QColor(24, 124, 112);
  if (type == "Aisle") return QColor(42, 157, 143);
  if (type == "Aisle End") return QColor(245, 165, 36);
  if (type == "Obstacle Region") return QColor(220, 53, 69);
  if (type == "Traversable Region") return QColor(46, 139, 87);
  if (type == "harvested_region") return QColor(231, 111, 81);
  if (type == "transition_region") return QColor(233, 196, 79);
  if (type == "dense_vegetation_region") return QColor(69, 123, 157);
  if (type == "external_background") return QColor(105, 110, 115);
  return QColor(240, 140, 30);
}

}  // namespace agt_map_studio
