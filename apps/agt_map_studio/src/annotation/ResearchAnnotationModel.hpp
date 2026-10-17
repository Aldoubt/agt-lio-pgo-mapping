#pragma once

#include "selection/SelectionManager.h"

#include <QColor>
#include <QJsonArray>
#include <QJsonObject>
#include <QPointF>
#include <QString>
#include <QStringList>
#include <QVector>

#include <string>
#include <vector>

namespace agt_map_studio {

struct AnnotationPolygonOverlay {
  QString annotation_id;
  QString annotation_type;
  QString geometry_kind = QStringLiteral("polygon_xy");
  QVector<QPointF> vertices_xy_m;
  QColor color;
  bool selected = false;
};

struct AisleAnnotationProposal {
  QString proposal_id;
  QVector<QPointF> polygon_xy_m;
  QPointF start_xy_m;
  QPointF end_xy_m;
  double start_z_m = 0.0;
  double end_z_m = 0.0;
  double width_m = 0.0;
  double length_m = 0.0;
  double confidence = 0.0;
};

class ResearchAnnotationModel {
public:
  bool load_file(const QString &path, QString *error = nullptr);
  bool load_document(const QJsonObject &document, QString *error = nullptr);
  bool save_file(const QString &path, QString *error = nullptr);

  bool add_polygon(const QString &annotation_type,
                   const QVector<QPointF> &vertices_xy_m,
                   const QStringList &source_session_ids,
                   const QString &source_candidate_id = {},
                   const QString &notes = {}, QString *new_id = nullptr,
                   QString *error = nullptr);
  bool add_polyline(const QVector<QPointF> &vertices_xy_m,
                    const QStringList &source_session_ids,
                    const QString &notes, QString *new_id = nullptr,
                    QString *error = nullptr);
  bool add_point(const QPointF &point_xy_m, double z_m,
                 const QStringList &source_session_ids,
                 const QString &notes, QString *new_id = nullptr,
                 QString *error = nullptr);
  bool add_aisle_proposals(const QVector<AisleAnnotationProposal> &proposals,
                           const QString &source_session_id,
                           const QString &generation_notes,
                           QStringList *new_ids = nullptr,
                           QString *error = nullptr);
  bool add_3d_selection(const QString &annotation_type,
                        const SelectionGeometry &geometry,
                        const QStringList &source_session_ids,
                        const QString &notes = {}, QString *new_id = nullptr,
                        QString *error = nullptr);
  bool move_polygon_vertex(const QString &annotation_id, int vertex,
                           const QPointF &point, QString *error = nullptr);
  bool delete_polygon_vertex(const QString &annotation_id, int vertex,
                             QString *error = nullptr);
  bool replace_polygon(const QString &annotation_id,
                       const QVector<QPointF> &vertices_xy_m,
                       QString *error = nullptr);
  bool replace_xy_geometry(const QString &annotation_id,
                           const QVector<QPointF> &vertices_xy_m,
                           QString *error = nullptr);
  bool delete_annotation(const QString &annotation_id, QString *error = nullptr);
  bool set_annotation_review_status(const QString &annotation_id,
                                    const QString &status,
                                    QString *error = nullptr,
                                    const QString &reviewer = {});
  bool freeze(const QString &reviewer, const QString &reviewed_at,
              QString *error = nullptr);
  bool undo();
  bool redo();

  QJsonObject document() const { return document_; }
  QJsonArray annotations() const { return document_.value("annotations").toArray(); }
  QString dataset_id() const { return document_.value("dataset_id").toString(); }
  QString annotation_version() const { return document_.value("annotation_version").toString(); }
  QString reference_frame() const { return document_.value("reference_frame").toString(); }
  QString review_status() const { return document_.value("review_status").toString("DRAFT"); }
  bool is_frozen() const { return review_status() == QStringLiteral("FROZEN"); }
  bool is_dirty() const { return dirty_; }
  bool is_empty() const { return document_.isEmpty(); }
  QStringList session_ids() const;
  bool has_candidate(const QString &source_candidate_id) const;
  QStringList annotation_ids() const;
  int annotation_index(const QString &annotation_id) const;
  QVector<AnnotationPolygonOverlay> polygon_overlays(
      const QString &selected_annotation_id = {}) const;

private:
  bool can_edit(QString *error) const;
  bool push_undo();
  void set_status_if_all_reviewed();
  static bool valid_status(const QString &status);
  static bool valid_annotation_type(const QString &type);
  static bool valid_polygon(const QVector<QPointF> &points, QString *error);
  static QColor type_color(const QString &type);

  QJsonObject document_;
  std::vector<QJsonObject> undo_stack_;
  std::vector<QJsonObject> redo_stack_;
  bool dirty_ = false;
};

}  // namespace agt_map_studio
