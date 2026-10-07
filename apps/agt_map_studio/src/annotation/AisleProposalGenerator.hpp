#pragma once

#include <QPointF>
#include <QString>
#include <QVector>

#include <cstddef>
#include <vector>

namespace agt_map_studio {

struct AisleExtractionParameters {
  double z_min_m = -1000.0;
  double z_max_m = 1000.0;
  double profile_bin_m = 0.10;
  double minimum_row_spacing_m = 0.80;
  double row_half_width_m = 0.22;
  double side_clearance_m = 0.12;
  double minimum_aisle_width_m = 0.45;
  double maximum_aisle_width_m = 2.50;
  double minimum_aisle_length_m = 1.50;
  double minimum_row_support_fraction = 0.05;
  double minimum_peak_fraction = 0.06;
  double minimum_peak_prominence_fraction = 0.02;
  int minimum_points_per_support_bin = 1;
  int maximum_support_gap_bins = 4;
};

struct AisleGeometryProposal {
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

struct AisleProposalGenerationResult {
  QVector<AisleGeometryProposal> proposals;
  std::size_t points_examined = 0;
  std::size_t points_inside_roi = 0;
  int supported_row_count = 0;
  double dominant_axis_deg = 0.0;
  double orientation_score = 0.0;
  double orientation_margin = 0.0;
};

// Produces provisional aisle area candidates only between adjacent supported
// point-density ridges inside the saved greenhouse boundary. It does not infer
// boundary aisles or declare any candidate traversable.
bool generate_aisle_proposals(const std::vector<float> &xyz,
                              const QVector<QPointF> &greenhouse_boundary_xy_m,
                              const AisleExtractionParameters &parameters,
                              AisleProposalGenerationResult *result,
                              QString *error = nullptr);

}  // namespace agt_map_studio
