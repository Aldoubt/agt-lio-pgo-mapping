#include "annotation/AisleProposalGenerator.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using agt_map_studio::AisleExtractionParameters;
using agt_map_studio::AisleProposalGenerationResult;
using agt_map_studio::generate_aisle_proposals;

namespace {

const QVector<QPointF> kBoundary{{0.0, 0.0}, {20.0, 0.0}, {20.0, 8.0}, {0.0, 8.0}};
const QVector<QPointF> kSquareBoundary{{0.0, 0.0}, {8.0, 0.0}, {8.0, 8.0}, {0.0, 8.0}};

std::vector<float> greenhouse_rows(const std::vector<double> &row_centers) {
  std::vector<float> xyz;
  const double lateral_offsets[] = {-0.12, -0.06, 0.0, 0.06, 0.12};
  const double heights[] = {0.3, 0.8, 1.2};
  for (double x = 1.0; x <= 19.001; x += 0.1) {
    for (const double center : row_centers) {
      for (const double offset : lateral_offsets) {
        for (const double z : heights) {
          xyz.push_back(static_cast<float>(x));
          xyz.push_back(static_cast<float>(center + offset));
          xyz.push_back(static_cast<float>(z));
        }
      }
    }
  }
  return xyz;
}

std::vector<float> perpendicular_greenhouse_rows(const std::vector<double> &row_centers) {
  std::vector<float> xyz;
  const double lateral_offsets[] = {-0.12, -0.06, 0.0, 0.06, 0.12};
  const double heights[] = {0.3, 0.8, 1.2};
  for (double y = 0.5; y <= 7.501; y += 0.1) {
    for (const double center : row_centers) {
      for (const double offset : lateral_offsets) {
        for (const double z : heights) {
          xyz.push_back(static_cast<float>(center + offset));
          xyz.push_back(static_cast<float>(y));
          xyz.push_back(static_cast<float>(z));
        }
      }
    }
  }
  return xyz;
}

}  // namespace

TEST(AisleProposalGeneratorTest, FindsCorridorsAndMarksBothEndsBetweenSupportedRows) {
  auto xyz = greenhouse_rows({1.2, 3.0, 4.8, 6.6});
  // Outside-ROI clutter and points outside the requested height band must not affect the result.
  xyz.insert(xyz.end(), {22.0F, 4.0F, 0.8F, 10.0F, 4.0F, 20.0F});
  AisleExtractionParameters parameters;
  parameters.z_min_m = 0.0;
  parameters.z_max_m = 2.0;
  AisleProposalGenerationResult result;
  QString error;

  ASSERT_TRUE(generate_aisle_proposals(xyz, kBoundary, parameters, &result, &error)) << error.toStdString();
  ASSERT_EQ(result.supported_row_count, 4);
  ASSERT_EQ(result.proposals.size(), 3);
  EXPECT_EQ(result.points_examined, xyz.size() / 3);
  EXPECT_GT(result.points_inside_roi, 100U);
  EXPECT_NEAR(result.dominant_axis_deg, 0.0, 1.0);
  EXPECT_GT(result.orientation_score, 0.20);
  EXPECT_GE(result.orientation_margin, 0.12);
  for (int i = 0; i < result.proposals.size(); ++i) {
    const auto &proposal = result.proposals[i];
    EXPECT_EQ(proposal.proposal_id, QStringLiteral("AUTO-A%1").arg(i + 1, 3, 10, QLatin1Char('0')));
    ASSERT_EQ(proposal.polygon_xy_m.size(), 4);
    EXPECT_NEAR(proposal.width_m, 1.12, 0.04);
    EXPECT_GT(proposal.length_m, 15.0);
    EXPECT_NEAR(proposal.start_xy_m.y(), proposal.end_xy_m.y(), 0.10);
    EXPECT_GT(proposal.end_xy_m.x(), proposal.start_xy_m.x());
    EXPECT_TRUE(std::isfinite(proposal.start_z_m));
    EXPECT_TRUE(std::isfinite(proposal.end_z_m));
    EXPECT_GE(proposal.confidence, 0.0);
    EXPECT_LE(proposal.confidence, 1.0);
  }
}

TEST(AisleProposalGeneratorTest, RejectsDirectionEstimateFromOnlyOneRow) {
  const auto xyz = greenhouse_rows({3.0});
  AisleExtractionParameters parameters;
  parameters.z_min_m = 0.0;
  parameters.z_max_m = 2.0;
  AisleProposalGenerationResult result;
  QString error;

  EXPECT_FALSE(generate_aisle_proposals(xyz, kBoundary, parameters, &result, &error));
  EXPECT_NE(error.indexOf(QStringLiteral("weak or ambiguous")), -1);
}

TEST(AisleProposalGeneratorTest, InfersRowsPerpendicularToGreenhouseBoundaryMajorAxis) {
  auto xyz = perpendicular_greenhouse_rows({1.2, 3.0, 4.8, 6.6});
  AisleExtractionParameters parameters;
  parameters.z_min_m = 0.0;
  parameters.z_max_m = 2.0;
  AisleProposalGenerationResult result;
  QString error;

  ASSERT_TRUE(generate_aisle_proposals(xyz, kBoundary, parameters, &result, &error)) << error.toStdString();
  EXPECT_NEAR(result.dominant_axis_deg, 90.0, 1.0);
  ASSERT_EQ(result.proposals.size(), 3);
  for (const auto &proposal : result.proposals) {
    EXPECT_NEAR(proposal.start_xy_m.x(), proposal.end_xy_m.x(), 0.15);
    EXPECT_GT(std::abs(proposal.end_xy_m.y() - proposal.start_xy_m.y()), 5.0);
  }
}

TEST(AisleProposalGeneratorTest, RejectsCrossHatchWhenRowDirectionIsAmbiguous) {
  std::vector<float> xyz = greenhouse_rows({1.2, 3.0, 4.8, 6.6});
  const auto vertical = perpendicular_greenhouse_rows({1.2, 3.0, 4.8, 6.6});
  xyz.insert(xyz.end(), vertical.begin(), vertical.end());
  AisleExtractionParameters parameters;
  parameters.z_min_m = 0.0;
  parameters.z_max_m = 2.0;
  AisleProposalGenerationResult result;
  QString error;

  EXPECT_FALSE(generate_aisle_proposals(xyz, kSquareBoundary, parameters, &result, &error));
  EXPECT_NE(error.indexOf(QStringLiteral("weak or ambiguous")), -1);
}

TEST(AisleProposalGeneratorTest, RejectsUnusuallyWideGapInsteadOfCallingItAnAisle) {
  const auto xyz = greenhouse_rows({1.2, 5.2});
  AisleExtractionParameters parameters;
  parameters.z_min_m = 0.0;
  parameters.z_max_m = 2.0;
  AisleProposalGenerationResult result;
  QString error;

  ASSERT_TRUE(generate_aisle_proposals(xyz, kBoundary, parameters, &result, &error)) << error.toStdString();
  ASSERT_EQ(result.supported_row_count, 2);
  EXPECT_TRUE(result.proposals.isEmpty());
}

TEST(AisleProposalGeneratorTest, RejectsInvalidBoundaryAndEmptyHeightBand) {
  auto xyz = greenhouse_rows({1.2, 3.0, 4.8, 6.6});
  AisleExtractionParameters parameters;
  AisleProposalGenerationResult result;
  QString error;
  const QVector<QPointF> degenerate{{0.0, 0.0}, {1.0, 1.0}, {2.0, 2.0}};
  EXPECT_FALSE(generate_aisle_proposals(xyz, degenerate, parameters, &result, &error));
  EXPECT_NE(error.indexOf(QStringLiteral("nonzero area")), -1);

  parameters.z_min_m = 20.0;
  parameters.z_max_m = 25.0;
  EXPECT_FALSE(generate_aisle_proposals(xyz, kBoundary, parameters, &result, &error));
  EXPECT_NE(error.indexOf(QStringLiteral("Fewer than 100")), -1);
}
