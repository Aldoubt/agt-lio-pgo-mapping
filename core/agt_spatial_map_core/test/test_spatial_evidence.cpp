#include "agt_spatial_map_core/spatial_evidence.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <stdexcept>

namespace agt_spatial_map_core {
namespace {

SpatialVoxelEvidence observed(std::uint32_t n, std::uint32_t span) {
  SpatialVoxelEvidence v;
  v.point_count = n;
  v.observed_keyframes = n;
  v.first_keyframe = 1;
  v.last_keyframe = span + 1;
  v.keyframe_span = span;
  return v;
}

TEST(VoxelIndexTest, FloorsPositiveNegativeAndExactBoundariesLikeLegacyFilter) {
  EXPECT_EQ((voxel_for(Eigen::Vector3f(0.0F, 0.199F, -0.001F), 0.2F)),
            (VoxelKey{0, 0, -1}));
  EXPECT_EQ((voxel_for(Eigen::Vector3f(0.2F, 0.4F, -0.2F), 0.2F)),
            (VoxelKey{1, 2, -1}));
  EXPECT_EQ((voxel_for(Eigen::Vector3f(-0.201F, -0.4F, 0.0F), 0.2F)),
            (VoxelKey{-2, -2, 0}));
  EXPECT_THROW(voxel_for(Eigen::Vector3f(std::numeric_limits<float>::quiet_NaN(), 0, 0),
                         0.2F), std::invalid_argument);
  EXPECT_THROW(voxel_for(Eigen::Vector3f::Zero(), 0.0F), std::invalid_argument);
}

TEST(ConfidenceTest, MatchesSingleSessionFormulaAndIsMonotone) {
  ConfidenceParameters p;
  auto a = observed(1, 0);
  auto b = observed(2, 3);
  auto c = observed(4, 3);
  auto d = observed(4, 6);
  for (auto *v : {&a, &b, &c, &d}) calculate_confidence(v, p);
  EXPECT_FLOAT_EQ(a.persistence_score, 0.0F);
  EXPECT_FLOAT_EQ(a.auto_confidence, 0.0F);
  EXPECT_LT(a.auto_confidence, b.auto_confidence);
  EXPECT_LT(b.auto_confidence, c.auto_confidence);
  EXPECT_GE(d.auto_confidence, c.auto_confidence);
  EXPECT_NEAR(c.observation_score, 1.0 - std::exp(-1.0), 1e-6);
  EXPECT_NEAR(c.persistence_score, std::pow(1.0 - std::exp(-1.0), 0.7), 1e-6);
  EXPECT_FLOAT_EQ(c.auto_confidence, c.final_confidence);
  EXPECT_GE(c.final_confidence, p.stable_threshold);
}

TEST(ConfidenceTest, ManualOverrideSurvivesAutomaticRecalculation) {
  ConfidenceParameters p;
  auto v = observed(4, 3);
  calculate_confidence(&v, p);
  const float automatic = v.auto_confidence;
  v.override_mode = ManualOverrideMode::FORCE_HIGH;
  calculate_confidence(&v, p);
  EXPECT_EQ(v.auto_confidence, automatic);
  EXPECT_FLOAT_EQ(v.final_confidence, 1.0F);
  v.override_mode = ManualOverrideMode::FORCE_LOW;
  calculate_confidence(&v, p);
  EXPECT_FLOAT_EQ(v.final_confidence, p.force_low_value);
  v.has_manual_value = true;
  v.manual_value = 0.12F;
  calculate_confidence(&v, p);
  EXPECT_FLOAT_EQ(v.final_confidence, 0.12F);
  v.override_mode = ManualOverrideMode::IGNORE;
  calculate_confidence(&v, p);
  EXPECT_FLOAT_EQ(v.final_confidence, 0.0F);
  EXPECT_FLOAT_EQ(v.auto_confidence, automatic);
  v.override_mode = ManualOverrideMode::AUTO;
  calculate_confidence(&v, p);
  EXPECT_FLOAT_EQ(v.final_confidence, automatic);
  EXPECT_EQ(parse_override_mode(override_mode_name(ManualOverrideMode::IGNORE)),
            ManualOverrideMode::IGNORE);
}

TEST(ConfidenceTest, RejectsInvalidEvidenceAndParameters) {
  ConfidenceParameters p;
  p.persistence_alpha = 1.2F;
  EXPECT_THROW(validate_parameters(p), std::invalid_argument);
  p.persistence_alpha = 0.7F;
  p.stable_threshold = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(validate_parameters(p), std::invalid_argument);
  p.stable_threshold = 0.60F;
  auto v = observed(3, 5);
  v.keyframe_span = 4;
  EXPECT_THROW(calculate_confidence(&v, p), std::invalid_argument);
  v.keyframe_span = 5;
  v.geometry_score = std::numeric_limits<float>::quiet_NaN();
  EXPECT_THROW(calculate_confidence(&v, p), std::invalid_argument);
  EXPECT_THROW(parse_override_mode("P_STABLE"), std::invalid_argument);
}

}  // namespace
}  // namespace agt_spatial_map_core
