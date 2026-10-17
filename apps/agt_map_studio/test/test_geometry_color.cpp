#include "geometry/GeometryColor.hpp"

#include <gtest/gtest.h>

namespace {
using namespace agt_map_studio;
using agt_spatial_map_core::GeometryVoxelEvidence;

TEST(GeometryColor, InsufficientEvidenceIsGrayNotFabricatedZero) {
  GeometryVoxelEvidence v;
  for (auto mode : {GeometryColorMode::NormalShape, GeometryColorMode::TranslationQ,
                    GeometryColorMode::RotationQ, GeometryColorMode::TranslationWeak,
                    GeometryColorMode::RotationWeak}) {
    const auto c = geometry_color(v, mode);
    EXPECT_FLOAT_EQ(c.r, c.g);
    EXPECT_FLOAT_EQ(c.g, c.b);
    EXPECT_GT(c.r, 0.4F);
  }
  v.normal_valid = true;
  v.linearity = .1F; v.planarity = .8F; v.scattering = .1F;
  const auto shape = geometry_color(v, GeometryColorMode::NormalShape);
  EXPECT_GT(shape.g, shape.r);
  EXPECT_GT(shape.g, shape.b);
  v.translation.valid = true; v.translation.isotropy = 0.0F;
  v.translation.weak_direction = Eigen::Vector3f::UnitX();
  v.rotation.valid = true; v.rotation.isotropy = 1.0F;
  v.rotation.weak_direction = -Eigen::Vector3f::UnitZ();
  const auto low = geometry_color(v, GeometryColorMode::TranslationQ);
  const auto high = geometry_color(v, GeometryColorMode::RotationQ);
  EXPECT_NE(low.r, high.r);
  const auto tx = geometry_color(v, GeometryColorMode::TranslationWeak);
  const auto rz = geometry_color(v, GeometryColorMode::RotationWeak);
  EXPECT_FLOAT_EQ(tx.r, 1.0F);
  EXPECT_FLOAT_EQ(tx.g, 0.0F);
  EXPECT_FLOAT_EQ(rz.b, 1.0F); // eigenvector sign is irrelevant
}
}  // namespace
