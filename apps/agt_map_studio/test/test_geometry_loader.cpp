#include "geometry/GeometryEvidenceLoader.hpp"
#include "confidence/SpatialConfidenceLoader.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace agt_map_studio;

namespace {
TEST(GeometryModel, VoxelKeyMappingIsNotPcdPointIndexOrUnverifiedRowOrder) {
  SpatialConfidenceModel confidence;
  ConfidenceArtifactInfo info;
  info.voxel_size = .2F;
  ConfidenceVoxel first, second;
  first.key = {1, 0, 0}; first.center = Eigen::Vector3f(.25F, .05F, .05F);
  second.key = {0, 0, 0}; second.center = Eigen::Vector3f(.05F, .05F, .05F);
  confidence.assign(info, {first, second}); // deliberately REVERSED order
  agt_spatial_map_core::GeometryEvidence geometry;
  geometry.voxel_size = .2F;
  agt_spatial_map_core::GeometryVoxelEvidence a, b;
  a.key = {0, 0, 0}; a.center = second.center; a.normal_support_points = 11;
  b.key = {1, 0, 0}; b.center = first.center; b.normal_support_points = 23;
  geometry.voxels = {a, b};
  GeometryEvidenceModel model;
  model.assign(geometry, confidence, "/tmp/sidecar");
  ASSERT_NE(model.at_confidence_index(0), nullptr);
  EXPECT_EQ(model.at_confidence_index(0)->key.x, 1);
  EXPECT_EQ(model.at_confidence_index(0)->normal_support_points, 23U);
  EXPECT_EQ(model.at_confidence_index(1)->key.x, 0);
  EXPECT_EQ(model.at_confidence_index(1)->normal_support_points, 11U);
  EXPECT_EQ(model.at_confidence_index(999), nullptr);
  geometry.voxels.pop_back();
  EXPECT_THROW(model.assign(geometry, confidence, "/tmp/wrong"), std::invalid_argument);
  EXPECT_EQ(model.at_confidence_index(0)->normal_support_points, 23U);
}

TEST(GeometryLoader, RealVerifiedSidecarBindsToV1AndFailureKeepsPreviousSnapshot) {
  const char *geo = std::getenv("AGT_GEOMETRY_TEST_DIR");
  const char *parent = std::getenv("AGT_SPATIAL_PARENT_TEST_DIR");
  const char *confidence = std::getenv("AGT_SPATIAL_DERIVATIVE_TEST_DIR");
  if (!geo || !parent || !confidence) return; // synthetic core IO tests run on every build
  SpatialConfidenceModel v1;
  std::string error;
  ASSERT_TRUE(SpatialConfidenceLoader::load(confidence, parent, &v1, &error)) << error;
  GeometryEvidenceModel model;
  ASSERT_TRUE(GeometryEvidenceLoader::load(geo, parent, confidence, v1, &model, &error)) << error;
  ASSERT_EQ(model.evidence().voxels.size(), v1.voxels().size());
  ASSERT_EQ(model.evidence().voxels.size(), 333839U);
  for (std::size_t index : {std::size_t{0}, std::size_t{12345}, std::size_t{333838}}) {
    const auto *v = model.at_confidence_index(index);
    ASSERT_NE(v, nullptr);
    EXPECT_EQ(v->key, v1.voxels()[index].key);
    EXPECT_EQ(v->center, v1.voxels()[index].center);
  }
  const auto first_key = model.at_confidence_index(0)->key;
  EXPECT_FALSE(GeometryEvidenceLoader::load(parent, parent, confidence, v1, &model, &error));
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(model.at_confidence_index(0)->key, first_key);
  EXPECT_FALSE(GeometryEvidenceLoader::load(geo, confidence, confidence, v1, &model, &error));
  EXPECT_EQ(model.at_confidence_index(0)->key, first_key);
}
}  // namespace
