#include "viewer/PointCloudRenderSampler.hpp"

#include <gtest/gtest.h>

using agt_map_studio::make_render_point_subset;

TEST(PointCloudRenderSampler, CapsOnlyRenderSubsetAndIsDeterministic) {
  std::vector<float> xyz;
  for (int i = 0; i < 100; ++i) {
    xyz.insert(xyz.end(), {static_cast<float>(i), 0.0F, static_cast<float>(i % 5)});
  }
  const auto first = make_render_point_subset(xyz, 10, false, 0.0, 4.0);
  const auto second = make_render_point_subset(xyz, 10, false, 0.0, 4.0);
  ASSERT_EQ(first.source_indices.size(), 10U);
  EXPECT_EQ(first.source_indices, second.source_indices);
  EXPECT_EQ(first.xyz, second.xyz);
  EXPECT_LT(first.source_indices.front(), 10U);
  EXPECT_GT(first.source_indices.back(), 89U);
  EXPECT_EQ(xyz.size(), 300U);  // canonical source remains untouched
}

TEST(PointCloudRenderSampler, ZFilterIsAppliedBeforeTheRenderCap) {
  const std::vector<float> xyz{
      0, 0, -2, 1, 0, -1, 2, 0, 0, 3, 0, 1,
      4, 0, 2, 5, 0, 3, 6, 0, 4};
  const auto subset = make_render_point_subset(xyz, 2, true, 0.0, 3.0);
  ASSERT_EQ(subset.source_indices.size(), 2U);
  EXPECT_EQ(subset.source_indices.front(), 3U);
  EXPECT_EQ(subset.source_indices.back(), 5U);
  EXPECT_EQ(subset.xyz[2], 1.0F);
  EXPECT_EQ(subset.xyz[5], 3.0F);
}

TEST(PointCloudRenderSampler, ZeroCapMeansFullEligibleCloud) {
  const std::vector<float> xyz{0, 0, 0, 1, 0, 1, 2, 0, 2};
  const auto subset = make_render_point_subset(xyz, 0, true, 1.0, 2.0);
  EXPECT_EQ(subset.source_indices, (std::vector<std::size_t>{1U, 2U}));
}
