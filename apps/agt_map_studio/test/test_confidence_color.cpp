#include "confidence/ConfidenceColor.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

using namespace agt_map_studio;

namespace {
float brightness(ConfidenceRgb color) {
  return 0.2126F * color.r + 0.7152F * color.g + 0.0722F * color.b;
}

TEST(ConfidenceColorTest, FiveStopsAreFiniteBoundedDeterministicAndMonotoneInLightness) {
  float previous = -1.0F;
  for (const float q : {0.0F, 0.25F, 0.5F, 0.75F, 1.0F}) {
    const auto color = confidence_color(q);
    const auto repeat = confidence_color(q);
    for (const float component : {color.r, color.g, color.b}) {
      EXPECT_TRUE(std::isfinite(component));
      EXPECT_GE(component, 0.0F);
      EXPECT_LE(component, 1.0F);
    }
    EXPECT_FLOAT_EQ(color.r, repeat.r);
    EXPECT_FLOAT_EQ(color.g, repeat.g);
    EXPECT_FLOAT_EQ(color.b, repeat.b);
    EXPECT_GT(brightness(color), previous);
    previous = brightness(color);
  }
}

TEST(ConfidenceColorTest, ClampsOutOfBoundsAndNonfinite) {
  EXPECT_FLOAT_EQ(confidence_color(-10).r, confidence_color(0).r);
  EXPECT_FLOAT_EQ(confidence_color(10).g, confidence_color(1).g);
  EXPECT_FLOAT_EQ(confidence_color(std::numeric_limits<float>::quiet_NaN()).b,
                  confidence_color(0).b);
}
}  // namespace
