#include "confidence/SpatialConfidenceEditor.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

namespace core = agt_spatial_map_core;
using namespace agt_map_studio;

namespace {
struct Fixture {
  SpatialConfidenceModel model;
  SpatialConfidenceEditor editor;
  Fixture() {
    ConfidenceArtifactInfo info;
    info.voxel_size = .2F;
    info.stable_threshold = .6F;
    info.force_low_value = .05F;
    std::vector<ConfidenceVoxel> voxels;
    // 5 x 5 x 3 voxel lattice. Raw cloud point indices are never used here.
    for (int z = 0; z < 3; ++z) {
      for (int y = 0; y < 5; ++y) {
        for (int x = 0; x < 5; ++x) {
          ConfidenceVoxel v;
          v.key = {x - 2, y - 2, z};
          v.center = Eigen::Vector3f(x * .2F, y * .2F, z * .2F);
          v.point_count = 10;
          v.observed_keyframes = 3;
          v.geometry_score = 1.0F;
          v.auto_confidence = z == 0 ? .7F : .4F;
          v.final_confidence = v.auto_confidence;
          voxels.push_back(v);
        }
      }
    }
    model.assign(info, std::move(voxels));
    editor.set_model(&model);
  }
  static ConfidenceOverrideIntent intent(core::ManualOverrideMode mode,
                                          const char *reason = "PARKING_AREA") {
    ConfidenceOverrideIntent v;
    v.mode = mode;
    v.audit = {reason, "2026-09-27T12:34:56Z", "map_studio"};
    return v;
  }
  std::vector<std::size_t> all() const {
    std::vector<std::size_t> result;
    for (std::size_t i = 0; i < model.voxels().size(); ++i) result.push_back(i);
    return result;
  }
};

TEST(SpatialConfidenceEditorTest, BatchUndoRedoRestoreAutoAndStablePreview) {
  Fixture f;
  const auto all = f.all();
  ASSERT_EQ(all.size(), 75U);
  EXPECT_EQ(f.editor.stable_preview_count(), 25U);
  EXPECT_FALSE(f.editor.dirty());
  std::string error;
  const auto high = Fixture::intent(core::ManualOverrideMode::FORCE_HIGH);
  EXPECT_TRUE(f.editor.apply(all, high, &error)) << error;
  EXPECT_EQ(f.editor.stable_preview_count(), 75U);
  EXPECT_TRUE(f.editor.dirty());
  EXPECT_FLOAT_EQ(f.model.voxels()[50].auto_confidence, .4F);
  EXPECT_FLOAT_EQ(f.model.voxels()[50].final_confidence, .4F);  // immutable derivative
  EXPECT_FLOAT_EQ(f.editor.preview_final(50), 1.0F);
  EXPECT_TRUE(f.editor.undo());
  EXPECT_EQ(f.editor.stable_preview_count(), 25U);
  EXPECT_FALSE(f.editor.dirty());
  EXPECT_TRUE(f.editor.redo());
  EXPECT_EQ(f.editor.stable_preview_count(), 75U);
  EXPECT_TRUE(f.editor.restore_auto(all, &error)) << error;
  EXPECT_EQ(f.editor.stable_preview_count(), 25U);
  EXPECT_FALSE(f.editor.dirty());
  EXPECT_TRUE(f.editor.undo());
  EXPECT_EQ(f.editor.stable_preview_count(), 75U);
  EXPECT_TRUE(f.editor.redo());
  EXPECT_EQ(f.editor.stable_preview_count(), 25U);
}

TEST(SpatialConfidenceEditorTest, ForceLowAlwaysExcludedAndReasonDoesNotChangeGeometry) {
  Fixture f;
  auto low = Fixture::intent(core::ManualOverrideMode::FORCE_LOW, "LOW_GEOMETRY");
  low.has_manual_value = true;
  low.manual_value = .9F;  // even above threshold, FORCE_LOW is excluded
  std::string error;
  EXPECT_TRUE(f.editor.apply({0, 1}, low, &error)) << error;
  EXPECT_FLOAT_EQ(f.editor.preview_final(0), .9F);
  EXPECT_FALSE(f.editor.stable_preview(0));
  EXPECT_EQ(f.editor.stable_preview_count(), 23U);
  EXPECT_FLOAT_EQ(f.model.voxels()[0].geometry_score, 1.0F);
  EXPECT_FLOAT_EQ(f.model.voxels()[0].auto_confidence, .7F);
  const auto output = f.editor.sorted_intents();
  ASSERT_EQ(output.size(), 2U);
  EXPECT_TRUE(output[0].first < output[1].first);
  EXPECT_EQ(output[0].second.audit.reason, "LOW_GEOMETRY");
  f.editor.mark_saved();
  EXPECT_FALSE(f.editor.dirty());
  EXPECT_TRUE(f.editor.undo());
  EXPECT_TRUE(f.editor.dirty());
  EXPECT_TRUE(f.editor.redo());
  EXPECT_FALSE(f.editor.dirty());
}

TEST(SpatialConfidenceEditorTest, RejectsMalformedSelectionOrAuditWithoutPartialMutation) {
  Fixture f;
  std::string error;
  EXPECT_FALSE(f.editor.apply({0, 74, 75},
                              Fixture::intent(core::ManualOverrideMode::FORCE_HIGH), &error));
  EXPECT_NE(error.find("outside"), std::string::npos);
  EXPECT_EQ(f.editor.override_count(), 0U);
  EXPECT_EQ(f.editor.stable_preview_count(), 25U);
  auto invalid = Fixture::intent(core::ManualOverrideMode::FORCE_HIGH, "FAKE_PROBABILITY");
  EXPECT_FALSE(f.editor.apply({1}, invalid, &error));
  EXPECT_NE(error.find("reason"), std::string::npos);
  invalid = Fixture::intent(core::ManualOverrideMode::FORCE_HIGH);
  invalid.has_manual_value = true;
  invalid.manual_value = .8F;
  EXPECT_FALSE(f.editor.apply({1}, invalid, &error));
  EXPECT_NE(error.find("FORCE_LOW"), std::string::npos);
  EXPECT_FALSE(f.editor.dirty());
  EXPECT_FALSE(f.editor.undo());
  EXPECT_FALSE(f.editor.redo());
}

TEST(SpatialConfidenceEditorTest, LoadsExistingReviewedIntentAndRestoresBaseline) {
  Fixture f;
  ConfidenceArtifactInfo info = f.model.info();
  auto voxels = f.model.voxels();
  voxels[50].has_override_entry = true;
  voxels[50].override_mode = core::ManualOverrideMode::FORCE_HIGH;
  voxels[50].final_confidence = 1.0F;
  voxels[50].audit = {"MANUAL_ANCHOR", "2026-09-27T12:34:56Z", "map_studio"};
  f.editor.set_model(nullptr);
  f.model.assign(info, std::move(voxels));
  f.editor.set_model(&f.model);
  EXPECT_EQ(f.editor.stable_preview_count(), 26U);
  ASSERT_NE(f.editor.intent(50), nullptr);
  EXPECT_EQ(f.editor.intent(50)->audit.reason, "MANUAL_ANCHOR");
  EXPECT_FALSE(f.editor.dirty());
  std::string error;
  EXPECT_TRUE(f.editor.restore_auto({50}, &error)) << error;
  EXPECT_EQ(f.editor.stable_preview_count(), 25U);
  EXPECT_TRUE(f.editor.dirty());
  EXPECT_TRUE(f.editor.undo());
  EXPECT_FALSE(f.editor.dirty());
  EXPECT_EQ(f.editor.stable_preview_count(), 26U);
}
}  // namespace
