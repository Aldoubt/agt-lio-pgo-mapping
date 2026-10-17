#include "confidence/SpatialConfidenceEditor.hpp"
#include "confidence/SpatialConfidenceIntentIO.hpp"
#include "confidence/SpatialConfidenceLoader.hpp"
#include "selection/SelectionManager.h"

#include <agt_spatial_map_core/spatial_export.hpp>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace fs = std::filesystem;
namespace core = agt_spatial_map_core;
using namespace agt_map_studio;

namespace {
class ConfidenceRoundTripTest : public ::testing::Test {
protected:
  fs::path root_, parent_, source_;
  SpatialConfidenceModel model_;
  SpatialConfidenceEditor editor_;
  void SetUp() override {
    static unsigned serial = 0;
    root_ = fs::temp_directory_path() /
            ("agt_studio_review_roundtrip_" + std::to_string(getpid()) + "_" +
             std::to_string(++serial));
    parent_ = root_ / "original_map_package";
    source_ = root_ / "source_derivative";
    fs::create_directories(parent_);
    write(parent_ / "manifest.yaml", "schema_version: 1\n");
    write(parent_ / "checksums.sha256", "fixture checksum index\n");
    core::SpatialEvidenceMap voxels;
    auto add = [&voxels](core::VoxelKey key, unsigned n, unsigned span) {
      core::SpatialVoxelEvidence v;
      v.key = key;
      v.centroid = Eigen::Vector3f(key.x * .2F + .05F, 0.05F, 0.05F);
      v.point_count = n + 2;
      v.observed_keyframes = n;
      v.last_keyframe = span;
      v.keyframe_span = span;
      core::calculate_confidence(&v, core::ConfidenceParameters{});
      voxels.emplace(key, v);
    };
    add({0, 0, 0}, 4, 4);
    add({1, 0, 0}, 4, 4);
    add({-2, 0, 0}, 1, 0);
    add({-3, 0, 0}, 1, 0);
    write(root_ / "initial.yaml",
          "schema_version: 1\ncoordinate_system: voxel_index\nvoxel_size: 0.2\n"
          "overrides: [{key: [0, 0, 0], mode: IGNORE, reason: PARKING_AREA}, "
          "{key: [-2, 0, 0], mode: FORCE_HIGH, reason: MANUAL_ANCHOR}]\n");
    core::SpatialExportOptions options;
    options.parent_package = parent_;
    options.output_directory = source_;
    options.manual_overrides = root_ / "initial.yaml";
    options.build_stats = core::EvidenceBuildStats{7, 20, 19, 1, 4};
    core::export_spatial_artifacts(&voxels, options);
    std::string error;
    if (!SpatialConfidenceLoader::load(source_, parent_, &model_, &error)) {
      throw std::runtime_error(error);
    }
    editor_.set_model(&model_);
  }
  void TearDown() override { fs::remove_all(root_); }
  static void write(const fs::path &path, const std::string &contents) {
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    file << contents;
  }
  std::size_t index(std::int64_t x) const {
    for (std::size_t i = 0; i < model_.voxels().size(); ++i) {
      if (model_.voxels()[i].key == core::VoxelKey{x, 0, 0}) return i;
    }
    throw std::runtime_error("missing test voxel");
  }
};

TEST_F(ConfidenceRoundTripTest, SaveIntentThenCoreReviewThenVerifiedLoaderPreservesAuto) {
  EXPECT_EQ(editor_.stable_preview_count(), 2U);
  const auto source_hash = core::sha256_file(source_ / "confidence_voxels.pcd");
  const auto parent_hash = core::sha256_file(parent_ / "manifest.yaml");
  std::string error;
  EXPECT_TRUE(editor_.restore_auto({index(0), index(-2)}, &error)) << error;
  ConfidenceOverrideIntent high;
  high.mode = core::ManualOverrideMode::FORCE_HIGH;
  high.audit = {"MANUAL_ANCHOR", "2026-09-27T12:34:56Z", "yangxuan"};
  ASSERT_TRUE(editor_.apply({index(-3)}, high, &error)) << error;
  ConfidenceOverrideIntent low;
  low.mode = core::ManualOverrideMode::FORCE_LOW;
  low.has_manual_value = true;
  low.manual_value = .9F;  // FORCE_LOW excludes even if numerical value is high
  low.audit = {"LOW_GEOMETRY", "2026-09-27T12:36:56Z", "yangxuan"};
  ASSERT_TRUE(editor_.apply({index(1)}, low, &error)) << error;
  EXPECT_EQ(editor_.stable_preview_count(), 2U);
  EXPECT_TRUE(editor_.dirty());
  const auto intent = root_ / "my_review_intent.yaml";
  ASSERT_TRUE(SpatialConfidenceIntentIO::save(intent, model_, editor_, false, &error)) << error;
  std::ifstream saved_yaml(intent);
  const std::string saved_text((std::istreambuf_iterator<char>(saved_yaml)),
                               std::istreambuf_iterator<char>());
  EXPECT_NE(saved_text.find("edited_at: \"2026-09-27T12:36:56Z\""), std::string::npos)
      << "Saving an intent must keep audited times as strings across YAML readers";
  const auto yaml = YAML::LoadFile(intent.string());
  ASSERT_EQ(yaml.size(), 4U);  // no unsanctioned Phase-1 schema extension
  EXPECT_EQ(yaml["schema_version"].as<int>(), 1);
  EXPECT_EQ(yaml["coordinate_system"].as<std::string>(), "voxel_index");
  EXPECT_EQ(yaml["voxel_size"].as<float>(), model_.info().voxel_size);
  ASSERT_EQ(yaml["overrides"].size(), 2U);
  EXPECT_EQ(yaml["overrides"][0]["key"][0].as<int>(), -3);
  EXPECT_EQ(yaml["overrides"][1]["key"][0].as<int>(), 1);
  EXPECT_EQ(yaml["overrides"][1]["reason"].as<std::string>(), "LOW_GEOMETRY");
  EXPECT_EQ(yaml["overrides"][1]["edited_at"].as<std::string>(), "2026-09-27T12:36:56Z");
  EXPECT_EQ(yaml["overrides"][1]["editor"].as<std::string>(), "yangxuan");
  EXPECT_TRUE(editor_.dirty());  // file I/O does not silently reset editor state
  editor_.mark_saved();
  EXPECT_FALSE(editor_.dirty());
  const auto target = root_ / "reviewed_new_directory";
  const auto result = core::review_spatial_artifacts(source_, parent_, intent, target);
  EXPECT_EQ(result.stable_voxel_count, editor_.stable_preview_count());
  SpatialConfidenceModel reviewed;
  ASSERT_TRUE(SpatialConfidenceLoader::load(target, parent_, &reviewed, &error)) << error;
  EXPECT_EQ(reviewed.stable_preview_count(), editor_.stable_preview_count());
  EXPECT_EQ(reviewed.find({0, 0, 0})->override_mode, core::ManualOverrideMode::AUTO);
  EXPECT_EQ(reviewed.find({-2, 0, 0})->override_mode, core::ManualOverrideMode::AUTO);
  EXPECT_EQ(reviewed.find({1, 0, 0})->audit.reason, "LOW_GEOMETRY");
  EXPECT_EQ(reviewed.find({1, 0, 0})->final_confidence, .9F);
  EXPECT_EQ(reviewed.find({1, 0, 0})->geometry_score, 1.0F);
  EXPECT_EQ(reviewed.find({1, 0, 0})->auto_confidence,
            model_.find({1, 0, 0})->auto_confidence);
  EXPECT_EQ(core::sha256_file(source_ / "confidence_voxels.pcd"), source_hash);
  EXPECT_EQ(core::sha256_file(parent_ / "manifest.yaml"), parent_hash);
  EXPECT_TRUE(editor_.undo());
  EXPECT_TRUE(editor_.dirty());
  EXPECT_TRUE(editor_.redo());
  EXPECT_FALSE(editor_.dirty());
}

TEST_F(ConfidenceRoundTripTest, VoxelSelectionsReuseGeometryButNeverBecomeRawPointDeletes) {
  SelectionManager raw_points;
  SelectionManager confidence_voxels;
  raw_points.reset(9);  // 9 raw PCD records are NOT 4 voxel representatives
  confidence_voxels.reset(model_.voxels().size());
  SelectionGeometry old_delete;
  old_delete.rule_type = "remove_box";
  raw_points.select_points({1, 3}, old_delete);
  ASSERT_TRUE(raw_points.delete_selected());
  EXPECT_EQ(raw_points.deleted_count(), 2U);
  const auto source_final = model_.find({0, 0, 0})->final_confidence;
  const auto source_hash = core::sha256_file(source_ / "confidence_voxels.pcd");

  SelectionGeometry rectangle;
  rectangle.rule_type = "remove_box";
  confidence_voxels.select_points({index(1), index(-3)}, rectangle);
  EXPECT_EQ(confidence_voxels.selected_indices().size(), 2U);
  EXPECT_EQ(confidence_voxels.deleted_count(), 0U);
  EXPECT_EQ(confidence_voxels.selection_geometry().rule_type, "remove_box");
  ConfidenceOverrideIntent high;
  high.mode = core::ManualOverrideMode::FORCE_HIGH;
  high.audit.reason = "MANUAL_ANCHOR";
  std::string error;
  ASSERT_TRUE(editor_.apply(confidence_voxels.selected_indices(), high, &error)) << error;
  EXPECT_EQ(editor_.effective_mode(index(-3)), core::ManualOverrideMode::FORCE_HIGH);
  EXPECT_EQ(raw_points.deleted_count(), 2U);
  EXPECT_EQ(raw_points.history().size(), 1U);  // old Delete mode remains unchanged
  EXPECT_EQ(confidence_voxels.deleted_count(), 0U);

  SelectionGeometry polygon;
  polygon.rule_type = "remove_polygon";
  polygon.has_z_range = true;
  polygon.z_min = -1.0;
  polygon.z_max = 3.0;
  confidence_voxels.select_points({index(0)}, polygon);
  EXPECT_EQ(confidence_voxels.selection_geometry().rule_type, "remove_polygon");
  ConfidenceOverrideIntent ignore;
  ignore.mode = core::ManualOverrideMode::IGNORE;
  ignore.audit.reason = "OTHER";
  ASSERT_TRUE(editor_.apply(confidence_voxels.selected_indices(), ignore, &error)) << error;
  EXPECT_FALSE(editor_.stable_preview(index(0)));

  SelectionGeometry sphere;
  sphere.rule_type = "remove_sphere";
  sphere.center = Eigen::Vector3d(0.0, 0.0, 0.0);
  sphere.radius = .5;
  confidence_voxels.select_points({index(-2)}, sphere);
  EXPECT_EQ(confidence_voxels.selection_geometry().rule_type, "remove_sphere");
  ASSERT_TRUE(editor_.apply(confidence_voxels.selected_indices(), ignore, &error)) << error;
  EXPECT_EQ(editor_.effective_mode(index(-2)), core::ManualOverrideMode::IGNORE);
  EXPECT_TRUE(editor_.undo());
  EXPECT_EQ(editor_.effective_mode(index(-2)), core::ManualOverrideMode::FORCE_HIGH);
  EXPECT_TRUE(editor_.redo());
  EXPECT_EQ(editor_.effective_mode(index(-2)), core::ManualOverrideMode::IGNORE);
  EXPECT_EQ(model_.find({0, 0, 0})->final_confidence, source_final);
  EXPECT_EQ(raw_points.deleted_count(), 2U);
  EXPECT_EQ(core::sha256_file(source_ / "confidence_voxels.pcd"), source_hash);
}

TEST_F(ConfidenceRoundTripTest, RefusesSourceOverwriteAndRequiresExplicitReplacementOfDraft) {
  const auto source_hash = core::sha256_file(source_ / "manual_overrides.yaml");
  std::string error;
  EXPECT_FALSE(SpatialConfidenceIntentIO::save(source_ / "manual_overrides.yaml",
                                               model_, editor_, true, &error));
  EXPECT_NE(error.find("outside"), std::string::npos) << error;
  EXPECT_FALSE(SpatialConfidenceIntentIO::save(parent_ / "human.yaml",
                                               model_, editor_, false, &error));
  EXPECT_NE(error.find("outside"), std::string::npos) << error;
  const auto alias = root_ / "source_alias";
  fs::create_directory_symlink(source_, alias);
  EXPECT_FALSE(SpatialConfidenceIntentIO::save(alias / "manual_overrides.yaml",
                                               model_, editor_, true, &error));
  EXPECT_EQ(core::sha256_file(source_ / "manual_overrides.yaml"), source_hash);
  const auto draft = root_ / "my_draft.yaml";
  ASSERT_TRUE(SpatialConfidenceIntentIO::save(draft, model_, editor_, false, &error)) << error;
  const auto first_hash = core::sha256_file(draft);
  EXPECT_FALSE(SpatialConfidenceIntentIO::save(draft, model_, editor_, false, &error));
  EXPECT_NE(error.find("exists"), std::string::npos) << error;
  ConfidenceOverrideIntent high;
  high.mode = core::ManualOverrideMode::FORCE_HIGH;
  high.audit.reason = "MANUAL_ANCHOR";
  ASSERT_TRUE(editor_.apply({index(-3)}, high, &error)) << error;
  ASSERT_TRUE(SpatialConfidenceIntentIO::save(draft, model_, editor_, true, &error)) << error;
  EXPECT_NE(core::sha256_file(draft), first_hash);
  const auto dangerous = root_ / "symlinked.yaml";
  fs::create_symlink(source_ / "manual_overrides.yaml", dangerous);
  EXPECT_FALSE(SpatialConfidenceIntentIO::save(dangerous, model_, editor_, true, &error));
  EXPECT_EQ(core::sha256_file(source_ / "manual_overrides.yaml"), source_hash);
}
}  // namespace
