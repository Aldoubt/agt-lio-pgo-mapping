#pragma once

#include "agt_spatial_map_core/spatial_evidence_builder.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <string>

namespace agt_spatial_map_core {

struct SpatialExportOptions {
  std::filesystem::path parent_package;      // already verified optimized PGO map_package
  std::filesystem::path output_directory;    // new or empty directory outside parent
  std::filesystem::path manual_overrides;     // optional input schema v1; never edited
  ConfidenceParameters parameters;
  EvidenceBuildStats build_stats;
  // Executed after staged artifacts and checksums exist but before atomic
  // publication. The CLI re-verifies the parent here; tests inject a failure.
  std::function<void()> before_publish;
};

struct SpatialExportSummary {
  std::uint64_t voxel_count = 0;
  std::uint64_t stable_voxel_count = 0;
  std::uint64_t manual_override_count = 0;
  std::filesystem::path output_directory;
};

// Mutates manual/final confidence in the supplied map only. Writes all five
// derivative files to a sibling staging directory, then atomically renames
// that directory into place. Failure removes staging; parent is read-only.
// Caller MUST verify parent manifest/checksums using agt_mapping_artifacts
// before invoking this library; the installed CLI does so twice.
SpatialExportSummary export_spatial_artifacts(
    SpatialEvidenceMap *evidence, const SpatialExportOptions &options);

// Loads schema_version: 1 config, rejects unknown fields and other geometry
// modes; absent config uses ConfidenceParameters' versioned defaults.
ConfidenceParameters load_confidence_config(const std::filesystem::path &config);

// For metadata provenance and checksum coverage; streaming file digest.
std::string sha256_file(const std::filesystem::path &file);

}  // namespace agt_spatial_map_core
