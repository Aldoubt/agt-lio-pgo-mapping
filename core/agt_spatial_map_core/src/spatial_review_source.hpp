#pragma once

#include "agt_spatial_map_core/spatial_evidence_builder.hpp"

#include <filesystem>
#include <string>

namespace agt_spatial_map_core {

struct VerifiedReviewSource {
  SpatialEvidenceMap evidence;
  ConfidenceParameters parameters;
  EvidenceBuildStats stats;
  std::filesystem::path derivative;
  std::string checksums_sha256;
};

// Internal core-only loader. No Studio models/PCD point indices cross this
// boundary: this reads and validates the source artifact independently.
VerifiedReviewSource load_verified_review_source(
    const std::filesystem::path &derivative,
    const std::filesystem::path &parent_package);

// Recheck the same five files immediately before a reviewed publication.
void verify_review_source_integrity(const std::filesystem::path &derivative,
                                    const std::string &expected_checksums_sha256);

}  // namespace agt_spatial_map_core
