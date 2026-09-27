#pragma once

#include "agt_spatial_map_core/spatial_evidence.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace agt_spatial_map_core {

// Phase 3A is a separate, read-only evidence chain. It does NOT change V1
// geometry_score (which remains 1), auto_confidence, or stable selection.
struct GeometryParameters {
  float normal_radius = 0.40F;          // meters, raw map-frame patch points
  std::uint32_t normal_min_points = 8;  // raw neighbors (including the voxel's points)
  float observability_radius = 0.80F;   // meters, centers of valid-normal voxels
  std::uint32_t min_valid_normals = 6;  // distinct voxel normals, not raw points
  double normal_epsilon_m2 = 1e-6;       // PCA lambda_mid, units m^2
  double translation_epsilon = 1e-6;    // Ht eigen/trace, dimensionless
  double rotation_epsilon_m2 = 1e-6;     // Hr eigen/trace, units m^2
};

void validate_geometry_parameters(const GeometryParameters &p);
// Strict schema_version: 1, rejects unknown/missing keys. Empty path uses
// the versioned defaults above. This file NEVER configures V1 confidence.
GeometryParameters load_geometry_config(const std::filesystem::path &path);

struct GeometrySpectrum {
  bool valid = false;
  // min, mid, max eigenvalues of Ht (unitless) / Hr (meters squared).
  Eigen::Vector3f eigenvalues = Eigen::Vector3f::Constant(
      std::numeric_limits<float>::quiet_NaN());
  Eigen::Vector3f weak_direction = Eigen::Vector3f::Constant(
      std::numeric_limits<float>::quiet_NaN());  // eigenvector at min, map frame, sign arbitrary
  float isotropy = std::numeric_limits<float>::quiet_NaN(); // Q = 3*min / (trace+eps)
  float condition = std::numeric_limits<float>::quiet_NaN(); // max / max(min,eps)
};

struct GeometryVoxelEvidence {
  VoxelKey key;
  Eigen::Vector3f center = Eigen::Vector3f::Zero(); // exactly the V1 voxel centroid
  bool normal_valid = false;
  std::uint32_t normal_support_points = 0;
  Eigen::Vector3f normal = Eigen::Vector3f::Constant(
      std::numeric_limits<float>::quiet_NaN());
  Eigen::Vector3f covariance_eigenvalues = Eigen::Vector3f::Constant(
      std::numeric_limits<float>::quiet_NaN()); // min, mid, max in m^2
  float linearity = std::numeric_limits<float>::quiet_NaN();
  float planarity = std::numeric_limits<float>::quiet_NaN();
  float scattering = std::numeric_limits<float>::quiet_NaN();
  std::uint32_t valid_normal_voxels = 0; // in observability radius
  std::uint64_t supporting_observations = 0; // points in contributing voxels
  GeometrySpectrum translation; // Ht = sum per-voxel n*n^T (dimensionless)
  GeometrySpectrum rotation;    // Hr = sum per-voxel mean(g*g^T), g=(p-t_body)xn (meters)
};

struct GeometryEvidence {
  std::vector<GeometryVoxelEvidence> voxels; // ascending VoxelKey, one per V1 key
  float voxel_size = 0.0F;
  GeometryParameters parameters;
  std::uint64_t source_observations = 0;
  std::uint32_t source_keyframes = 0;
};

struct GeometryObservation {
  Eigen::Vector3f position_map = Eigen::Vector3f::Zero();
  Eigen::Vector3f body_origin_map = Eigen::Vector3f::Zero(); // T_map_body.translation()
};

// The synthetic entry point accepts the same transformed observations as the
// real patch reader. It checks EVERY key's point count and centroid against
// the immutable V1 evidence; no invented normals or geometry scores.
class GeometryEvidenceEstimator {
public:
  static GeometryEvidence estimate(const SpatialEvidenceMap &confidence,
                                   const std::vector<GeometryObservation> &observations,
                                   float voxel_size, const GeometryParameters &parameters,
                                   std::uint32_t keyframes = 0);
  static GeometryEvidence build(const std::filesystem::path &parent_package,
                                const SpatialEvidenceMap &confidence,
                                float voxel_size, const GeometryParameters &parameters,
                                std::uint32_t expected_keyframes,
                                std::uint64_t expected_usable_points);
};

struct GeometryExportOptions {
  std::filesystem::path parent_package;    // CLI validates the optimized PGO parent
  std::filesystem::path confidence_source; // verified Phase 1/2 five-file derivative
  std::filesystem::path output_directory;  // separate new/empty directory
  std::function<void()> before_publish;    // CLI re-validates parent + config here
};

// Library verifies full V1 source identity, key/centroid coverage, and SHA-256
// provenance. CLI ALSO verifies the optimized PGO parent before and after build.
// Writes only geometry_voxels.pcd, geometry_metadata.yaml, checksums.sha256.
void export_geometry_evidence(const GeometryEvidence &geometry,
                              const GeometryExportOptions &options);

// Strict three-file checksum, schema, field/type and per-voxel validator. Also
// verifies the expected parent and V1 confidence source, including its SHA-256
// checksum-index digest, and rejects unexpected entries/symlinks. UI read-only.
GeometryEvidence load_geometry_evidence(const std::filesystem::path &sidecar,
                                        const std::filesystem::path &parent_package,
                                        const std::filesystem::path &confidence_source);

}  // namespace agt_spatial_map_core
