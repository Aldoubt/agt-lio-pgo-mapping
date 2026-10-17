#include "agt_spatial_map_core/spatial_evidence_builder.hpp"
#include "agt_spatial_map_core/spatial_export.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <filesystem>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace {
namespace fs = std::filesystem;
using namespace agt_spatial_map_core;

void usage(std::ostream &out) {
  out << "agt_spatial_map_export --map-package <optimized PGO map_package> "
         "--output-dir <new-or-empty-dir> [--config <v1.yaml>] "
         "[--manual-overrides <v1.yaml>] [--voxel-size <m>] "
         "[--n0 <keyframes>] [--s0 <index-span>] [--alpha <0..1>] "
         "[--stable-threshold <0..1>] [--force-low-value <0..1>]\n"
         "Review (no automatic evidence recalculation): --map-package <same PGO parent> "
         "--source-derivative <verified derivative> --manual-overrides <saved v1 intent.yaml> "
         "--output-dir <new-or-empty-dir>\n";
}

float number(const std::string &text) {
  std::size_t consumed = 0;
  const auto value = std::stof(text, &consumed);
  if (consumed != text.size()) throw std::invalid_argument("invalid numeric parameter: " + text);
  return value;
}

void verify_parent_with_existing_validator(const fs::path &package) {
  // No shell interpolation: the original artifact's validator checks full
  // checksum coverage, source paths, PGO optimization and nonempty map PCD.
  const auto filename = package.string();
  const pid_t child = fork();
  if (child < 0) throw std::runtime_error("cannot fork mapping artifact validator");
  if (child == 0) {
    execlp("python3", "python3", "-m", "agt_mapping_artifacts.validation",
           filename.c_str(), static_cast<char *>(nullptr));
    _exit(127);
  }
  int status = 0;
  while (waitpid(child, &status, 0) == -1) {
    if (errno != EINTR) throw std::runtime_error("failed to wait for mapping artifact validator");
  }
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    throw std::runtime_error("mapping artifact validation failed (check optimized PGO, "
                             "checksum coverage and ROS workspace Python overlay)");
  }
}

}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--help") {
      usage(std::cout);
      return 0;
    }
    const std::set<std::string> supported = {
        "--map-package", "--output-dir", "--config", "--manual-overrides",
        "--source-derivative", "--voxel-size", "--n0", "--s0", "--alpha",
        "--stable-threshold", "--force-low-value"};
    std::map<std::string, std::string> arguments;
    for (int i = 1; i < argc; i += 2) {
      const std::string name = argv[i];
      if (!supported.count(name) || i + 1 >= argc ||
          !arguments.emplace(name, argv[i + 1]).second) {
        throw std::invalid_argument("unknown, repeated, or missing CLI argument: " + name);
      }
    }
    if (!arguments.count("--map-package") || !arguments.count("--output-dir")) {
      throw std::invalid_argument("--map-package and --output-dir are required");
    }
    const auto package = fs::path(arguments.at("--map-package"));
    const auto output = fs::path(arguments.at("--output-dir"));
    if (!fs::is_directory(package / "patches") ||
        !fs::is_regular_file(package / "poses_timed.txt")) {
      throw std::invalid_argument("not an optimized mapping package with patches/ + poses_timed.txt; "
                                  "a navigation release cannot supply keyframe evidence");
    }
    if (fs::is_symlink(fs::symlink_status(output)) ||
        (fs::exists(output) && (!fs::is_directory(output) || !fs::is_empty(output)))) {
      throw std::invalid_argument("output exists and is nonempty, not a directory, or a symlink");
    }
    const auto config = arguments.count("--config") ? fs::path(arguments.at("--config")) : fs::path();
    auto p = load_confidence_config(config);
    if (arguments.count("--voxel-size")) p.voxel_size = number(arguments.at("--voxel-size"));
    if (arguments.count("--n0")) p.observation_reference = number(arguments.at("--n0"));
    if (arguments.count("--s0")) p.keyframe_span_reference = number(arguments.at("--s0"));
    if (arguments.count("--alpha")) p.persistence_alpha = number(arguments.at("--alpha"));
    if (arguments.count("--stable-threshold")) p.stable_threshold = number(arguments.at("--stable-threshold"));
    if (arguments.count("--force-low-value")) p.force_low_value = number(arguments.at("--force-low-value"));
    validate_parameters(p);

    if (arguments.count("--source-derivative")) {
      if (!arguments.count("--manual-overrides") || arguments.count("--config") ||
          arguments.count("--voxel-size") || arguments.count("--n0") ||
          arguments.count("--s0") || arguments.count("--alpha") ||
          arguments.count("--stable-threshold") || arguments.count("--force-low-value")) {
        throw std::invalid_argument("review mode requires saved intent YAML and uses ONLY the "
                                    "verified source derivative's unchanged confidence parameters");
      }
      verify_parent_with_existing_validator(package);
      const auto summary = review_spatial_artifacts(
          arguments.at("--source-derivative"), package,
          arguments.at("--manual-overrides"), output,
          [&package] { verify_parent_with_existing_validator(package); });
      std::cout << "Reviewed spatial confidence published: " << summary.output_directory << '\n'
                << "Voxels: " << summary.voxel_count
                << "; stable voxels: " << summary.stable_voxel_count
                << "; manual overrides: " << summary.manual_override_count
                << "; automatic evidence: preserved from verified source derivative\n";
      return 0;
    }
    verify_parent_with_existing_validator(package);
    EvidenceBuildStats stats;
    auto voxels = SpatialEvidenceBuilder::build(package, p, &stats);
    SpatialExportOptions options;
    options.parent_package = package;
    options.output_directory = output;
    options.parameters = p;
    options.build_stats = stats;
    if (arguments.count("--manual-overrides")) {
      options.manual_overrides = fs::path(arguments.at("--manual-overrides"));
    }
    options.before_publish = [&package] { verify_parent_with_existing_validator(package); };
    const auto summary = export_spatial_artifacts(&voxels, options);
    std::cout << "Spatial confidence published: " << summary.output_directory << '\n'
              << "Keyframes: " << stats.source_keyframes << "; voxels: " << summary.voxel_count
              << "; stable voxels: " << summary.stable_voxel_count
              << "; manual overrides: " << summary.manual_override_count << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "agt_spatial_map_export: " << error.what() << '\n';
    usage(std::cerr);
    return 1;
  }
}
