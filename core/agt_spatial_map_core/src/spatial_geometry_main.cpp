#include "agt_spatial_map_core/geometry_evidence.hpp"
#include "agt_spatial_map_core/spatial_export.hpp"
#include "spatial_review_source.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace {
namespace fs = std::filesystem;
using namespace agt_spatial_map_core;

bool inside(const fs::path &candidate, const fs::path &base) {
  auto i = candidate.begin();
  for (auto b = base.begin(); b != base.end(); ++b, ++i) {
    if (i == candidate.end() || *i != *b) return false;
  }
  return true;
}

void validate_target_early(const fs::path &output, const fs::path &parent,
                           const fs::path &confidence) {
  const auto target = output.lexically_normal();
  if (target.empty() || target.filename().empty() || target.filename() == "." ||
      target.filename() == "..") throw std::invalid_argument("unsafe geometry output path");
  const auto dir = target.has_parent_path() ? target.parent_path() : fs::path(".");
  if (!fs::is_directory(dir)) throw std::invalid_argument("geometry output parent missing");
  const auto resolved = fs::canonical(dir) / target.filename();
  for (const auto &source : {fs::canonical(parent), fs::canonical(confidence)}) {
    if (inside(resolved, source) || inside(source, resolved)) {
      throw std::invalid_argument("geometry output overlaps a protected source");
    }
  }
  if (fs::is_symlink(fs::symlink_status(resolved)) ||
      (fs::exists(resolved) && (!fs::is_directory(resolved) || !fs::is_empty(resolved)))) {
    throw std::invalid_argument("geometry output is a symlink or nonempty target");
  }
}

void verify_pgo_package(const fs::path &package) {
  if (fs::is_symlink(fs::symlink_status(package)) ||
      !fs::is_directory(package / "patches") ||
      !fs::is_regular_file(package / "poses_timed.txt")) {
    throw std::invalid_argument("geometry requires optimized PGO mapping patches/poses");
  }
  const auto filename = package.string();
  const pid_t child = fork();
  if (child < 0) throw std::runtime_error("cannot fork PGO artifact validator");
  if (child == 0) {
    execlp("python3", "python3", "-m", "agt_mapping_artifacts.validation",
           filename.c_str(), static_cast<char *>(nullptr));
    _exit(127);
  }
  int status = 0;
  while (waitpid(child, &status, 0) == -1) {
    if (errno != EINTR) throw std::runtime_error("cannot wait for PGO artifact validator");
  }
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    throw std::runtime_error("optimized PGO parent checksum / manifest validation failed");
  }
}

void usage() {
  std::cerr << "agt_spatial_geometry_estimate --map-package <verified optimized PGO package> "
               "--confidence-source <verified confidence_v1 derivative> "
               "--output-dir <new/empty separate directory> [--config <geometry_v1.yaml>]\n";
}
}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc == 2 && std::string(argv[1]) == "--help") {
      usage(); return 0;
    }
    std::map<std::string, std::string> arguments;
    const std::set<std::string> supported = {
        "--map-package", "--confidence-source", "--output-dir", "--config"};
    for (int i = 1; i < argc; i += 2) {
      const std::string option(argv[i]);
      if (!supported.count(option) || i + 1 == argc || argv[i + 1][0] == '\0' ||
          !arguments.emplace(option, argv[i + 1]).second) {
        throw std::invalid_argument("unknown, repeated or unpaired CLI argument: " + option);
      }
    }
    for (auto required : {"--map-package", "--confidence-source", "--output-dir"}) {
      if (!arguments.count(required)) throw std::invalid_argument(std::string("missing ") + required);
    }
    const fs::path parent(arguments.at("--map-package"));
    const fs::path confidence(arguments.at("--confidence-source"));
    const fs::path output(arguments.at("--output-dir"));
    if (fs::is_symlink(fs::symlink_status(parent)) ||
        fs::is_symlink(fs::symlink_status(confidence)) ||
        !fs::is_directory(parent) || !fs::is_directory(confidence)) {
      throw std::invalid_argument("PGO/confidence source must be regular directories");
    }
    validate_target_early(output, parent, confidence);
    const fs::path config = arguments.count("--config")
        ? fs::path(arguments.at("--config")) : fs::path();
    const auto parameters = load_geometry_config(config);
    const auto config_digest = config.empty() ? std::string() : sha256_file(config);
    verify_pgo_package(parent);
    auto verified = load_verified_review_source(confidence, parent);
    const auto begin = std::chrono::steady_clock::now();
    auto evidence = GeometryEvidenceEstimator::build(parent, verified.evidence,
        verified.parameters.voxel_size, parameters, verified.stats.source_keyframes,
        verified.stats.usable_points);
    GeometryExportOptions options;
    options.parent_package = parent;
    options.confidence_source = confidence;
    options.output_directory = output;
    options.before_publish = [parent, config, config_digest] {
      verify_pgo_package(parent);
      if (!config.empty() && (fs::is_symlink(fs::symlink_status(config)) ||
          sha256_file(config) != config_digest)) {
        throw std::runtime_error("geometry configuration changed before publication");
      }
    };
    export_geometry_evidence(evidence, options);
    std::uint64_t normals = 0, translation = 0, rotation = 0;
    for (const auto &v : evidence.voxels) {
      normals += v.normal_valid;
      translation += v.translation.valid;
      rotation += v.rotation.valid;
    }
    const double seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();
    std::cout << "Independent geometry evidence published: " << fs::canonical(output) << '\n'
              << "PGO keyframes: " << evidence.source_keyframes
              << "; raw usable observations: " << evidence.source_observations
              << "; voxel keys: " << evidence.voxels.size() << '\n'
              << "Valid normals: " << normals << "; Ht: " << translation
              << "; Hr: " << rotation << "; elapsed: " << seconds << " s\n"
              << "V1 geometry_score, confidence, stable_map and source trees unchanged.\n";
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "agt_spatial_geometry_estimate: " << error.what() << '\n';
    usage(); return 1;
  }
}
