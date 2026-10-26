#pragma once

#include "agt_pcd2grid_exporter/ParameterLoader.hpp"

#include <cerrno>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <sys/wait.h>
#include <unistd.h>

#ifndef AGT_PCD2GRID_VERSION
#define AGT_PCD2GRID_VERSION "0.1.0"
#endif

namespace agt_pcd2grid_exporter {
namespace detail {

inline bool within(const std::filesystem::path &path, const std::filesystem::path &root) {
  auto p = path.begin();
  for (auto r = root.begin(); r != root.end(); ++r, ++p) {
    if (p == path.end() || *p != *r) return false;
  }
  return true;
}

// Use the installed Python authority for source hashes/manifests. No shell,
// ad-hoc hash implementation, or source-tree discovery/fallback is involved.
inline void artifact_helper(const std::vector<std::string> &arguments) {
  std::vector<std::string> command{"python3", "-m", "agt_mapping_artifacts.derived_export"};
  command.insert(command.end(), arguments.begin(), arguments.end());
  std::vector<char *> argv;
  for (auto &value : command) argv.push_back(value.data());
  argv.push_back(nullptr);
  const pid_t child = fork();
  if (child < 0) throw std::runtime_error("cannot start installed artifact integrity helper");
  if (child == 0) {
    execvp(argv[0], argv.data());
    _exit(127);
  }
  int status = 0;
  pid_t waited;
  do { waited = waitpid(child, &status, 0); } while (waited < 0 && errno == EINTR);
  if (waited < 0 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    throw std::runtime_error("artifact integrity/publication helper refused output; "
                             "install agt_mapping_artifacts and check its diagnostic");
  }
}

struct CloudPublication {
  std::filesystem::path stage, output, cloud_directory;

  ~CloudPublication() {
    if (!stage.empty()) {
      std::error_code ignored;
      std::filesystem::remove_all(stage, ignored);
    }
  }

  void prepare(const std::string &source, const std::string &destination,
               const std::string &subdirectory, const ProjectionParameters &parameters,
               const std::string &projection_source) {
    namespace fs = std::filesystem;
    const auto parent_source = fs::canonical(source);
    if (fs::exists(destination) || fs::is_symlink(fs::symlink_status(destination))) {
      throw std::invalid_argument("cleanup output already exists; choose a new directory");
    }
    output = fs::weakly_canonical(fs::absolute(destination));
    if (within(output, parent_source) || within(parent_source, output)) {
      throw std::invalid_argument("cleanup output must be disjoint from the immutable source");
    }
    if (fs::exists(output) || fs::is_symlink(fs::symlink_status(output))) {
      throw std::invalid_argument("cleanup output already exists; choose a new directory");
    }
    const fs::path relative = subdirectory.empty() ? fs::path(".") : fs::path(subdirectory);
    if (relative.is_absolute()) throw std::invalid_argument("cloud subdirectory must be relative");
    for (const auto &part : relative) {
      if (part == "..") throw std::invalid_argument("cloud subdirectory must remain inside output");
    }
    fs::create_directories(output.parent_path());
    std::string pattern = (output.parent_path() / ("." + output.filename().string() + ".staging-XXXXXX")).string();
    std::vector<char> writable(pattern.begin(), pattern.end());
    writable.push_back('\0');
    const char *created = mkdtemp(writable.data());
    if (!created) throw std::runtime_error("cannot create cleanup staging directory");
    stage = created;
    cloud_directory = stage / relative;
    std::string error;
    if (!ParameterLoader::write((stage / "projection.yaml").string(), parameters, &error)) {
      throw std::runtime_error(error);
    }
    std::vector<std::string> arguments{"prepare", "--source", source, "--stage", stage.string(),
        "--output", output.string(), "--producer-version", AGT_PCD2GRID_VERSION,
        "--cloud-subdirectory", relative.string()};
    if (!projection_source.empty()) {
      fs::copy_file(projection_source, stage / "input_projection.yaml");
      ProjectionParameters reloaded;
      if (!ParameterLoader::load((stage / "input_projection.yaml").string(), &reloaded, &error) ||
          !ParameterLoader::write((stage / ".projection_check.yaml").string(), reloaded, &error)) {
        throw std::runtime_error(error);
      }
      const auto bytes = [](const fs::path &path) {
        std::ifstream stream(path, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
      };
      if (bytes(stage / "projection.yaml") != bytes(stage / ".projection_check.yaml")) {
        throw std::runtime_error("selected projection config changed after parameter loading");
      }
      fs::remove(stage / ".projection_check.yaml");
      arguments.insert(arguments.end(), {"--projection-source", projection_source});
    }
    artifact_helper(arguments);
  }

  void publish() {
    artifact_helper({"finish", "--stage", stage.string(), "--output", output.string()});
    stage.clear();
  }
};

}  // namespace detail
}  // namespace agt_pcd2grid_exporter
