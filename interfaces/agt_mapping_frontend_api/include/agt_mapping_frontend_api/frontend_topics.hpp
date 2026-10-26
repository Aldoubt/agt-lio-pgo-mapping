#pragma once

#include <string>

namespace agt_mapping_frontend_api
{

inline constexpr char kDefaultNamespace[] = "/mapping/frontend";
inline constexpr char kOdometrySuffix[] = "odometry";
inline constexpr char kCloudSuffix[] = "cloud";
inline constexpr char kPathSuffix[] = "path";
inline constexpr char kStatusSuffix[] = "status";

struct FrontendTopics
{
  std::string odometry;
  std::string cloud;
  std::string path;
  std::string status;
};

inline FrontendTopics makeFrontendTopics(const std::string & output_namespace)
{
  std::string prefix = output_namespace.empty() ? kDefaultNamespace : output_namespace;
  if (prefix.front() != '/') {
    prefix.insert(prefix.begin(), '/');
  }
  while (prefix.size() > 1 && prefix.back() == '/') {
    prefix.pop_back();
  }
  return {prefix + "/" + kOdometrySuffix, prefix + "/" + kCloudSuffix,
          prefix + "/" + kPathSuffix, prefix + "/" + kStatusSuffix};
}

}  // namespace agt_mapping_frontend_api
