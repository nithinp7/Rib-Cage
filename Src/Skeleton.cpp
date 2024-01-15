#include "Skeleton.h"

#include <Althea/Application.h>
#include <Althea/Utilities.h>

#include <memory>

using namespace AltheaEngine;

namespace RibCage {

/*static*/
bool SkeletonLoader::load(const std::string& path, Skeleton& result) {
  if (!Utilities::checkFileExists(path))
    return false;

  std::vector<char> data = Utilities::readFile(path);
  if (data.size() != sizeof(Skeleton))
    return false;

  memcpy(&result, data.data(), sizeof(Skeleton));
  return true;
}

/*static*/
bool SkeletonLoader::save(const std::string& path, const Skeleton& skeleton) {
  return Utilities::writeFile(
      path,
      gsl::span((const char*)&skeleton, sizeof(skeleton)));
}
} // namespace RibCage