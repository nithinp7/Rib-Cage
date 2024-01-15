#pragma once 

#include <glm/glm.hpp>
#include <cstdint>
#include <string>

namespace RibCage {

#define INVALID_BONE_IDX -1
#define MAX_BONE_COUNT 32

struct Bone {
  glm::mat4 transform;
  int childrenStartIdx;
  int childrenCount;
};

struct Skeleton {  
  Bone bones[MAX_BONE_COUNT];
};

class SkeletonLoader {
public:
  static bool load(const std::string& path, Skeleton& result);
  static bool save(const std::string& path, const Skeleton& skeleton);

private:
};
} // namespace RibCage