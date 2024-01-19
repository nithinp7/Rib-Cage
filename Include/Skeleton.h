#pragma once

#include "DebugTools.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace RibCage {

#define INVALID_BONE_IDX -1
#define MAX_BONE_COUNT 32

struct Bone {
  glm::mat4 transform;
  int childrenStartIdx = -1;
  int childrenCount = 0;
};

struct Skeleton {
  Bone bones[MAX_BONE_COUNT];
  int boneCount = 0;
};

class SkeletonLoader {
public:
  static bool load(const std::string& path, Skeleton& result);
  static bool save(const std::string& path, const Skeleton& skeleton);

private:
};

class SkeletonEditor {
public:
  SkeletonEditor() = default;

  void update(
      SelectableScene& scene,
      const glm::vec3& cameraPos,
      const glm::vec3& cursorDir,
      uint32_t prevInputMask,
      uint32_t inputMask);

private:
  Skeleton m_skeleton{};

  int m_gizmoAxis{};
  bool m_bDraggingGizmo{};
  glm::vec3 m_originalGizmoPosition{};
  
  // Constraints for spawning new vertices
  glm::vec3 m_constraintPlane;
  glm::vec3 m_constraintPoint;
};
} // namespace RibCage