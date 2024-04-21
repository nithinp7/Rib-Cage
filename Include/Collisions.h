#pragma once

#include "AABBTree.h"

#include <Althea/Application.h>
#include <Althea/Containers/StridedView.h>
#include <glm/glm.hpp>

#include <cstdint>

using namespace AltheaEngine;

namespace RibCage {
struct CollisionConstraint {
  uint32_t primA;
  uint32_t primB;

  glm::vec3 normal;
};

class Collisions {
public:
  Collisions(
      const StridedView<uint32_t>& indices,
      const StridedView<glm::vec3>& positions,
      const StridedView<glm::vec3>& prevPositions,
      const AABBTree& aabb);
  const std::vector<CollisionConstraint>& getCollisions() const {
    return m_collisions;
  }

private:
  std::vector<CollisionConstraint> m_collisions;
};

class CollisionsManager {
public:
  CollisionsManager() = default;
  CollisionsManager(
      Application& app,
      const GBufferResources& gBuffer,
      GlobalHeap& heap);

private:
  Collisions m_collisions;
};
} // namespace RibCage