#pragma once

#include "AABBTree.h"
#include "DebugTools.h"

#include <Althea/Application.h>
#include <Althea/FrameContext.h>
#include <Althea/Containers/StridedView.h>
#include <Althea/BindlessHandle.h>
#include <glm/glm.hpp>

#include <vulkan/vulkan.h>

#include <cstdint>

using namespace AltheaEngine;

namespace RibCage {
struct PointPointCollision {
  uint32_t triIdxA : 30;
  uint32_t pointIdxA : 2;
  uint32_t triIdxB : 30;
  uint32_t pointIdxB : 2;

  glm::vec3 normal;
};

struct PointTriangleCollision {
  uint32_t pointIdx;
  uint32_t triangleIdx : 31;
  uint32_t bBackFace : 1;
};

class Collisions {
public:
  Collisions() = default;
  void update(
      const StridedView<uint32_t>& indices,
      const StridedView<glm::vec3>& positions,
      const StridedView<glm::vec3>& prevPositions,
      float thresholdDistance,
      const AABBTree& aabb);
  const std::vector<PointPointCollision>& getPointCollisions() const {
    return m_pointCollisions;
  }

private:
  std::vector<PointPointCollision> m_pointCollisions;
};

class CollisionsManager {
public:
  CollisionsManager() = default;
  CollisionsManager(
      Application& app,
      VkCommandBuffer commandBuffer,
      const GBufferResources& gBuffer,
      GlobalHeap& heap);

  void updateUI();
  void update(
      const FrameContext& frame,
      const StridedView<uint32_t>& indices,
      const StridedView<glm::vec3>& positions,
      const StridedView<glm::vec3>& prevPositions,
      const AABBTree& aabb);

void draw(
    const Application& app,
    VkCommandBuffer commandBuffer,
    const FrameContext& frame,
    VkDescriptorSet heapSet,
    UniformHandle globalUniformsHandle);

private:
  Collisions m_collisions;
  SelectableScene m_scene;
};
} // namespace RibCage