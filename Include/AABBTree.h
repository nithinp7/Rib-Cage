#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include <Althea/Application.h>
#include <Althea/RenderPass.h>
#include <Althea/Framebuffer.h>
#include <Althea/DeferredRendering.h>

#include <vulkan/vulkan.h>

using namespace AltheaEngine;

namespace RibCage {
class AABBTree {
public:
  AABBTree(const std::vector<uint32_t>& tris, const std::vector<glm::vec3>& verts, float padding);

private:
  void populateInnerNode(const glm::vec3& min, const glm::vec3& max, gsl::span<uint32_t> sortedLeaves);

  struct InnerNode {
    glm::vec3 min;
    glm::vec3 max;
    uint32_t childA;
    uint32_t childB;
  };
  std::vector<InnerNode> m_innerNodes;

  struct Leaf {
    glm::vec3 min;
    glm::vec3 max;
    uint32_t triIdx;
  };
  std::vector<Leaf> m_leaves;
};

class AABBManager {
public:
  AABBManager(Application& app, const GBufferResources& gBuffer, VkDescriptorSetLayout heapLayout);

private:
  RenderPass m_dbgRenderPass;
  FrameBuffer m_dbgFrameBuffer;

};
} // namespace RibCage