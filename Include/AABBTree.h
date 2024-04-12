#pragma once

#include "StridedView.h"

#include <Althea/Application.h>
#include <Althea/DeferredRendering.h>
#include <Althea/DynamicVertexBuffer.h>
#include <Althea/Framebuffer.h>
#include <Althea/FrameContext.h>
#include <Althea/GlobalHeap.h>
#include <Althea/RenderPass.h>
#include <glm/glm.hpp>
#include <vulkan/vulkan.h>

#include <cstdint>
#include <vector>

using namespace AltheaEngine;

namespace RibCage {

enum AABBInnerNodeFlagBits { InnerNodeFlag_HasLeafChildren = 1 };
struct AABBInnerNode {
  alignas(16) glm::vec3 min;
  alignas(4) uint32_t childA;
  alignas(16) glm::vec3 max;
  alignas(4) uint32_t childB;
  uint32_t flags;
  uint32_t padding;
};

struct AABBLeaf {
  alignas(16) glm::vec3 min;
  alignas(4) uint32_t triIdx;
  alignas(16) glm::vec3 max;
  alignas(4) uint32_t padding;
};

struct AABBHandles {
  uint32_t innerNodes;
  uint32_t leaves;
};

class AABBTree {
public:
  AABBTree() = default;
  AABBTree(Application& app, GlobalHeap& heap, uint32_t leafCount);

  void upload(uint32_t ringBufferIndex);

  AABBHandles getHandles(uint32_t ringBufferIndex) const;

  void refitTriangles(
      const StridedView<uint32_t>& tris,
      const StridedView<glm::vec3>& verts,
      float padding);

private:
  void populateInnerNode(
      const glm::vec3& min,
      const glm::vec3& max,
      gsl::span<uint32_t> sortedLeaves);

  std::vector<AABBInnerNode> m_innerNodes;
  DynamicVertexBuffer<AABBInnerNode> m_innerNodesBuffer;

  std::vector<AABBLeaf> m_leaves;
  DynamicVertexBuffer<AABBLeaf> m_leavesBuffer;
};

class AABBManager {
public:
  AABBManager(
      Application& app,
      const GBufferResources& gBuffer,
      GlobalHeap& heap);

  void update(const FrameContext& frame);
  
private:
  AABBTree m_tree;

  RenderPass m_dbgRenderPass;
  FrameBuffer m_dbgFrameBuffer;
};
} // namespace RibCage