#pragma once

#include "StridedView.h"

#include <Althea/Application.h>
#include <Althea/BindlessHandle.h>
#include <Althea/DeferredRendering.h>
#include <Althea/DynamicVertexBuffer.h>
#include <Althea/FrameContext.h>
#include <Althea/Framebuffer.h>
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

struct AABBPushConstants {
  AABBHandles handles;
  uint32_t globalResources;
  uint32_t globalUniforms;
  uint32_t flags;
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

  uint32_t getInnerNodeCount() const { return m_innerNodes.size(); }
  uint32_t getLeafCount() const { return m_leaves.size(); }

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
  AABBManager() = default;
  AABBManager(
      Application& app,
      const GBufferResources& gBuffer,
      GlobalHeap& heap,
      uint32_t leafCount);

  // TODO: Generalize?
  void update(
      const StridedView<uint32_t>& tris,
      const StridedView<glm::vec3>& verts,
      const FrameContext& frame);
  void updateUI();

  void debugDraw(
      const Application& app,
      VkCommandBuffer commandBuffer,
      const FrameContext& frame,
      VkDescriptorSet heapSet,
      BufferHandle globalResources,
      UniformHandle globalUniforms) const;

  void tryRecompile(Application& app) { m_dbgRenderPass.tryRecompile(app); }

private:
  AABBTree m_tree;

  RenderPass m_dbgRenderPass;
  FrameBuffer m_dbgFrameBuffer;
};
} // namespace RibCage