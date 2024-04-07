#pragma once

#include <Althea/Application.h>
#include <Althea/BufferHeap.h>
#include <Althea/ComputePipeline.h>
#include <Althea/DeferredRendering.h>
#include <Althea/DynamicVertexBuffer.h>
#include <Althea/GlobalHeap.h>
#include <Althea/IndexBuffer.h>
#include <Althea/RenderPass.h>
#include <Althea/SingleTimeCommandBuffer.h>
#include <Althea/StructuredBuffer.h>
#include <Althea/TransientUniforms.h>
#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

using namespace AltheaEngine;

namespace RibCage {
struct ClothUniforms {
  uint32_t globalResources;
  uint32_t globalUniforms;

  float deltaTime;

  uint32_t nodes;
  uint32_t nodePositions;
  uint32_t nodesCount;

  uint32_t distanceConstraints;
  uint32_t distanceConstraintsCount;
};

struct ClothSection {
  IndexBuffer indices;
};

struct Node {
  alignas(16) glm::vec3 position;
  alignas(4) uint32_t objectIdx;
};

struct DistanceConstraint {
  uint32_t a;
  uint32_t b;
  float restLength;
  float padding;
};

class ClothSim {
public:
  ClothSim() = default;
  ClothSim(
      Application& app,
      SingleTimeCommandBuffer& commandBuffer,
      const GBufferResources& gBuffer,
      GlobalHeap& heap);

void tryRecompileShaders(Application& app);

void update(const FrameContext& frame);

void draw(
    const Application& app,
    VkCommandBuffer commandBuffer,
    const FrameContext& frame,
    VkDescriptorSet heapSet,
    BufferHandle globalResourcesHandle,
    UniformHandle globalUniformsHandle);

private:
  TransientUniforms<ClothUniforms> m_uniforms;

  RenderPass m_renderPass;
  FrameBuffer m_frameBuffer;

  std::vector<ComputePipeline> m_solvePasses;

  // TODO: These probably need to become paged heaps
  StructuredBuffer<Node> m_nodes;
  StructuredBuffer<DistanceConstraint> m_distanceConstraints;
  std::vector<glm::vec3> m_prevPositions;
  DynamicVertexBuffer<glm::vec3> m_nodePositions;

  std::vector<ClothSection> m_clothSections;
};
} // namespace RibCage