#pragma once

#include "SceneElement.h"
#include "AABBTree.h"
#include "Collisions.h"

#include <Althea/Application.h>
#include <Althea/BufferHeap.h>
#include <Althea/ComputePipeline.h>
#include <Althea/Containers/StackVector.h>
#include <Althea/DeferredRendering.h>
#include <Althea/DynamicVertexBuffer.h>
#include <Althea/GlobalHeap.h>
#include <Althea/IndexBuffer.h>
#include <Althea/IntrusivePtr.h>
#include <Althea/RenderPass.h>
#include <Althea/SingleTimeCommandBuffer.h>
#include <Althea/StructuredBuffer.h>
#include <Althea/TransientUniforms.h>
#include <glm/glm.hpp>
#include <gsl/span>

#include <cstdint>
#include <vector>

using namespace AltheaEngine;

namespace RibCage {
struct ClothUniforms {
  uint32_t globalResources;
  uint32_t globalUniforms;

  float deltaTime;

  uint32_t nodesCount;
  uint32_t nodePositions;
  uint32_t nodeFlags;

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

struct FixedPositionConstraint {
  alignas(16) glm::vec3 position;
  alignas(4) uint32_t nodeIdx;
};

class ClothSim : public ISceneElement, public IGBufferSubpass{
public:
  ClothSim() = default;
  virtual ~ClothSim() = default;

  // ISceneElement impl
  void init(
      Application& app,
      SingleTimeCommandBuffer& commandBuffer,
      SceneToGBufferPassBuilder& gBufferPassBuilder,
      GlobalHeap& heap) override;
  void tryRecompileShaders(Application& app) override;
  void update(const FrameContext& frame) override;
  void preDraw(
      const Application& app,
      VkCommandBuffer commandBuffer,
      const FrameContext& frame,
      VkDescriptorSet heapSet,
      BufferHandle globalResourcesHandle,
      UniformHandle globalUniformsHandle) override;
  void updateUI() override;

  // IGBufferSubpass impl
  void registerGBufferSubpass(GraphicsPipelineBuilder& builder) const override;
  void beginGBufferSubpass(
      const DrawContext& context,
      BufferHandle globalResourcesHandle,
      UniformHandle globalUniformsHandle) override;

private:
  void _resetPositions();
  void _updateConstraints();

  float _computeConstraintResiduals(
      StackVector<glm::vec3>& residual,
      StackVector<float>& wSum) const;
  void _computeASearchDir(
      StackVector<glm::vec3>& A_searchDir,
      const StackVector<glm::vec3>& searchDir,
      const StackVector<float>& wSum) const;

  void _conjugateGradientSolve();
  void _projectedGaussSeidelSolve();

  TransientUniforms<ClothUniforms> m_uniforms;

  std::vector<ComputePipeline> m_solvePasses;

  // These will eventually be GPU visible
  std::vector<FixedPositionConstraint> m_positionConstraints;
  gsl::span<FixedPositionConstraint> m_clothTop;
  gsl::span<FixedPositionConstraint> m_clothBottom;

  std::vector<DistanceConstraint> m_distanceConstraints;
  std::vector<glm::vec3> m_prevPositions;

  DynamicVertexBuffer<glm::vec3> m_nodePositions;
  DynamicVertexBuffer<uint32_t> m_nodeFlags;

  std::vector<ClothSection> m_clothSections;

  IntrusivePtr<AABBManager> m_aabb;
  CollisionsManager m_collisions;
};
} // namespace RibCage