#pragma once

#include <Althea/Application.h>
#include <Althea/DrawContext.h>
#include <Althea/DeferredRendering.h>
#include <Althea/FrameContext.h>
#include <Althea/GlobalHeap.h>
#include <Althea/SingleTimeCommandBuffer.h>
#include <vulkan/vulkan.h>

using namespace AltheaEngine;

namespace RibCage {
class ISceneElement {
public:
  ISceneElement() = default;
  virtual ~ISceneElement() = default;

  virtual void init(
      Application& app,
      SingleTimeCommandBuffer& commandBuffer,
      const GBufferResources& gBuffer,
      GlobalHeap& heap) = 0;

  virtual bool hasGBufferPass() const { return false; }
  virtual void registerGBufferPass(GraphicsPipelineBuilder& builder) const {}

  virtual void tryRecompileShaders(Application& app) {}

  virtual void update(const FrameContext& frame) {}

  virtual void draw(
      const Application& app,
      VkCommandBuffer commandBuffer,
      const FrameContext& frame,
      VkDescriptorSet heapSet,
      BufferHandle globalResourcesHandle,
      UniformHandle globalUniformsHandle) {}
  virtual void drawGBuffer(
      const DrawContext& context,
      BufferHandle globalResourcesHandle,
      UniformHandle globalUniformsHandle) {}

  virtual void updateUI() {}
};
} // namespace RibCage