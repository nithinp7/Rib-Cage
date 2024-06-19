#pragma once

#include <Althea/Application.h>
#include <Althea/DrawContext.h>
#include <Althea/DeferredRendering.h>
#include <Althea/FrameContext.h>
#include <Althea/GlobalHeap.h>
#include <Althea/IntrusivePtr.h>
#include <Althea/SingleTimeCommandBuffer.h>
#include <vulkan/vulkan.h>

#include <memory>

using namespace AltheaEngine;

namespace RibCage {
class ISceneElement : public virtual RefCounted {
public:
  ISceneElement() = default;
  virtual ~ISceneElement() = default;

  virtual void init(
      Application& app,
      SingleTimeCommandBuffer& commandBuffer,
      SceneToGBufferPassBuilder& gBufferPassBuilder,
      GlobalHeap& heap) = 0;

  virtual void tryRecompileShaders(Application& app) {}

  virtual void update(const FrameContext& frame) {}

  virtual void preDraw(
      const Application& app,
      VkCommandBuffer commandBuffer,
      const FrameContext& frame,
      VkDescriptorSet heapSet,
      BufferHandle globalResourcesHandle,
      UniformHandle globalUniformsHandle) {}

  virtual void updateUI() {}
};
} // namespace RibCage