#pragma once

#include "SceneElement.h"

#include <Althea/ImageResource.h>
#include <Althea/IndexBuffer.h>
#include <Althea/SingleTimeCommandBuffer.h>
#include <Althea/StructuredBuffer.h>
#include <Althea/VertexBuffer.h>
#include <Althea/DeferredRendering.h>
#include <Althea/FrameContext.h>
#include <Althea/RenderPass.h>
#include <Althea/Framebuffer.h>
#include <Althea/GlobalHeap.h>
#include <Althea/Model.h>
#include <glm/glm.hpp>

using namespace AltheaEngine;

namespace RibCage {
class GltfTestScene : public ISceneElement, public IGBufferSubpass {
public:
  GltfTestScene() = default;
  virtual ~GltfTestScene() = default;
  
  // ISceneElement impl
  void init(
      Application& app,
      SingleTimeCommandBuffer& commandBuffer,
      SceneToGBufferPassBuilder& gBufferPassBuilder,
      GlobalHeap& heap) override;
  void update(const FrameContext& frame) override;
  void updateUI() override;

  // IGBufferSubpass impl
  void registerGBufferSubpass(GraphicsPipelineBuilder& builder) const override;
  void beginGBufferSubpass(
      const DrawContext& context,
      BufferHandle globalResourcesHandle,
      UniformHandle globalUniformsHandle) override;

private:
  std::vector<Model> m_models;
  StructuredBuffer<PrimitiveConstants> m_constantBuffer;
  
};
} // namespace 