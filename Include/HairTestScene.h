#pragma once

#include "SceneElement.h"

#include <Althea/../../Shaders/Hair/HairCommon.glsl>
#include <Althea/Allocator.h>
#include <Althea/BufferHeap.h>
#include <Althea/CameraController.h>
#include <Althea/ComputePipeline.h>
#include <Althea/DeferredRendering.h>
#include <Althea/DescriptorSet.h>
#include <Althea/FrameBuffer.h>
#include <Althea/GlobalHeap.h>
#include <Althea/GlobalResources.h>
#include <Althea/GlobalUniforms.h>
#include <Althea/IGameInstance.h>
#include <Althea/Image.h>
#include <Althea/ImageBasedLighting.h>
#include <Althea/ImageResource.h>
#include <Althea/ImageView.h>
#include <Althea/Model.h>
#include <Althea/PerFrameResources.h>
#include <Althea/PointLight.h>
#include <Althea/RenderPass.h>
#include <Althea/Sampler.h>
#include <Althea/ScreenSpaceReflection.h>
#include <Althea/StructuredBuffer.h>
#include <Althea/Texture.h>
#include <Althea/TransientUniforms.h>
#include <Althea/Scene/Floor.h>
#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

using namespace AltheaEngine;

namespace AltheaEngine {
class Application;
} // namespace AltheaEngine

namespace RibCage {

class HairTestScene : public ISceneElement, public IGBufferSubpass {
public:
  HairTestScene() = default;
  virtual ~HairTestScene() = default;

  // ISceneElement impl
  void init(
      Application& app,
      SingleTimeCommandBuffer& commandBuffer,
      SceneToGBufferPassBuilder& gBufferPassBuilder,
      GlobalHeap& heap) override;
  void update(const FrameContext& frame) override;

  void preDraw(
      const Application& app,
      VkCommandBuffer commandBuffer,
      const FrameContext& frame,
      VkDescriptorSet heapSet,
      BufferHandle globalResourcesHandle,
      UniformHandle globalUniformsHandle) override;

  void updateUI() override;

  void tryRecompileShaders(Application& app) override;

  // IGBufferSubpass impl
  void registerGBufferSubpass(GraphicsPipelineBuilder& builder) const override;
  void beginGBufferSubpass(
      const DrawContext& context,
      BufferHandle globalResourcesHandle,
      UniformHandle globalUniformsHandle) override;

private:
  enum ComputePassEnum : uint8_t {
    ADVECT_PARTICLES_PASS = 0,
    FTL_STRANDS_PASS,
    P2G_PASS,
    COMPUTE_PASS_COUNT
  };

  struct PassInfo {
    const char* shaderName = nullptr;
    const char* entryPoint = nullptr;
  };
  const PassInfo m_shaderInfos[COMPUTE_PASS_COUNT] = {
      {"Hair/HairSim.comp", "cs_advectParticles"},
      {"Hair/HairSim.comp", "cs_ftlStrands"},
      {"Hair/HairSim.comp", "cs_p2g_dense"}};

  void _resetParticles(Application& app, VkCommandBuffer commandBuffer);

  void _createSimResources(
      Application& app,
      SingleTimeCommandBuffer& commandBuffer,
      GlobalHeap& heap);
  std::vector<ComputePipeline> m_computePasses;

  TransientUniforms<SimUniforms> m_simUniforms;
  PushConstants m_push;

  StructuredBufferHeap<Particle> m_particleBuffer;
  StructuredBufferHeap<GridCell> m_grid;

  StructuredBufferHeap<Strand> m_strandBuffer;

  struct Mesh {
    VertexBuffer<glm::vec3> vertices;
    IndexBuffer indices;
  };
  Mesh m_sphere;
  Mesh m_box;

  IntrusivePtr<Floor> m_floor;

  bool m_flagReset = false;
  uint32_t m_activeParticleCount = 10000; // 0;
};
} // namespace RibCage
