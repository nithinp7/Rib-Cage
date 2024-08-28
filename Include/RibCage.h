#pragma once

#include "SceneElement.h"
#include "Skeleton.h"
#include "DebugTools.h"
#include "ClothSim.h"
#include "SimpleObjLoader.h"

#include <Althea/Allocator.h>
#include <Althea/CameraController.h>
#include <Althea/ComputePipeline.h>
#include <Althea/DeferredRendering.h>
#include <Althea/DescriptorSet.h>
#include <Althea/FrameBuffer.h>
#include <Althea/IGameInstance.h>
#include <Althea/Image.h>
#include <Althea/ImageBasedLighting.h>
#include <Althea/ImageResource.h>
#include <Althea/ImageView.h>
#include <Althea/IntrusivePtr.h>
#include <Althea/Model.h>
#include <Althea/PerFrameResources.h>
#include <Althea/RenderPass.h>
#include <Althea/Sampler.h>
#include <Althea/ScreenSpaceReflection.h>
#include <Althea/StructuredBuffer.h>
#include <Althea/Texture.h>
#include <Althea/TransientUniforms.h>
#include <Althea/TextureHeap.h>
#include <Althea/GlobalHeap.h>
#include <Althea/GlobalUniforms.h>
#include <Althea/GlobalResources.h>
#include <Althea/OrbitCamera.h>
#include <Althea/StencilQuery.h>
#include <glm/glm.hpp>

#include <vector>
#include <memory>

using namespace AltheaEngine;

namespace AltheaEngine {
class Application;
} // namespace AltheaEngine

namespace RibCage {

extern StencilQueryManager* GStencilQueryManager;

class RibCage : public IGameInstance {
public:
  RibCage();
  // virtual ~RibCage();

  void initGame(Application& app) override;
  void shutdownGame(Application& app) override;

  void createRenderState(Application& app) override;
  void destroyRenderState(Application& app) override;

  void tick(Application& app, const FrameContext& frame) override;
  void draw(
      Application& app,
      VkCommandBuffer commandBuffer,
      const FrameContext& frame) override;

private:
  bool m_adjustingExposure = false;

  std::unique_ptr<CameraController> m_pCameraController;
  OrbitCamera m_orbitCamera;
  
  void _createGlobalResources(
      Application& app,
      SingleTimeCommandBuffer& commandBuffer);
  GlobalHeap m_globalHeap;
  GlobalUniformsResource m_globalUniforms;
  GlobalResources m_globalResources;
  SceneToGBufferPass m_gBufferPass;

  void _createDeferredPass(Application& app);
  RenderPass m_deferredPass;
  SwapChainFrameBufferCollection m_swapChainFrameBuffers;

  ScreenSpaceReflection m_SSR;
  float m_exposure = 0.3f;

  SkeletonEditor m_skeletonEditor;
  SelectableScene m_debugScene;

  std::vector<IntrusivePtr<ISceneElement>> m_sceneElements;

  uint32_t m_inputMask = 0;
};
} // namespace RibCage
