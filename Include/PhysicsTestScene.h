#pragma once

#include "SceneElement.h"

#include <Althea/Animation.h>
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
#include <Althea/Physics/PhysicsSystem.h>
#include <Althea/Scene/Floor.h>
#include <glm/glm.hpp>

using namespace AltheaEngine;
using namespace AltheaEngine::AltheaPhysics;

namespace RibCage {
class PhysicsTestScene : public ISceneElement {
public:
  PhysicsTestScene() = default;
  virtual ~PhysicsTestScene() = default;
  
  // ISceneElement impl
  void init(
      Application& app,
      SingleTimeCommandBuffer& commandBuffer,
      SceneToGBufferPassBuilder& gBufferPassBuilder,
      GlobalHeap& heap) override;
  void update(const FrameContext& frame) override;
  void updateUI() override;

private:
  PhysicsSystem m_physicsSystem;
  IntrusivePtr<Floor> m_floor;

  struct PhysicsCapture {
    std::vector<RigidBodyState> rbStates;

    void capture(const PhysicsSystem& system);
    void restore(PhysicsSystem& system) const;
  };
  std::vector<PhysicsCapture> m_rewindBuffer;
  uint32_t m_rewindBufferOffset = 0;
  uint32_t m_rewindBufferCapacity = 32;

  PhysicsCapture m_manualCapture;
};
} // namespace 