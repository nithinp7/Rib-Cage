#include "PhysicsTestScene.h"

#include <Althea/Gui.h>
#include <glm/gtc/matrix_transform.hpp>

using namespace AltheaEngine;
using namespace AltheaEngine::AltheaPhysics;

namespace RibCage {

static uint32_t s_testCapsuleIdx = 0;
static glm::vec3 s_testCapsuleA = glm::vec3(0.0f);
static glm::vec3 s_testCapsuleB = glm::vec3(5.0f);
static float s_testCapsuleRadius = 2.0f;

void PhysicsTestScene::init(
    Application& app,
    SingleTimeCommandBuffer& commandBuffer,
    SceneToGBufferPassBuilder& gBufferPassBuilder,
    GlobalHeap& heap) {
  m_physicsSystem = PhysicsSystem(app, commandBuffer, gBufferPassBuilder, heap);

  glm::vec3 a(0.0f);
  glm::vec3 b(5.0f);
  float radius = 2.0f;

  s_testCapsuleIdx = m_physicsSystem.registerCapsuleCollider(
      s_testCapsuleA,
      s_testCapsuleB,
      s_testCapsuleRadius);
}

void PhysicsTestScene::update(const FrameContext& frame) {
  m_physicsSystem.tick(frame.deltaTime);
}

void PhysicsTestScene::updateUI() {
  if (ImGui::CollapsingHeader("Test Physics System")) {
    ImGui::Indent();

    ImGui::Text("Capsule A");
    bool bChanged = ImGui::InputFloat3("##capsulea", &s_testCapsuleA[0]);

    ImGui::Text("Capsule B");
    bChanged |= ImGui::InputFloat3("##capsuleb", &s_testCapsuleB[0]);

    ImGui::Text("Capsule Radius");
    bChanged |= ImGui::InputFloat("##capsuleradius", &s_testCapsuleRadius);

    if (bChanged)
      m_physicsSystem.updateCapsule(
          s_testCapsuleIdx,
          s_testCapsuleA,
          s_testCapsuleB,
          s_testCapsuleRadius);

    ImGui::Unindent();
  }
}
} // namespace RibCage