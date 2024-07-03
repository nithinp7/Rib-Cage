#include "PhysicsTestScene.h"

#include <Althea/Gui.h>
#include <glm/gtc/matrix_transform.hpp>

using namespace AltheaEngine;
using namespace AltheaEngine::AltheaPhysics;

namespace RibCage {

void PhysicsTestScene::init(
    Application& app,
    SingleTimeCommandBuffer& commandBuffer,
    SceneToGBufferPassBuilder& gBufferPassBuilder,
    GlobalHeap& heap) {
  m_physicsSystem = PhysicsSystem(app, commandBuffer, gBufferPassBuilder, heap);

  glm::vec3 a(0.0f);
  glm::vec3 b(5.0f);
  float radius = 2.0f;
}

void PhysicsTestScene::update(const FrameContext& frame) {
  m_physicsSystem.tick(frame.deltaTime);
}

void PhysicsTestScene::updateUI() {
  if (ImGui::CollapsingHeader("Test Physics System")) {
    ImGui::Indent();

    if (ImGui::Button("Create Capsule")) {
      m_physicsSystem.registerCapsuleCollider(glm::vec3(0.0f), glm::vec3(5.0f), 2.0f);
    }

    for (uint32_t i = 0; i < m_physicsSystem.getCapsuleCount(); ++i) {
      Capsule& c = const_cast<Capsule&>(m_physicsSystem.getCapsule(i));
      ImGui::Separator();

      ImGui::Text("Capsule %d:", i);

      char buf[128];
      sprintf(buf, "##capsulea_%d", i);
      ImGui::DragFloat3(buf, &c.a[0]);
      sprintf(buf, "##capsuleb_%d", i);
      ImGui::DragFloat3(buf, &c.b[0]);
      sprintf(buf, "##capsuler_%d", i);
      ImGui::DragFloat(buf, &c.radius);
    }

    ImGui::Unindent();
  }
}
} // namespace RibCage