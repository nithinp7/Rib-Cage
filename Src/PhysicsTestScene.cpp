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
  float fixedDt = 1.0f / 30.0f;
  m_physicsSystem.tick(fixedDt);
}

void PhysicsTestScene::updateUI() {
  static uint32_t unboundCapsules = 0;
  char buf[128];

  if (ImGui::CollapsingHeader("Test Physics System")) {
    ImGui::Indent();

    ImGui::Text("Gravity:");
    ImGui::DragFloat("##gravity", &m_physicsSystem.getSettings().gravity);
    ImGui::Text("Restitution:");
    ImGui::DragFloat(
        "##restitution",
        &m_physicsSystem.getSettings().restitution);
    ImGui::Text("Floor Height:");
    ImGui::DragFloat(
        "##floorheight",
        &m_physicsSystem.getSettings().floorHeight);
    ImGui::Separator();

    if (ImGui::Button("Create Capsule")) {
      float theta = 2.0f * glm::pi<float>() * std::rand() / RAND_MAX;
      float phi = glm::pi<float>() * std::rand() / RAND_MAX;
      m_physicsSystem.registerCapsuleCollider(
          glm::vec3(0.0f),
          5.0f * glm::vec3(cos(theta) * cos(phi), sin(theta), cos(theta) * sin(phi)),
          2.0f);
      ++unboundCapsules;
    }

    if (ImGui::Button("Create Rigid Body")) {
      RigidBodyHandle rb = m_physicsSystem.registerRigidBody(
          glm::vec3(0.0f),
          glm::quat(glm::vec3(0.0f, 0.0f, 0.0f)));
      for (uint32_t capsuleIdx =
               m_physicsSystem.getCapsuleCount() - unboundCapsules;
           capsuleIdx < m_physicsSystem.getCapsuleCount();
           ++capsuleIdx) {
        m_physicsSystem.bindCapsuleToRigidBody(
            ColliderHandle{capsuleIdx, ColliderType::CAPSULE},
            rb);
      }
      m_physicsSystem.bakeRigidBody(rb);
      unboundCapsules = 0;
    }

    for (uint32_t i = 0; i < m_physicsSystem.getRigidBodyCount(); ++i) {
      const RigidBody& rb = m_physicsSystem.getRigidBody(i);
      RigidBodyState& state =
          const_cast<RigidBodyState&>(m_physicsSystem.getRigidBodyState(i));

      ImGui::Separator();
      ImGui::Text("Rigid Body %d:", i);

      ImGui::Indent();

      ImGui::Text("Translation:");
      sprintf(buf, "##rbtranslate_%d", i);
      ImGui::DragFloat3(buf, &state.translation[0]);
      ImGui::Text("Rotate:");
      glm::vec3 angles = glm::degrees(glm::eulerAngles(state.rotation));
      sprintf(buf, "##rbrotate_%d", i);
      if (ImGui::DragFloat3(buf, &angles[0])) {
        state.rotation = glm::quat(glm::radians(angles));
      }

      ImGui::Unindent();
    }

    for (uint32_t i = m_physicsSystem.getCapsuleCount() - unboundCapsules;
         i < m_physicsSystem.getCapsuleCount();
         ++i) {
      Capsule& c = const_cast<Capsule&>(m_physicsSystem.getCapsule(i));
      ImGui::Separator();

      ImGui::Text("Capsule %d:", i);

      sprintf(buf, "##capsuletranslate_%d", i);
      glm::vec3 orig = 0.5f * (c.a + c.b);
      glm::vec3 center = orig;
      if (ImGui::DragFloat3(buf, &center[0])) {
        glm::vec3 diff = center - orig;
        c.a += diff;
        c.b += diff;
      }

      // sprintf(buf, "##capsulea_%d", i);
      // ImGui::DragFloat3(buf, &c.a[0]);
      // sprintf(buf, "##capsuleb_%d", i);
      // ImGui::DragFloat3(buf, &c.b[0]);
      // sprintf(buf, "##capsuler_%d", i);
      // ImGui::DragFloat(buf, &c.radius);
    }

    ImGui::Unindent();
  }
}
} // namespace RibCage