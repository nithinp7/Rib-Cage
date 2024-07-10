#include "PhysicsTestScene.h"

#include <Althea/Gui.h>
#include <glm/gtc/matrix_transform.hpp>

using namespace AltheaEngine;
using namespace AltheaEngine::AltheaPhysics;

namespace RibCage {

static bool s_paused = false;
static int32_t s_stepFrameCounter = 0;

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
  float fixedDt = 1.0f / 60.0f;
  if (!s_paused || s_stepFrameCounter > 0)
    m_physicsSystem.tick(fixedDt);
  if (s_stepFrameCounter > 0)
    --s_stepFrameCounter;
}

void PhysicsTestScene::updateUI() {
  static uint32_t unboundCapsules = 0;
  char buf[128];

  if (ImGui::CollapsingHeader("Test Physics System")) {
    ImGui::Indent();

    ImGui::Text("Pause:");
    ImGui::SameLine();
    ImGui::Checkbox("##pause", &s_paused);

    ImGui::Text("Step:");
    ImGui::SameLine();
    if (ImGui::Button(">"))
      s_stepFrameCounter = 1;
    ImGui::SameLine();
    if (ImGui::Button(">>"))
      s_stepFrameCounter = 8;
    ImGui::SameLine();
    if (ImGui::Button(">>>"))
      s_stepFrameCounter = 16;

    static bool s_settingsVisible = false;
    if (ImGui::CollapsingHeader("Physics World Settings", s_settingsVisible)) {
      ImGui::Text("Iterations:");
      ImGui::DragInt(
          "##iterations",
          &m_physicsSystem.getSettings().SI_iters,
          1.0F,
          1,
          40);
      ImGui::Text("Gravity:");
      ImGui::DragFloat(
          "##gravity",
          &m_physicsSystem.getSettings().gravity,
          1.0f,
          -20.0f,
          20.0f);
      ImGui::Text("Friction:");
      ImGui::DragFloat(
          "##friction",
          &m_physicsSystem.getSettings().frictionCoeff,
          1.0f,
          0.0f,
          1.0f);
      ImGui::Text("Linear Damping:");
      ImGui::DragFloat(
          "##lindamping",
          &m_physicsSystem.getSettings().linearDamping,
          0.01f,
          0.0f,
          1.0f);
      ImGui::Text("Angular Damping:");
      ImGui::DragFloat(
          "##angdamping",
          &m_physicsSystem.getSettings().angularDamping,
          0.01f,
          0.0f,
          1.0f);
      ImGui::Text("Bias:");
      ImGui::DragFloat(
          "##sibias",
          &m_physicsSystem.getSettings().SI_bias,
          1.0f,
          0.0f,
          1.0f);
      ImGui::Text("Restitution:");
      ImGui::DragFloat(
          "##restitution",
          &m_physicsSystem.getSettings().restitution,
          1.0f,
          0.0f,
          1.0f);
      ImGui::Text("Floor Height:");
      ImGui::DragFloat(
          "##floorheight",
          &m_physicsSystem.getSettings().floorHeight,
          1.0f,
          -20.0f,
          20.0f);
    }
    ImGui::Separator();

    if (ImGui::Button("Create Capsule")) {
      float theta = 2.0f * glm::pi<float>() * std::rand() / RAND_MAX;
      float phi = glm::pi<float>() * std::rand() / RAND_MAX;
      m_physicsSystem.registerCapsuleCollider(
          glm::vec3(0.0f),
          5.0f * glm::vec3(
                     cos(theta) * cos(phi),
                     sin(theta),
                     cos(theta) * sin(phi)),
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

      glm::vec3 orig = 0.5f * (c.a + c.b);
      glm::vec3 center = orig;
      sprintf(buf, "##capsuletranslate_%d", i);
      if (ImGui::DragFloat3(buf, &center[0])) {
        glm::vec3 diff = center - orig;
        c.a += diff;
        c.b += diff;
      }

      glm::vec3 r = c.b - c.a;
      float anglesScale[3];
      anglesScale[0] = glm::degrees(atan2(r.y, sqrt(r.x * r.x + r.z * r.z)));
      anglesScale[1] = glm::degrees(atan2(r.z, r.x));
      anglesScale[2] = glm::length(r);
      sprintf(buf, "##capsulerot_%d", i);
      if (ImGui::DragFloat3(buf, anglesScale)) {
        float sTheta = sin(glm::radians(anglesScale[0]));
        float cTheta = cos(glm::radians(anglesScale[0]));
        float sPhi = sin(glm::radians(anglesScale[1]));
        float cPhi = cos(glm::radians(anglesScale[1]));

        c.b = c.a +
              anglesScale[2] * glm::vec3(cTheta * cPhi, sTheta, cTheta * sPhi);
      }
    }

    ImGui::Unindent();
  }
}
} // namespace RibCage