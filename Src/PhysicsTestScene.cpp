#include "PhysicsTestScene.h"

#include <Althea/Gui.h>
#include <Althea/InputManager.h>
#include <Althea/InputMask.h>
#include <Althea/Utilities.h>
#include <Althea/StencilQuery.h>
#include <glm/gtc/matrix_transform.hpp>

using namespace AltheaEngine;
using namespace AltheaEngine::AltheaPhysics;

namespace RibCage {

static bool s_paused = false;
static int32_t s_stepFrameCounter = 0;
static bool s_activeStepping = false;

void PhysicsTestScene::init(
    Application& app,
    SingleTimeCommandBuffer& commandBuffer,
    SceneToGBufferPassBuilder& gBufferPassBuilder,
    GlobalHeap& heap) {
  float floorHeight = -80.0f;

  app.getInputManager().addKeyBinding({GLFW_KEY_P, GLFW_PRESS, 0}, []() {
    s_paused = !s_paused;
  });
  app.getInputManager().addKeyBinding({GLFW_KEY_RIGHT, GLFW_PRESS, 0}, []() {
    s_activeStepping = true;
  });
  app.getInputManager().addKeyBinding(
      {GLFW_KEY_RIGHT, GLFW_PRESS, GLFW_MOD_CONTROL},
      []() { s_stepFrameCounter++; });
  app.getInputManager().addKeyBinding({GLFW_KEY_RIGHT, GLFW_RELEASE, 0}, []() {
    s_activeStepping = false;
  });
  app.getInputManager().addKeyBinding({GLFW_KEY_LEFT, GLFW_PRESS, 0}, [this]() {
    m_rewindBufferOffset =
        m_rewindBufferOffset > 1 ? m_rewindBufferOffset - 2 : 0;
    m_rewindBuffer[m_rewindBufferOffset++ % m_rewindBufferCapacity].restore(
        m_physicsSystem);
  });
  app.getInputManager().addKeyBinding({GLFW_KEY_DOWN, GLFW_PRESS, 0}, [this]() {
    m_manualCapture.capture(m_physicsSystem);
  });
  app.getInputManager().addKeyBinding({GLFW_KEY_UP, GLFW_PRESS, 0}, [this]() {
    m_manualCapture.restore(m_physicsSystem);
  });
  app.getInputManager().addKeyBinding(
      {GLFW_KEY_W, GLFW_PRESS, GLFW_MOD_CONTROL},
      [this]() {
        m_physicsSystem.getSettings().wireframeCapsules =
            !m_physicsSystem.getSettings().wireframeCapsules;
        m_physicsSystem.forceDebugDraw(1.0f / 60.0f);
      });

  m_physicsSystem = PhysicsSystem(app, commandBuffer, gBufferPassBuilder, heap);
  m_physicsSystem.getSettings().floorHeight = floorHeight;

  m_floor = makeIntrusive<Floor>();
  gBufferPassBuilder.registerSubpass(m_floor);
  m_floor->m_floorHeight = floorHeight;
  m_floor->m_floorHalfWidth = 500.0f;

  m_rewindBuffer.resize(m_rewindBufferCapacity);
}

void PhysicsTestScene::PhysicsCapture::capture(const PhysicsSystem& system) {
  rbStates.resize(system.getRigidBodyCount());
  for (uint32_t i = 0; i < system.getRigidBodyCount(); ++i) {
    rbStates[i] = system.getRigidBodyState(i);
  }
}

void PhysicsTestScene::PhysicsCapture::restore(PhysicsSystem& system) const {
  for (uint32_t i = 0; i < system.getRigidBodyCount() && i < rbStates.size();
       ++i) {
    const_cast<RigidBodyState&>(system.getRigidBodyState(i)) = rbStates[i];
  }

  system.forceUpdateCapsules();
  system.forceDebugDraw(1.0f / 60.0f);
}

void PhysicsTestScene::update(const FrameContext& frame) {
  float fixedDt = 1.0f / 60.0f;
  if (!s_paused || s_stepFrameCounter > 0 || s_activeStepping) {
    m_physicsSystem.tick(fixedDt);

    // pause simulation if we hit nan positions
    for (uint32_t i = 0; i < m_physicsSystem.getRigidBodyCount(); ++i) {
      const glm::vec3& pos = m_physicsSystem.getRigidBodyState(i).translation;
      if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z)) {
        s_paused = true;
        break;
      }
    }

    static uint32_t s_frameCounter = 0;

    // add a capture to the rewind buffer every second
    if (s_frameCounter % 60 == 0) {
      ++m_rewindBufferOffset;
      m_rewindBuffer[m_rewindBufferOffset % m_rewindBufferCapacity].capture(
          m_physicsSystem);
    }

    s_frameCounter++;
  }

  if (s_stepFrameCounter > 0)
    --s_stepFrameCounter;
}

extern Application* GApplication;
extern StencilQueryManager* GStencilQueryManager;

void PhysicsTestScene::updateUI() {
  
  static uint32_t unboundCapsules = 0;
  char buf[128];

  if (ImGui::CollapsingHeader("Test Physics System")) {
    ImGui::Indent();

    const VkExtent2D& extent = GApplication->getSwapChainExtent();
    InputManager::MousePos mpos = GInputManager->getCurrentMousePos();
    glm::vec2 mouseUV(static_cast<float>(mpos.x) / extent.width, static_cast<float>(mpos.y) / extent.height);
    
    static StencilQueryHandle query = GStencilQueryManager->createStencilQuery(mouseUV);

    static uint32_t queryResult = 0;
    if (GStencilQueryManager->getQueryResults(query, queryResult)) {
      query = GStencilQueryManager->createStencilQuery(mouseUV);
    }
    ImGui::Text("Stencil Query Result %u", queryResult);

    static char s_filename[256] = "Data/PhysicsUnitTests/Test.simsave";
    if (ImGui::Button("Save Simulation")) {
      std::string filename =
        GProjectDirectory + "/" + std::string(s_filename);
      m_physicsSystem.debugSaveToFile(filename.c_str());
    }

    if (ImGui::Button("Load Simulation")) {
      std::string filename =
        GProjectDirectory + "/" + std::string(s_filename);
      m_physicsSystem.debugLoadFromFile(filename.c_str());
      m_manualCapture.capture(m_physicsSystem);
    }

    ImGui::InputText("##simulationfilename", s_filename, 256);

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

    // TODO: Fix-up debug line rendering, it doesn't work with rewinding
    /* if (ImGui::Button("Rewind")) {
       m_rewindBufferOffset = (m_rewindBufferOffset + m_rewindBufferCapacity -
     1) % m_rewindBufferCapacity;
       m_rewindBuffer[m_rewindBufferOffset].restore(m_physicsSystem);
     }*/

    if (ImGui::Button("Manual Capture")) {
      m_manualCapture.capture(m_physicsSystem);
    }

    if (ImGui::Button("Restore Capture")) {
      m_manualCapture.restore(m_physicsSystem);
    }

    static bool s_settingsVisible = false;
    if (ImGui::CollapsingHeader("Physics World Settings", s_settingsVisible)) {
      ImGui::Text("Iterations:");
      ImGui::DragInt(
          "##iterations",
          &m_physicsSystem.getSettings().timeSubsteps,
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
      ImGui::Text("Static Friction:");
      ImGui::DragFloat(
          "##stfriction",
          &m_physicsSystem.getSettings().staticFriction,
          1.0f,
          0.0f,
          20.0f);
      ImGui::Text("Dynamic Friction:");
      ImGui::DragFloat(
          "##dynfriction",
          &m_physicsSystem.getSettings().dynamicFriction,
          1.0f,
          0.0f,
          20.0f);
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
      ImGui::Text("Restitution:");
      ImGui::DragFloat(
          "##restitution",
          &m_physicsSystem.getSettings().restitution,
          1.0f,
          0.0f,
          1.0f);
      ImGui::Text("Floor Height:");
      if (ImGui::DragFloat(
              "##floorheight",
              &m_physicsSystem.getSettings().floorHeight,
              1.0f,
              -80.0f,
              20.0f)) {
        m_floor->m_floorHeight = m_physicsSystem.getSettings().floorHeight;
      }

      ImGui::Separator();
      ImGui::Text("Enable Velocity Update:");
      ImGui::Checkbox(
          "##enablevelupdate",
          &m_physicsSystem.getSettings().enableVelocityUpdate);
      if (!m_physicsSystem.getSettings().enableVelocityUpdate) {
        ImGui::Text("Override Angular Velocity:");
        static glm::vec3 angVel(0.0f);
        if (ImGui::DragFloat3("##angularvel", &angVel[0])) {
          for (uint32_t i = 0; i < m_physicsSystem.getRigidBodyCount(); ++i) {
            const_cast<glm::vec3&>(
                m_physicsSystem.getRigidBodyState(i).angularVelocity) = angVel;
            const_cast<glm::vec3&>(
                m_physicsSystem.getRigidBodyState(i).linearVelocity) =
                glm::vec3(0.0f);
          }
        }
      }
    }
    ImGui::Separator();

    if (ImGui::Button("Create Chain")) {
      for (uint32_t i = 0; i < 10; ++i) {
        float theta = 2.0f * glm::pi<float>() * Utilities::randf();
        float phi = glm::pi<float>() * Utilities::randf();
        float length = 15.0f * Utilities::randf() + 5.0f;
        float radius = 2.0f; // *Utilities::randf() + 2.0f;
        m_physicsSystem.registerCapsuleCollider(
            glm::vec3(0.0f),
            length * glm::vec3(
                         cos(theta) * cos(phi),
                         sin(theta),
                         cos(theta) * sin(phi)),
            radius);
        ++unboundCapsules;
      }
    }

    if (ImGui::Button("Push Apart")) {

      // initial collision iters
      for (uint32_t iter = 0; iter < 10; ++iter) {
        for (uint32_t i = m_physicsSystem.getCapsuleCount() - unboundCapsules;
             i < m_physicsSystem.getCapsuleCount();
             ++i) {
          for (uint32_t j = m_physicsSystem.getCapsuleCount() - unboundCapsules;
               j < m_physicsSystem.getCapsuleCount();
               ++j) {
            if (i == j)
              continue;

            Capsule& ca = const_cast<Capsule&>(m_physicsSystem.getCapsule(i));
            Capsule& cb = const_cast<Capsule&>(m_physicsSystem.getCapsule(j));

            CollisionResult result{};
            if (Collisions::checkIntersection(ca, cb, result)) {
              glm::vec3 diff = result.rb - result.ra;
              float C = glm::dot(result.n, diff);
              if (C < 0.0f) {
                ca.a += 0.5f * diff;
                ca.b += 0.5f * diff;

                cb.a -= 0.5f * diff;
                cb.b -= 0.5f * diff;
              }
            }
          }
        }
      }
    }

    if (ImGui::Button("Create Capsule")) {
      float theta = 2.0f * glm::pi<float>() * Utilities::randf();
      float phi = glm::pi<float>() * Utilities::randf();
      float length = 15.0f * Utilities::randf() + 5.0f;
      float radius = 2.0f; // *Utilities::randf() + 2.0f;
      m_physicsSystem.registerCapsuleCollider(
          glm::vec3(0.0f),
          length * glm::vec3(
                       cos(theta) * cos(phi),
                       sin(theta),
                       cos(theta) * sin(phi)),
          radius);
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
      m_manualCapture.capture(m_physicsSystem);
      unboundCapsules = 0;
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

    if (ImGui::CollapsingHeader("Active Rigid Bodies")) {
      for (uint32_t i = 0; i < m_physicsSystem.getRigidBodyCount(); ++i) {
        const RigidBody& rb = m_physicsSystem.getRigidBody(i);
        RigidBodyState& state =
            const_cast<RigidBodyState&>(m_physicsSystem.getRigidBodyState(i));

        ImGui::Separator();
        sprintf(buf, "Rigid Body %d", i);
        if (ImGui::CollapsingHeader(buf)) {
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

          // ImGui::Text("Linear Velocity");
          // sprintf(buf, "##linvel_%d", i);
          // ImGui::DragFloat3(buf, &state.linearVelocity[0]);

          // ImGui::Text("Angular Velocity");
          // sprintf(buf, "##angvel_%d", i);
          // ImGui::DragFloat3(buf, &state.angularVelocity[0]);

          ImGui::Unindent();
        }
      }
    }

    ImGui::Unindent();
  }
}
} // namespace RibCage