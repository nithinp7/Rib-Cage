#include "Skeleton.h"

#include <Althea/Gui.h>
#include <Althea/InputMask.h>
#include <Althea/ShapeUtilities.h>
#include <Althea/Utilities.h>
#include <glm/gtc/matrix_inverse.hpp>

#include <memory>

using namespace AltheaEngine;

namespace RibCage {

static int s_editorMode = 0;

void Skeleton::recomputeWorldTransforms(
    const glm::mat4& parentTransform,
    uint8_t idx) {
  assert(idx < jointCount && idx != INVALID_JOINT_IDX);
  worldTransforms[idx] = parentTransform * localTransforms[idx];
  for (uint8_t childIdx : jointChildren[idx].children) {
    if (childIdx != INVALID_JOINT_IDX) {
      recomputeWorldTransforms(worldTransforms[idx], childIdx);
    }
  }
}

void Skeleton::recomputeWorldTransforms() {
  if (jointCount > 0)
    recomputeWorldTransforms(glm::mat4(1.0f), rootJoint);
}

void Skeleton::recomputeLocalTransforms(
    const glm::mat4& invParentTransform,
    uint8_t idx) {
  assert(idx < jointCount && idx != INVALID_JOINT_IDX);
  localTransforms[idx] = invParentTransform * worldTransforms[idx];
  glm::mat4 invWorldTransform = glm::affineInverse(worldTransforms[idx]);
  for (uint8_t childIdx : jointChildren[idx].children) {
    if (childIdx != INVALID_JOINT_IDX) {
      recomputeLocalTransforms(invWorldTransform, childIdx);
    }
  }
}

void Skeleton::recomputeLocalTransforms() {
  if (jointCount > 0)
    recomputeLocalTransforms(glm::mat4(1.0f), rootJoint);
}

void Skeleton::solveIk(const std::vector<IkHandle>& ikHandles, uint8_t idx) {
  glm::vec3 jointPos(worldTransforms[idx][3]);
  for (uint8_t childIdx : jointChildren[idx].children) {
    if (childIdx == INVALID_JOINT_IDX)
      break;

    for (const auto& handle : ikHandles) {
      if (handle.jointIdx == childIdx) {
        // Child has an ik handle
        glm::vec3 childPos(worldTransforms[childIdx][3]);
        float radius = glm::length(glm::vec3(localTransforms[childIdx][3]));

        break;
      }
    }
  }
}

void Skeleton::solveIk(const std::vector<IkHandle>& ikHandles) {
  assert(jointCount > 0);
}

/*static*/
bool SkeletonLoader::load(const std::string& path, Skeleton& result) {
  if (!Utilities::checkFileExists(path))
    return false;

  std::vector<char> data = Utilities::readFile(path);
  if (data.size() != sizeof(Skeleton))
    return false;

  memcpy(&result, data.data(), sizeof(Skeleton));
  return true;
}

/*static*/
bool SkeletonLoader::save(const std::string& path, const Skeleton& skeleton) {
  return Utilities::writeFile(
      path,
      gsl::span((const char*)&skeleton, sizeof(skeleton)));
}

namespace {
struct PushConstants {
  uint32_t globalUniformsHandle;
  uint32_t skeletonUniformsHandle;
};
}; // namespace

SkeletonEditor::SkeletonEditor(
    Application& app,
    VkCommandBuffer commandBuffer,
    GlobalHeap& heap,
    const GBufferResources& gBuffer) {
  ShapeUtilities::createCylinder(
      app,
      commandBuffer,
      m_cylinderVB,
      m_cylinderIB,
      16);

  m_skeletonUniforms = TransientUniforms<Skeleton>(app);
  m_skeletonUniforms.registerToHeap(heap);

  std::vector<SubpassBuilder> builders;
  {
    SubpassBuilder& builder = builders.emplace_back();

    // The GBuffer contains the following color attachments
    // 1. Position
    // 2. Normal
    // 3. Albedo
    // 4. Metallic-Roughness-Occlusion
    builder.colorAttachments = {0, 1, 2, 3};
    builder.depthAttachment = 4;

    builder.pipelineBuilder
        .addVertexInputBinding<glm::vec3>(VK_VERTEX_INPUT_RATE_VERTEX)
        .addVertexAttribute(VertexAttributeType::VEC3, 0)

        .addVertexShader(GProjectDirectory + "/Shaders/Bone.vert")
        .addFragmentShader(
            GProjectDirectory + "/Shaders/GBufferPassThrough.frag")

        .layoutBuilder //
        .addDescriptorSet(heap.getDescriptorSetLayout())
        .addPushConstants<PushConstants>(VK_SHADER_STAGE_ALL);
  }

  std::vector<Attachment> attachments = gBuffer.getAttachmentDescriptions();
  for (auto& attachment : attachments)
    attachment.load = true;

  m_skeletonPass = RenderPass(
      app,
      app.getSwapChainExtent(),
      std::move(attachments),
      std::move(builders));
  m_frameBuffer = FrameBuffer(
      app,
      m_skeletonPass,
      app.getSwapChainExtent(),
      gBuffer.getAttachmentViews());
}

void SkeletonEditor::updateUI() {
  if (ImGui::CollapsingHeader(
          "Skeleton Editor",
          ImGuiTreeNodeFlags_DefaultOpen)) {
    static const char* editorModes[] = {"Skeleton Creator", "IK Creator"};
    ImGui::Text("Skeleton Editor Mode:");
    ImGui::Combo("##skeletonEditorMode", &s_editorMode, editorModes, 2);
    ImGui::Separator();

    if (s_editorMode == 0) {
      static char s_filename[256] = "SkeletonData/Test.skl";
      if (ImGui::Button("Save Skeleton")) {
        SkeletonLoader::save(
            GProjectDirectory + "/" + std::string(s_filename),
            m_skeleton);
      }

      if (ImGui::Button("Load Skeleton")) {
        SkeletonLoader::load(
            GProjectDirectory + "/" + std::string(s_filename),
            m_skeleton);
      }

      ImGui::InputText("##skeletonFilename", s_filename, 256);
    }
  }
}

void SkeletonEditor::update(
    SelectableScene& scene,
    const glm::vec3& cameraPos,
    const glm::vec3& cursorDir,
    uint32_t prevInputMask,
    uint32_t inputMask) {
  if (s_editorMode == 0)
    _updateSkeletonCreator(
        scene,
        cameraPos,
        cursorDir,
        prevInputMask,
        inputMask);
}

void SkeletonEditor::draw(
    const Application& app,
    VkCommandBuffer commandBuffer,
    const FrameContext& frame,
    VkDescriptorSet heapSet,
    UniformHandle globalUniformsHandle) {
  m_skeletonUniforms.updateUniforms(m_skeleton, frame);

  ActiveRenderPass pass =
      m_skeletonPass.begin(app, commandBuffer, frame, m_frameBuffer);

  pass.setGlobalDescriptorSets(gsl::span(&heapSet, 1));

  const DrawContext& context = pass.getDrawContext();
  if (m_skeleton.jointCount > 0) {
    PushConstants constants{};
    constants.globalUniformsHandle = globalUniformsHandle.index;
    constants.skeletonUniformsHandle =
        m_skeletonUniforms.getCurrentHandle(frame).index;
    context.updatePushConstants(constants, 0);
    context.bindDescriptorSets();

    context.bindVertexBuffer(m_cylinderVB);
    context.bindIndexBuffer(m_cylinderIB);
    context.drawIndexed(m_cylinderIB.getIndexCount(), m_skeleton.jointCount);
  }
}

void SkeletonEditor::_updateSkeletonCreator(
    SelectableScene& scene,
    const glm::vec3& cameraPos,
    const glm::vec3& cursorDir,
    uint32_t prevInputMask,
    uint32_t inputMask) {
  bool bLastPressed = prevInputMask & INPUT_BIT_LEFT_MOUSE;
  bool bPressed = inputMask & INPUT_BIT_LEFT_MOUSE;

  bool bNewPress = !bLastPressed && bPressed;
  bool bHeldDown = bLastPressed && bPressed;
  bool bRelease = bLastPressed && !bPressed;

  bool bAlt = inputMask & INPUT_BIT_ALT;
  bool bCtrl = inputMask & INPUT_BIT_CTRL;

  // Is there a smarter way to spawn new points in a useful place?
  m_constraintPlane = glm::vec3(1.0f, 0.0f, 0.0f);
  m_constraintPoint = glm::vec3(0.0f, 0.0f, 0.0f);

  if (bNewPress) {
    SceneQueryResult query = scene.query(cameraPos, cursorDir);

    if (bAlt) {
      // Alt-click spawns new vertex, the last selected vertex is used
      // as the parent
      if (m_skeleton.jointCount == MAX_JOINT_COUNT)
        return;

      // Intersect the camera ray with the constraint plane to compute
      // a spawn position
      float distToPlane =
          glm::dot(m_constraintPoint - cameraPos, m_constraintPlane);
      float dxdt = glm::dot(m_constraintPlane, cursorDir);
      if (abs(dxdt) < 0.01)
        return;

      int lastSelected = scene.getLastSelected();

      float t = distToPlane / dxdt;
      glm::vec3 spawnPos = cameraPos + cursorDir * t;
      int newVertex = scene.addVertex(spawnPos);
      scene.selectVertex(newVertex);
      scene.enableGizmo();
      scene.setGizmoPosition(spawnPos);

      uint8_t jointIdx = m_skeleton.jointCount;
      m_skeleton.worldTransforms[jointIdx] = glm::mat4(1.0f);
      m_skeleton.worldTransforms[jointIdx][3] = glm::vec4(spawnPos, 1.0f);
      // if (query.hitType == QHT_VERTEX)
      {
        if (m_skeleton.jointCount == 0) {
          // The newly added vertex will be the root joint
          m_skeleton.rootJoint = 0;
          m_skeleton.jointCount = 1;
          m_skeleton.localTransforms[0] = m_skeleton.worldTransforms[0];
        } else {
          if (lastSelected >= 0) {
            uint8_t parentIdx = static_cast<uint8_t>(lastSelected);
            bool bFoundSlotForChild = false;
            for (uint8_t& child :
                 m_skeleton.jointChildren[parentIdx].children) {
              if (child == INVALID_JOINT_IDX) {
                child = m_skeleton.jointCount;
                bFoundSlotForChild = true;
                break;
              }
            }

            assert(bFoundSlotForChild);
          }
          ++m_skeleton.jointCount;
          m_skeleton.recomputeLocalTransforms();
        }
      }

    } else if (bCtrl) {
      // Ctrl-click adds to selection
      if (query.hitType == QHT_NONE) {
        scene.clearSelection();
        scene.disableGizmo();
      } else if (query.hitType == QHT_VERTEX) {
        scene.addToSelection(query.index);
        scene.enableGizmo();
        glm::vec3 gizmoPos(0.0f);
        for (int idx : scene.getSelection())
          gizmoPos += scene.getVertex(idx).position;
        gizmoPos /= scene.getSelection().size();
        scene.setGizmoPosition(gizmoPos);
      }
    } else {
      // Regular click replaces the current selection
      if (query.hitType == QHT_NONE) {
        scene.clearSelection();
        scene.disableGizmo();
      } else if (query.hitType == QHT_VERTEX) {
        scene.selectVertex(query.index);
        scene.enableGizmo();
        scene.setGizmoPosition(scene.getVertex(query.index).position);
      }
    }

    // Any type of click on a gizmo starts a drag
    if (query.hitType == QHT_GIZMO_HANDLE) {
      m_bDraggingGizmo = true;
      m_gizmoAxis = query.index;
      m_originalGizmoPosition = scene.getGizmoVertexPosition(m_gizmoAxis);
    }
  } else if (bHeldDown) {
    if (m_bDraggingGizmo) {
      // Update the gizmo position if this is a gizmo drag
      const glm::vec3& gizmoPos = scene.getGizmoVertexPosition(m_gizmoAxis);
      glm::vec3 axis(
          (float)(m_gizmoAxis == 0),
          (float)(m_gizmoAxis == 1),
          (float)(m_gizmoAxis == 2));
      glm::vec3 n = glm::normalize(glm::cross(axis, cursorDir));
      glm::vec3 c = cameraPos - gizmoPos;

      // can simplify dot with axis obviously
      float v0v1 = glm::dot(cursorDir, axis);
      float x0v0 = glm::dot(c, cursorDir);
      float x0v1 = glm::dot(c, axis);
      float denom = 1.0f - v0v1 * v0v1;
      if (denom > 0.001f) {
        // What is the interpretation of denom < EPS? Parallel lines?

        float s = (x0v1 * v0v1 - x0v0) / denom;
        float t = x0v1 + s * v0v1;

        glm::vec3 translation = t * axis;
        glm::vec3 newGizmoPos = gizmoPos + translation;
        scene.setGizmoVertexPosition(newGizmoPos, m_gizmoAxis);
        const std::vector<int>& selection = scene.getSelection();
        if (selection.size() == 1 &&
            (scene.getVertexRef(selection[0]).infoMask &
             SelectionInfoMaskBits::IK_HANDLE)) {
          // Solve from IK handle
          int idx = selection[0];
          auto& v = scene.getVertexRef(idx);
          v.position += translation;
          m_skeleton.worldTransforms[idx][3] += glm::vec4(translation, 0.0f);
          // m_skeleton.recomputeFromIkHandle(idx);
        } else {
          for (int idx : scene.getSelection()) {
            scene.getVertexRef(idx).position += translation;
            m_skeleton.worldTransforms[idx][3] += glm::vec4(translation, 0.0f);
          }
          m_skeleton.recomputeLocalTransforms();
        }
      } else {
        // Degenerate case, stop dragging to avoid glitches
        m_bDraggingGizmo = false;
      }
    }
  } else if (bRelease) {
    if (m_bDraggingGizmo) {
      m_bDraggingGizmo = false;
    }
  }

  if (inputMask & INPUT_BIT_K) {
    // Mark currently selected vertices as ik handles
    for (int idx : scene.getSelection())
      scene.getVertexRef(idx).infoMask |= SelectionInfoMaskBits::IK_HANDLE;
  }
}
} // namespace RibCage