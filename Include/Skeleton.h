#pragma once

#include "DebugTools.h"

#include <Althea/Application.h>
#include <Althea/Framebuffer.h>
#include <Althea/GlobalHeap.h>
#include <Althea/GlobalResources.h>
#include <Althea/IndexBuffer.h>
#include <Althea/RenderPass.h>
#include <Althea/TransientUniforms.h>
#include <Althea/VertexBuffer.h>
#include <glm/glm.hpp>
#include <vulkan/vulkan.h>

#include <cstdint>
#include <string>
#include <vector>

using namespace AltheaEngine;

namespace RibCage {

#define INVALID_BONE_IDX 0xFF
#define INVALID_JOINT_IDX 0xFF
#define MAX_BONE_COUNT 32
#define MAX_JOINT_COUNT 32

struct JointChildren {
  uint8_t children[4] = {
      INVALID_JOINT_IDX,
      INVALID_JOINT_IDX,
      INVALID_JOINT_IDX,
      INVALID_JOINT_IDX};
};

struct IkHandle {
  glm::vec3 target; 
  uint8_t jointIdx;
};

struct SkeletonState {
  glm::vec3 prevPositions[MAX_JOINT_COUNT];
  glm::vec3 positions[MAX_JOINT_COUNT];
};

struct Skeleton {
  glm::mat4 localTransforms[MAX_JOINT_COUNT];
  glm::mat4 worldTransforms[MAX_JOINT_COUNT];
  JointChildren jointChildren[MAX_JOINT_COUNT];
  uint8_t jointCount = 0;
  uint8_t rootJoint = 0;
  uint8_t padding1;
  uint8_t padding2;

  void recomputeWorldTransforms(const glm::mat4& parentTransform, uint8_t idx);
  void recomputeWorldTransforms();

  void
  recomputeLocalTransforms(const glm::mat4& invParentTransform, uint8_t idx);
  void recomputeLocalTransforms();
  void solveIk(const std::vector<IkHandle>& ikHandles, uint8_t idx);
  void solveIk(const std::vector<IkHandle>& ikHandles);
  void initState(SkeletonState& state);
  void simStep(float dt, SkeletonState& state);
};

class SkeletonLoader {
public:
  static bool load(const std::string& path, Skeleton& result);
  static bool save(const std::string& path, const Skeleton& skeleton);

private:
};

class SkeletonEditor {
public:
  SkeletonEditor() = default;
  SkeletonEditor(
      Application& app,
      VkCommandBuffer commandBuffer,
      GlobalHeap& heap,
      const GBufferResources& gBuffer);

  void updateUI();
  void update(
      SelectableScene& scene,
      const glm::vec3& cameraPos,
      const glm::vec3& cursorDir,
      uint32_t prevInputMask,
      uint32_t inputMask);

  void draw(
      const Application& app,
      VkCommandBuffer commandBuffer,
      const FrameContext& frame,
      VkDescriptorSet heapSet,
      UniformHandle globalUniformsHandle);

  void tryRecompileShaders(Application& app) {
    m_skeletonPass.tryRecompile(app);
  }

private:
  void _updateSkeletonCreator(
      SelectableScene& scene,
      const glm::vec3& cameraPos,
      const glm::vec3& cursorDir,
      uint32_t prevInputMask,
      uint32_t inputMask);

  // Bind-pose skeleton
  Skeleton m_skeleton{};

  int m_gizmoAxis{};
  bool m_bDraggingGizmo{};
  glm::vec3 m_originalGizmoPosition{};

  // Constraints for spawning new vertices
  glm::vec3 m_constraintPlane;
  glm::vec3 m_constraintPoint;

  RenderPass m_skeletonPass;
  FrameBuffer m_frameBuffer;
  TransientUniforms<Skeleton> m_skeletonUniforms;
  VertexBuffer<glm::vec3> m_cylinderVB;
  IndexBuffer m_cylinderIB;

  SkeletonState m_simState;
  bool m_bSimulating = false;
};
} // namespace RibCage