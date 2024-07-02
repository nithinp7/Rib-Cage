#pragma once

#include <Althea/Application.h>
#include <Althea/BindlessHandle.h>
#include <Althea/DrawContext.h>
#include <Althea/DynamicVertexBuffer.h>
#include <Althea/Framebuffer.h>
#include <Althea/GlobalHeap.h>
#include <Althea/GlobalResources.h>
#include <Althea/IndexBuffer.h>
#include <Althea/RenderPass.h>
#include <Althea/VertexBuffer.h>
#include <glm/glm.hpp>
#include <vulkan/vulkan.h>

#include <vector>

#define MAX_SELECTABLE_VERTS_COUNT 1024
#define MAX_DBG_LINES 2048

using namespace AltheaEngine;

#define BIT(x) (1 << x)
namespace RibCage {
enum SelectionInfoMaskBits : uint32_t { SELECTED = BIT(0), IK_HANDLE = BIT(1) };

struct SelectableVertex {
  alignas(16) glm::vec3 position;
  alignas(4) uint32_t infoMask;

  bool intersect(const glm::vec3& orig, const glm::vec3& dir, float& t) const;
};

enum QueryHitType : uint8_t { QHT_NONE = 0, QHT_VERTEX, QHT_GIZMO_HANDLE };

struct SceneQueryResult {
  float t;
  int index; // vertex index or gizmo axis index
  QueryHitType hitType;
};

// Debug scene elements that are selectable
class SelectableScene {
public:
  SelectableScene() = default;
  SelectableScene(
      Application& app,
      GlobalHeap& heap,
      VkCommandBuffer commandBuffer,
      const GBufferResources& gBuffer);

  void clear() { m_currentVertCount = 0; }
  
  int addVertex(const glm::vec3& pos) {
    if (m_currentVertCount == MAX_SELECTABLE_VERTS_COUNT)
      return -1;

    SelectableVertex& v = m_selectableVertices[m_currentVertCount];
    v.position = pos;
    v.infoMask = 0;

    return static_cast<int>(m_currentVertCount++);
  }

  const SelectableVertex& getVertex(int idx) const {
    return m_selectableVertices[idx];
  }

  SelectableVertex& getVertexRef(int idx) { return m_selectableVertices[idx]; }

  void selectVertex(int idx) {
    m_selection.clear();
    m_selection.push_back(idx);
  }

  void addToSelection(int idx) { m_selection.push_back(idx); }

  void clearSelection() { m_selection.clear(); }

  const std::vector<int>& getSelection() const { return m_selection; }
  int getLastSelected() const {
    if (m_selection.size() > 0)
      return m_selection.back();

    return -1;
  }

  void update(const FrameContext& frame);

  SceneQueryResult
  query(const glm::vec3& cameraPos, const glm::vec3& cursorDir);

  void draw(
      const Application& app,
      VkCommandBuffer commandBuffer,
      const FrameContext& frame,
      VkDescriptorSet heapSet,
      UniformHandle globalUniformsHandle,
      float scale = 1.0f);

  void tryRecompileShaders(Application& app) { m_pass.tryRecompile(app); }

  void enableGizmo() { m_bEnableGizmo = true; }
  void disableGizmo() { m_bEnableGizmo = false; }
  bool isGizmoEnabled() const { return m_bEnableGizmo; }
  const glm::vec3& getGizmoPosition() const { return m_gizmoPosition; }
  glm::vec3 getGizmoVertexPosition(int axis) const {
    return m_gizmoPosition + m_gizmoVertices[axis].position;
  }
  void setGizmoPosition(const glm::vec3& position) {
    m_gizmoPosition = position;
  }
  void setGizmoVertexPosition(const glm::vec3& position, int axis) {
    m_gizmoPosition = position - m_gizmoVertices[axis].position;
  }

private:
  SelectableVertex m_selectableVertices[MAX_SELECTABLE_VERTS_COUNT];

  SelectableVertex m_gizmoVertices[3];
  glm::vec3 m_gizmoPosition{};
  bool m_bEnableGizmo{};

  glm::vec3 m_originalPosition{};
  glm::vec3 m_constrainPlaneNormal{};

  uint32_t m_currentVertCount = 0;

  std::vector<int> m_selection{};

  DynamicVertexBuffer<SelectableVertex> m_selectableVB;
  VertexBuffer<glm::vec3> m_sphereVB;
  IndexBuffer m_sphereIB;

  VertexBuffer<glm::vec3> m_cylinderVB;
  IndexBuffer m_cylinderIB;

  RenderPass m_pass;
  FrameBuffer m_frameBuffer;
};
} // namespace RibCage