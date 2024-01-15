#pragma once

#include <Althea/Application.h>
#include <Althea/DynamicVertexBuffer.h>
#include <Althea/IndexBuffer.h>
#include <Althea/RenderPass.h>
#include <Althea/VertexBuffer.h>

#include <glm/glm.hpp>
#include <vulkan/vulkan.h>

#include <vector>

#define MAX_SELECTABLE_VERTS_COUNT 1024

using namespace AltheaEngine;

namespace RibCage {

struct SelectableVertex {
  alignas(16) glm::vec3 position;
  alignas(4) float selectRadius;

  bool intersect(const glm::vec3& orig, const glm::vec3& dir, float& t) const;
};

// Debug scene elements that are selectable
class SelectableScene {
public:
  static void
  buildSubpass(SubpassBuilder& builder, VkDescriptorSetLayout heapLayout);

  SelectableScene() = default;
  SelectableScene(Application& app, VkCommandBuffer commandBuffer);

  int addVertex(const glm::vec3& pos, float radius) {
    if (m_currentVertCount == MAX_SELECTABLE_VERTS_COUNT)
      return -1;

    SelectableVertex& v = m_selectableVertices[m_currentVertCount];
    v.position = pos;
    v.selectRadius = radius;

    return static_cast<int>(m_currentVertCount++);
  }

  const SelectableVertex& getVertex(int idx) const {
    return m_selectableVertices[idx];
  }

  SelectableVertex& getVertexRef(int idx) { return m_selectableVertices[idx]; }

  void update(
      const glm::vec3& cameraPos,
      const glm::vec3& cursorDir,
      bool mouseButton);

  void drawSubpass(const DrawContext& context, uint32_t globalUniformsHandle);

private:
  SelectableVertex m_selectableVertices[MAX_SELECTABLE_VERTS_COUNT];
  uint32_t m_currentVertCount = 0;
  glm::vec3 m_originalPosition{};
  glm::vec3 m_constrainPlaneNormal{};
  int m_currentSelectedVertex = -1;

  DynamicVertexBuffer<SelectableVertex> m_selectableVB;
  VertexBuffer<glm::vec3> m_sphereVB;
  IndexBuffer m_sphereIB;
};
} // namespace RibCage