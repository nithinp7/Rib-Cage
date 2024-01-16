#include "DebugTools.h"

#include <Althea/ShapeUtilities.h>

#include <limits>

#define SELECTION_RADIUS 0.1f
namespace {
struct PushConstants {
  uint32_t globalUniformsHandle;
  float selectionRadius;
};
} // namespace
namespace RibCage {
/*static*/
void SelectableScene::buildPipeline(
    GraphicsPipelineBuilder& builder,
    VkDescriptorSetLayout heapLayout) {
  builder
      // instance data
      .addVertexInputBinding<SelectableVertex>(VK_VERTEX_INPUT_RATE_INSTANCE)
      .addVertexAttribute(VertexAttributeType::VEC3, 0)
      .addVertexAttribute(VertexAttributeType::UINT, 1)
      // vertex data
      .addVertexInputBinding<glm::vec3>(VK_VERTEX_INPUT_RATE_VERTEX)
      .addVertexAttribute(VertexAttributeType::VEC3, 0) // vert pos

      .addVertexShader(GProjectDirectory + "/Shaders/SelectableVertex.vert")
      .addFragmentShader(GProjectDirectory + "/Shaders/SelectableVertex.frag")

      .layoutBuilder //
      .addDescriptorSet(heapLayout)
      .addPushConstants<PushConstants>(VK_SHADER_STAGE_ALL);
}

SelectableScene::SelectableScene(
    Application& app,
    VkCommandBuffer commandBuffer) {
  m_selectableVB =
      DynamicVertexBuffer<SelectableVertex>(app, MAX_SELECTABLE_VERTS_COUNT);
  ShapeUtilities::createSphere(app, commandBuffer, m_sphereVB, m_sphereIB, 8);
}

bool SelectableScene::trySelect(
    const glm::vec3& cameraPos,
    const glm::vec3& cursorDir,
    bool bAddToSelection) {

  int selectedIndex = -1;
  float minT = std::numeric_limits<float>::max();
  for (int i = 0; i < m_currentVertCount; ++i) {
    SelectableVertex& vertex = m_selectableVertices[i];
    float t;
    if (vertex.intersect(cameraPos, cursorDir, t) && t < minT) {
      selectedIndex = i;
      minT = t;
    }
  }

  if (selectedIndex != -1) {
    if (bAddToSelection)
      addToSelection(selectedIndex);
    else
      selectVertex(selectedIndex);

    return true;
  }

  return false;
}

void SelectableScene::update(const FrameContext& frame) {
  m_selectableVB.updateVertices(
      frame.frameRingBufferIndex,
      gsl::span(m_selectableVertices, MAX_SELECTABLE_VERTS_COUNT));
}

void SelectableScene::drawSubpass(
    const DrawContext& context,
    UniformHandle globalUniformsHandle) {
  PushConstants constants{};
  constants.globalUniformsHandle = globalUniformsHandle.index;
  constants.selectionRadius = SELECTION_RADIUS;
  context.updatePushConstants(constants, 0);
  context.bindDescriptorSets();
  context.bindIndexBuffer(m_sphereIB);

  VkBuffer VBs[] = {
      m_selectableVB.getBuffer(),
      m_sphereVB.getAllocation().getBuffer()};
  VkDeviceSize offsets[] = {0, 0};
  vkCmdBindVertexBuffers(context.getCommandBuffer(), 0, 2, VBs, offsets);

  context.drawIndexed(m_sphereIB.getIndexCount(), m_currentVertCount);
}

bool SelectableVertex::intersect(
    const glm::vec3& orig,
    const glm::vec3& dir,
    float& t) const {
  // ||orig + t * dir - pos|| = r
  // let diff = orig - pos ==>
  // ||diff + t dir|| = r
  // t2 + 2t dir * diff + diff * diff = r2
  // ==>
  // a = 1, b = 2 dir * diff, c = diff * diff - r2

  glm::vec3 diff = orig - position;
  float distSq = glm::dot(diff, diff);

  float b = 2.0 * glm::dot(diff, dir);
  float c = glm::dot(diff, diff) - SELECTION_RADIUS * SELECTION_RADIUS;

  float b2_4ac = b * b - 4.0 * c;
  if (b2_4ac < 0.0) {
    return false;
  }

  float _0_5_sqrt_b2_4ac = 0.5 * sqrt(b2_4ac);
  float _0_5_b = -0.5 * b;
  float t0 = _0_5_b - _0_5_sqrt_b2_4ac;
  float t1 = _0_5_b + _0_5_sqrt_b2_4ac;
  t = t0 < 0.0 ? t1 : t0;

  return t >= 0.0;
}
} // namespace RibCage