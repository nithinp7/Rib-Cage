#include "DebugTools.h"

#include <Althea/ShapeUtilities.h>

#include <limits>

#define SELECTION_RADIUS 0.5f
namespace {
struct PushConstants {
  uint32_t globalUniformsHandle;
  uint32_t selectableVBHandle;
  float selectionRadius;
};
} // namespace
namespace RibCage {
SelectableScene::SelectableScene(
    Application& app,
    GlobalHeap& heap,
    VkCommandBuffer commandBuffer,
    const GBufferResources& gBuffer) {
  m_selectableVB =
      DynamicVertexBuffer<SelectableVertex>(app, MAX_SELECTABLE_VERTS_COUNT);
  m_selectableVB.registerToHeap(heap);

  ShapeUtilities::createSphere(app, commandBuffer, m_sphereVB, m_sphereIB, 16);
  ShapeUtilities::createCylinder(
      app,
      commandBuffer,
      m_cylinderVB,
      m_cylinderIB,
      16);

  std::vector<SubpassBuilder> builders;

  // Selectable nodes
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

        .addVertexShader(GProjectDirectory + "/Shaders/SelectableVertex.vert")
        .addFragmentShader(
            GProjectDirectory + "/Shaders/GBufferPassThrough.frag")

        .layoutBuilder //
        .addDescriptorSet(heap.getDescriptorSetLayout())
        .addPushConstants<PushConstants>(VK_SHADER_STAGE_ALL);
  }

  // Translation gizmos
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

        .addVertexShader(GProjectDirectory + "/Shaders/TranslateGizmo.vert")
        .addFragmentShader(
            GProjectDirectory + "/Shaders/GBufferPassThrough.frag")

        .layoutBuilder //
        .addDescriptorSet(heap.getDescriptorSetLayout())
        .addPushConstants<PushConstants>(VK_SHADER_STAGE_ALL);
  }

  std::vector<Attachment> attachments = gBuffer.getAttachmentDescriptions();
  for (auto& attachment : attachments)
    attachment.load = true;

  m_pass = RenderPass(
      app,
      app.getSwapChainExtent(),
      std::move(attachments),
      std::move(builders));
  m_frameBuffer = FrameBuffer(
      app,
      m_pass,
      app.getSwapChainExtent(),
      gBuffer.getAttachmentViews());
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
  // Update selected bit
  for (int i = 0; i < m_currentVertCount; ++i)
    m_selectableVertices[i].infoMask &= ~SelectionInfoMaskBits::SELECTED;
  for (int selected : m_selection)
    m_selectableVertices[selected].infoMask |= SelectionInfoMaskBits::SELECTED;

  m_selectableVB.updateVertices(
      frame.frameRingBufferIndex,
      gsl::span(m_selectableVertices, MAX_SELECTABLE_VERTS_COUNT));
}

void SelectableScene::draw(
    const Application& app,
    VkCommandBuffer commandBuffer,
    const FrameContext& frame,
    VkDescriptorSet heapSet,
    UniformHandle globalUniformsHandle) {

  {
    PushConstants constants{};
    constants.globalUniformsHandle = globalUniformsHandle.index;
    constants.selectableVBHandle =
        m_selectableVB.getCurrentBufferHandle(frame.frameRingBufferIndex).index;
    constants.selectionRadius = SELECTION_RADIUS;

    ActiveRenderPass pass =
        m_pass.begin(app, commandBuffer, frame, m_frameBuffer);

    pass.setGlobalDescriptorSets(gsl::span(&heapSet, 1));

    const DrawContext& context = pass.getDrawContext();
    context.updatePushConstants(constants, 0);
    context.bindDescriptorSets();

    context.bindVertexBuffer(m_sphereVB);
    context.bindIndexBuffer(m_sphereIB);
    context.drawIndexed(m_sphereIB.getIndexCount(), m_currentVertCount);

    pass.nextSubpass();

    context.bindVertexBuffer(m_cylinderVB);
    context.bindIndexBuffer(m_cylinderIB);
    context.drawIndexed(m_cylinderIB.getIndexCount(), 3 * m_currentVertCount);
  }
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