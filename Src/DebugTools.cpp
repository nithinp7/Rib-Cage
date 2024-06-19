#include "DebugTools.h"

#include <Althea/ShapeUtilities.h>

#include <limits>

#define SELECTION_RADIUS 0.5f
#define GIZMO_LENGTH 5.0f

namespace {
struct PushConstants {
  uint32_t globalUniformsHandle;
  uint32_t selectableVBHandle;
  float selectionRadius;
};

struct GizmoPushConstants {
  alignas(16) glm::vec3 gizmoPos;
  alignas(4) float gizmoScale;
  float gizmoLength;
  uint32_t gizmoPart; // 0 for the cylinders 1 for sphere
  uint32_t globalUniformsHandle;
};

struct DebugLinePush {
  uint32_t globalUniforms;
};
} // namespace
namespace RibCage {
DebugVisualizationScene::DebugVisualizationScene(Application& app) {
  m_lines = DynamicVertexBuffer<DebugVert>(app, 2 * MAX_DBG_LINES, true);
}

// IGBufferSubpass impl
void DebugVisualizationScene::registerGBufferSubpass(
    GraphicsPipelineBuilder& builder) const {

  builder.setPrimitiveType(PrimitiveType::LINES)
      .setLineWidth(2.5f)
      .addVertexInputBinding<DebugVert>(VK_VERTEX_INPUT_RATE_VERTEX)
      .addVertexAttribute(
          VertexAttributeType::VEC3,
          offsetof(DebugVert, position))
      .addVertexAttribute(VertexAttributeType::UINT, offsetof(DebugVert, color))

      .addVertexShader(GProjectDirectory + "/Shaders/Debug/DebugLine.vert")
      .addFragmentShader(GProjectDirectory + "/Shaders/GBufferPassThrough.frag")

      .layoutBuilder //
      .addPushConstants<DebugLinePush>(VK_SHADER_STAGE_ALL);
}

void DebugVisualizationScene::beginGBufferSubpass(
    const DrawContext& context,
    BufferHandle globalResources,
    UniformHandle globalUniforms) {
  const FrameContext& frame = context.getFrame();

  m_lines.upload(frame.frameRingBufferIndex);
  {
    DebugLinePush constants{};
    constants.globalUniforms = globalUniforms.index;

    context.updatePushConstants(constants, 0);
    context.bindDescriptorSets();

    VkBuffer vb = m_lines.getBuffer();
    size_t offset = m_lines.getCurrentBufferOffset(frame.frameRingBufferIndex);
    vkCmdBindVertexBuffers(context.getCommandBuffer(), 0, 1, &vb, &offset);
    context.draw(2 * m_lineCount);
  }
}

SelectableScene::SelectableScene(
    Application& app,
    GlobalHeap& heap,
    VkCommandBuffer commandBuffer,
    const GBufferResources& gBuffer) {
  m_gizmoVertices[0].position = glm::vec3(GIZMO_LENGTH, 0.0f, 0.0f);
  m_gizmoVertices[1].position = glm::vec3(0.0f, GIZMO_LENGTH, 0.0f);
  m_gizmoVertices[2].position = glm::vec3(0.0f, 0.0f, GIZMO_LENGTH);

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

    GBufferResources::setupAttachments(builder);

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

    GBufferResources::setupAttachments(builder);

    builder.pipelineBuilder
        .addVertexInputBinding<glm::vec3>(VK_VERTEX_INPUT_RATE_VERTEX)
        .addVertexAttribute(VertexAttributeType::VEC3, 0)

        .addVertexShader(GProjectDirectory + "/Shaders/TranslateGizmo.vert")
        .addFragmentShader(
            GProjectDirectory + "/Shaders/GBufferPassThrough.frag")

        .layoutBuilder //
        .addDescriptorSet(heap.getDescriptorSetLayout())
        .addPushConstants<GizmoPushConstants>(VK_SHADER_STAGE_ALL);
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
      gBuffer.getAttachmentViewsA());
}

SceneQueryResult
SelectableScene::query(const glm::vec3& cameraPos, const glm::vec3& cursorDir) {
  SceneQueryResult result{};
  result.hitType = QHT_NONE;
  result.t = std::numeric_limits<float>::max();
  result.index = -1;

  for (int i = 0; i < m_currentVertCount; ++i) {
    SelectableVertex& v = m_selectableVertices[i];
    float t;
    if (v.intersect(cameraPos, cursorDir, t) && t < result.t) {
      result.hitType = QHT_VERTEX;
      result.index = i;
      result.t = t;
    }
  }

  if (m_bEnableGizmo) {
    for (int i = 0; i < 3; ++i) {
      SelectableVertex& v = m_gizmoVertices[i];
      float t;
      if (v.intersect(cameraPos - m_gizmoPosition, cursorDir, t) &&
          t < result.t) {
        result.hitType = QHT_GIZMO_HANDLE;
        result.index = i;
        result.t = t;
      }
    }
  }

  return result;
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
    UniformHandle globalUniformsHandle,
    float scale) {

  {

    ActiveRenderPass pass =
        m_pass.begin(app, commandBuffer, frame, m_frameBuffer);

    pass.setGlobalDescriptorSets(gsl::span(&heapSet, 1));

    const DrawContext& context = pass.getDrawContext();
    {
      PushConstants constants{};
      constants.globalUniformsHandle = globalUniformsHandle.index;
      constants.selectableVBHandle =
          m_selectableVB.getCurrentBufferHandle(frame.frameRingBufferIndex)
              .index;
      constants.selectionRadius = scale;
      context.updatePushConstants(constants, 0);
      context.bindDescriptorSets();

      context.bindVertexBuffer(m_sphereVB);
      context.bindIndexBuffer(m_sphereIB);
      context.drawIndexed(m_sphereIB.getIndexCount(), m_currentVertCount);
    }

    pass.nextSubpass();

    if (m_bEnableGizmo) {
      GizmoPushConstants constants{};
      constants.globalUniformsHandle = globalUniformsHandle.index;
      constants.gizmoScale = SELECTION_RADIUS;
      constants.gizmoLength = GIZMO_LENGTH;
      constants.gizmoPart = 0;
      constants.gizmoPos = m_gizmoPosition;

      context.updatePushConstants(constants, 0);
      context.bindDescriptorSets();
      context.bindVertexBuffer(m_cylinderVB);
      context.bindIndexBuffer(m_cylinderIB);
      context.drawIndexed(m_cylinderIB.getIndexCount(), 3);

      constants.gizmoPart = 1;
      context.updatePushConstants(constants, 0);
      context.bindDescriptorSets();
      context.bindVertexBuffer(m_sphereVB);
      context.bindIndexBuffer(m_sphereIB);
      context.drawIndexed(m_sphereIB.getIndexCount(), 3);
    }
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