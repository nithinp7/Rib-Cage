#include "ClothSim.h"

#include <Althea/DeferredRendering.h>
#include <Althea/Gui.h>
#include <glm/gtc/constants.hpp>

using namespace AltheaEngine;

#define MAX_NODES 10000 // 100x100
#define MAX_DISTANCE_CONSTRAINTS 100000

namespace RibCage {
struct PushConstants {
  uint32_t clothUniforms;
};

ClothSim::ClothSim(
    Application& app,
    SingleTimeCommandBuffer& commandBuffer,
    const GBufferResources& gBuffer,
    GlobalHeap& heap) {
  m_uniforms = TransientUniforms<ClothUniforms>(app);
  m_uniforms.registerToHeap(heap);

  {
    // Render pass

    std::vector<SubpassBuilder> subpasses;
    {
      SubpassBuilder& builder = subpasses.emplace_back();

      GBufferResources::setupAttachments(builder);

      builder.pipelineBuilder
          .setCullMode(VK_CULL_MODE_NONE)

          .addVertexShader(GProjectDirectory + "/Shaders/Cloth/Cloth.vert")
          .addFragmentShader(
              GProjectDirectory + "/Shaders/GBufferPassThrough.frag")

          .layoutBuilder.addDescriptorSet(heap.getDescriptorSetLayout())
          .addPushConstants<PushConstants>();
    }

    std::vector<Attachment> attachments = gBuffer.getAttachmentDescriptions();
    m_renderPass = RenderPass(
        app,
        app.getSwapChainExtent(),
        std::move(attachments),
        std::move(subpasses));

    m_frameBuffer = FrameBuffer(
        app,
        m_renderPass,
        app.getSwapChainExtent(),
        gBuffer.getAttachmentViewsA());
  }

  // TODO: Compute passes..

  // Resources
  {
    m_nodes = StructuredBuffer<Node>(app, MAX_NODES);
    m_nodes.registerToHeap(heap);

    m_distanceConstraints =
        StructuredBuffer<DistanceConstraint>(app, MAX_DISTANCE_CONSTRAINTS);
    m_distanceConstraints.registerToHeap(heap);

    m_nodePositions = DynamicVertexBuffer<glm::vec3>(app, MAX_NODES, true);
    m_nodePositions.registerToHeap(heap);

    // TODO: Should this be in a dynamic vert buffer as well...
    m_prevPositions.resize(MAX_NODES);
  }

  // Set up cloth section
  {
    float cellSize = 1.0f;
    float cellDiagonal = cellSize * glm::sqrt(2.0f);

    uint32_t width = 10;
    for (uint32_t i = 0; i < width; ++i) {
      for (uint32_t j = 0; j < width; ++j) {
        uint32_t flatIdx = i * width + j;
        glm::vec3 position = glm::vec3(i * cellSize, 0.0f, j * cellSize);

        Node node{};
        node.position = position; // TODO: Remove...
        node.objectIdx = flatIdx / 12;
        m_nodes.setElement(node, flatIdx);
        m_nodePositions.setVertex(position, flatIdx);
        m_prevPositions[flatIdx] = position;
      }
    }
    m_nodes.upload(app, commandBuffer);

    uint32_t distConstraintCount = 0;

    // Setup indices
    std::vector<uint32_t> indices;
    indices.reserve((width - 1) * (width - 1) * 2 * 3);
    for (uint32_t i = 0; i < width - 1; ++i) {
      for (uint32_t j = 0; j < width - 1; ++j) {
        uint32_t idx0 = i * width + j;
        uint32_t idx1 = i * width + j + 1;
        uint32_t idx2 = (i + 1) * width + j + 1;
        uint32_t idx3 = (i + 1) * width + j;

        // TODO: Sort out front-face / back-face here...
        indices.push_back(idx0);
        indices.push_back(idx1);
        indices.push_back(idx2);

        indices.push_back(idx0);
        indices.push_back(idx2);
        indices.push_back(idx3);

        if (i == 0)
          m_distanceConstraints.setElement(
              DistanceConstraint{idx0, idx1, cellSize, 0.0f},
              distConstraintCount++);
        if (j == 0)
          m_distanceConstraints.setElement(
              DistanceConstraint{idx0, idx3, cellSize, 0.0f},
              distConstraintCount++);

        m_distanceConstraints.setElement(
            DistanceConstraint{idx1, idx2, cellSize, 0.0f},
            distConstraintCount++);
        m_distanceConstraints.setElement(
            DistanceConstraint{idx2, idx3, cellSize, 0.0f},
            distConstraintCount++);

        m_distanceConstraints.setElement(
            DistanceConstraint{idx0, idx2, cellDiagonal, 0.0f},
            distConstraintCount++);
        m_distanceConstraints.setElement(
            DistanceConstraint{idx1, idx3, cellDiagonal, 0.0f},
            distConstraintCount++);
      }
    }
    m_distanceConstraints.upload(app, commandBuffer);

    ClothSection& section = m_clothSections.emplace_back();
    section.indices =
        IndexBuffer(app, (VkCommandBuffer)commandBuffer, std::move(indices));
  }

  // Setup AABB
  m_aabb = AABBManager(
      app,
      gBuffer,
      heap,
      m_clothSections[0].indices.getIndexCount() / 3);

  m_collisions = CollisionsManager(app, commandBuffer, gBuffer, heap);
}

void ClothSim::tryRecompileShaders(Application& app) {
  m_renderPass.tryRecompile(app);
  for (ComputePipeline& computePass : m_solvePasses)
    computePass.tryRecompile(app);

  m_aabb.tryRecompile(app);
}

static bool s_simPaused = false;
static int s_solverSubsteps = 1;
static float s_damping = 0.0f;
static float s_k = 1.0f;
static float s_gravity = 1.0f;

static bool s_fixTop = true;
static bool s_fixBottom = true;
static float s_twist = 0.0f;

void ClothSim::update(const FrameContext& frame) {
  if (!s_simPaused) {
    // TODO: Sim solver step
    float dt = 1.0f / 30.0f;

    // TODO: time substeps
    // Apply gravity
    glm::vec3 gravity(0.0f, -s_gravity, 0.0f);
    for (uint32_t nodeIdx = 0; nodeIdx < m_nodePositions.getVertexCount();
         ++nodeIdx) {
      glm::vec3& pos = m_nodePositions.getVertex(nodeIdx);
      glm::vec3 velDt = pos - m_prevPositions[nodeIdx];

      m_prevPositions[nodeIdx] = pos;

      pos += 0.5f * gravity * dt * dt + velDt * (1.0f - s_damping);
    }

    for (uint32_t solverStep = 0; solverStep < s_solverSubsteps; ++solverStep) {
      // TODO: Not actually the right number of dist constraints...
      for (uint32_t constraintIdx = 0;
           constraintIdx < m_distanceConstraints.getCount();
           ++constraintIdx) {
        const DistanceConstraint& c =
            m_distanceConstraints.getElement(constraintIdx);
        glm::vec3& p0 = m_nodePositions.getVertex(c.a);
        glm::vec3& p1 = m_nodePositions.getVertex(c.b);

        glm::vec3 diff = p1 - p0;
        float dist = glm::length(diff);
        diff /= dist;

        if (dist < 0.001f)
          diff = glm::vec3(1.0f, 0.0f, 0.0f);

        glm::vec3 disp = 0.5f * s_k * (c.restLength - dist) * diff;
        p0 -= disp;
        p1 += disp;
      }

      uint32_t width = 10;
      float cellSize = 1.0f;
      // Fixed top row
      if (s_fixTop) {
        for (uint32_t nodeIdx = 0; nodeIdx < width; ++nodeIdx) {
          glm::vec3 target(nodeIdx * cellSize, 0.0f, 0.0f);
          glm::vec3& pos = m_nodePositions.getVertex(nodeIdx);
          glm::vec3 diff = target - pos;
          pos += s_k * diff;
        }
      }

      // Fixed bottom row
      if (s_fixBottom) {
        glm::vec3 targetCenter(0.5f * width * cellSize, 0.0f, (width - 1) * cellSize);
        float theta = s_twist * 2.0f * glm::pi<float>();
        float cosTheta = glm::cos(theta);
        float sinTheta = glm::sin(theta);

        for (uint32_t i = 0; i < width; ++i) {
          uint32_t nodeIdx = width * (width - 1) + i;
          float x = i * cellSize - 0.5f * width;
          glm::vec3 target = targetCenter + glm::vec3(cosTheta * x, sinTheta * x, 0.0f);
          glm::vec3& pos = m_nodePositions.getVertex(nodeIdx);
          glm::vec3 diff = target - pos;
          pos += s_k * diff;
        }
      }
    }
  }

  m_nodePositions.upload(frame.frameRingBufferIndex);

  // Fix-up AABB
  {
    const std::vector<uint32_t>& indices =
        m_clothSections[0].indices.getIndices();
    const std::vector<glm::vec3>& positions = m_nodePositions.getVertices();

    m_aabb.update(indices, positions, m_prevPositions, frame);
    m_collisions.update(frame, indices, positions, m_prevPositions, m_aabb.getTree());
  }
}

void ClothSim::draw(
    const Application& app,
    VkCommandBuffer commandBuffer,
    const FrameContext& frame,
    VkDescriptorSet heapSet,
    BufferHandle globalResourcesHandle,
    UniformHandle globalUniformsHandle) {

  // Update uniforms
  {
    ClothUniforms uniforms{};
    uniforms.globalResources = globalResourcesHandle.index;
    uniforms.globalUniforms = globalUniformsHandle.index;

    uniforms.deltaTime = frame.deltaTime;

    uniforms.nodes = m_nodes.getHandle().index;
    uniforms.nodePositions =
        m_nodePositions.getCurrentBufferHandle(frame.frameRingBufferIndex)
            .index;
    uniforms.nodesCount = MAX_NODES;

    uniforms.distanceConstraints = m_distanceConstraints.getHandle().index;
    uniforms.distanceConstraintsCount = MAX_DISTANCE_CONSTRAINTS;

    m_uniforms.updateUniforms(uniforms, frame);
  }

  PushConstants push{};
  push.clothUniforms = m_uniforms.getCurrentHandle(frame).index;

  // Draw
  {
    ActiveRenderPass pass =
        m_renderPass.begin(app, commandBuffer, frame, m_frameBuffer);

    pass.setGlobalDescriptorSets(gsl::span(&heapSet, 1));
    pass.getDrawContext().bindDescriptorSets();
    pass.getDrawContext().updatePushConstants(push, 0);

    for (const ClothSection& section : m_clothSections) {
      pass.getDrawContext().bindIndexBuffer(section.indices);
      pass.getDrawContext().drawIndexed(section.indices.getIndexCount());
    }
  }

  m_aabb.debugDraw(
      app,
      commandBuffer,
      frame,
      heapSet,
      globalResourcesHandle,
      globalUniformsHandle);

  m_collisions.draw(app, commandBuffer, frame, heapSet, globalUniformsHandle);
}

void ClothSim::updateUI() {
  if (ImGui::CollapsingHeader("Cloth Sim", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Paused:");
    ImGui::Checkbox("##paused", &s_simPaused);

    ImGui::Separator();
    ImGui::Text("Substeps:");
    ImGui::SliderInt("##substeps", &s_solverSubsteps, 1, 8);
    ImGui::Text("Damping:");
    ImGui::SliderFloat("##damping", &s_damping, 0.0f, 1.0f);
    ImGui::Text("K:");
    ImGui::SliderFloat("##springstrength", &s_k, 0.01f, 1.0f);
    ImGui::Text("Gravity:");
    ImGui::SliderFloat("##gravity", &s_gravity, 0.25f, 4.0f);

    ImGui::Separator();
    ImGui::Text("Fix Sheet Top:");
    ImGui::Checkbox("##fixtop", &s_fixTop);
    ImGui::Text("Fix Sheet Bottom:");
    ImGui::Checkbox("##fixbottom", &s_fixBottom);
    ImGui::Text("Twist Percentage:");
    ImGui::SliderFloat("##twistamt", &s_twist, 0.0f, 1.0f);
  }

  m_aabb.updateUI();
  m_collisions.updateUI();
}
} // namespace RibCage