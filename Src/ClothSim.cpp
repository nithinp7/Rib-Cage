#include "ClothSim.h"

#include <Althea/DeferredRendering.h>

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

    m_nodePositions = DynamicVertexBuffer<glm::vec4>(app, MAX_NODES, true);
    m_nodePositions.registerToHeap(heap);
  }

  // Set up cloth section
  {
    float cellSize = 1.0f;

    uint32_t width = 10;
    for (uint32_t i = 0; i < width; ++i) {
      for (uint32_t j = 0; j < width; ++j) {
        uint32_t flatIdx = i * width + j;
        glm::vec3 position = glm::vec3(i * cellSize, 0.0f, j * cellSize);

        Node node{};
        node.position = position; // TODO: Remove...
        node.objectIdx = flatIdx / 12;
        m_nodes.setElement(node, flatIdx);
        m_nodePositions.setVertex(glm::vec4(position, 1.0f), flatIdx);
      }
    }
    m_nodes.upload(app, commandBuffer);

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
      }
    }
    ClothSection& section = m_clothSections.emplace_back();
    section.indices =
        IndexBuffer(app, (VkCommandBuffer)commandBuffer, std::move(indices));
  }
}

void ClothSim::update(const FrameContext& frame) {
  // TODO: Sim solver step

  m_nodePositions.upload(frame.frameRingBufferIndex);
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
    uniforms.nodePositions = m_nodePositions.getCurrentBufferHandle(frame.frameRingBufferIndex).index;
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
}
} // namespace RibCage