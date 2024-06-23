#include "GltfTestScene.h"

using namespace AltheaEngine;

namespace RibCage {

void GltfTestScene::init(
    Application& app,
    SingleTimeCommandBuffer& commandBuffer,
    SceneToGBufferPassBuilder& gBufferPassBuilder,
    GlobalHeap& heap) {
  std::string path = GProjectDirectory + "/Data/ImportedModels/test.gltf";
  // std::string path = GEngineDirectory + "/Content/Models/DamagedHelmet.glb";
  m_models.emplace_back(app, commandBuffer, path);
  m_models.back().registerToHeap(heap);
  m_models.back().createConstantBuffers(app, commandBuffer, heap);
  gBufferPassBuilder.registerSubpass(IntrusivePtr(this));
}

namespace {
struct GltfPushConstants {
  glm::mat4 model;
  uint32_t primConstantsHandle;
  uint32_t globalResourcesHandle;
  uint32_t globalUniformsHandle;
};
} // namespace

void GltfTestScene::registerGBufferSubpass(GraphicsPipelineBuilder& builder) const {
  Primitive::buildPipeline(builder);
  builder.addVertexShader(GProjectDirectory + "/Shaders/Gltf/Gltf.vert");
  builder.addFragmentShader(GProjectDirectory + "/Shaders/Gltf/Gltf.frag");
  builder.layoutBuilder.addPushConstants<GltfPushConstants>();
}

void GltfTestScene::update(const FrameContext& frame) {}

void GltfTestScene::beginGBufferSubpass(
    const DrawContext& context,
    BufferHandle globalResourcesHandle,
    UniformHandle globalUniformsHandle) {

  GltfPushConstants push{};
  push.globalResourcesHandle = globalResourcesHandle.index;
  push.globalUniformsHandle = globalUniformsHandle.index;

  for (const Model& model : m_models) {
    for (const Primitive& primitive : model.getPrimitives()) {
      push.model = primitive.getTransform();
      push.primConstantsHandle = primitive.getConstantBufferHandle().index;
      context.bindDescriptorSets();
      context.updatePushConstants(push, 0);
      context.bindVertexBuffer(primitive.getVertexBuffer());
      context.bindIndexBuffer(primitive.getIndexBuffer());
      context.drawIndexed(primitive.getIndices().size());
    }
  }
}

void GltfTestScene::updateUI() {}
} // namespace RibCage