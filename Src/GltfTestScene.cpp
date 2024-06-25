#include "GltfTestScene.h"

using namespace AltheaEngine;

namespace RibCage {

void GltfTestScene::init(
    Application& app,
    SingleTimeCommandBuffer& commandBuffer,
    SceneToGBufferPassBuilder& gBufferPassBuilder,
    GlobalHeap& heap) {
  {
    std::string path = GProjectDirectory + "/Data/ImportedModels/test.gltf";
    Model& heli = m_models.emplace_back(app, commandBuffer, path);
    heli.registerToHeap(heap);
    heli.createConstantBuffers(app, commandBuffer, heap);

    int32_t animationIdx = heli.getAnimationIndex("All Animations");
    heli.startAnimation(animationIdx, true);
  }

  {
    std::string path = GProjectDirectory + "/Data/ImportedModels/interpolationTest.gltf";
    // std::string path = GProjectDirectory + "/Data/ImportedModels/interpolationTest.glb";
    Model& test = m_models.emplace_back(app, commandBuffer, path);
    test.registerToHeap(heap);
    test.createConstantBuffers(app, commandBuffer, heap);

    int32_t animationIdx = test.getAnimationIndex("Linear Rotation");
    test.startAnimation(animationIdx, true);
  }

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

void GltfTestScene::registerGBufferSubpass(
    GraphicsPipelineBuilder& builder) const {
  Primitive::buildPipeline(builder);
  builder.addVertexShader(GProjectDirectory + "/Shaders/Gltf/Gltf.vert");
  builder.addFragmentShader(GProjectDirectory + "/Shaders/Gltf/Gltf.frag");
  builder.layoutBuilder.addPushConstants<GltfPushConstants>();
}

void GltfTestScene::update(const FrameContext& frame) {
  for (Model& model : m_models)
    model.updateAnimation(frame.deltaTime);
}

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