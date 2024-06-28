#include "GltfTestScene.h"

#include <Althea/Gui.h>

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
  }

  {
    std::string path =
        GProjectDirectory + "/Data/ImportedModels/interpolationTest.gltf";
    // std::string path = GProjectDirectory +
    // "/Data/ImportedModels/interpolationTest.glb";
    Model& test = m_models.emplace_back(app, commandBuffer, path);
    test.registerToHeap(heap);
    test.createConstantBuffers(app, commandBuffer, heap);
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
  m_animationSystem.update(frame.deltaTime);
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

void GltfTestScene::updateUI() {
  // TODO: actually pull out model names from paths
  static std::vector<std::string> s_modelNames;
  static int s_modelIdx;
  if (s_modelNames.size() != m_models.size()) {
    s_modelNames.clear();
    s_modelIdx = 0;
    for (int i = 0; i < m_models.size(); ++i) {
      char buf[128];
      sprintf(buf, "Model %d", i);
      s_modelNames.emplace_back(buf);
    }
  }

  if (ImGui::CollapsingHeader("Gltf Animations")) {
    ImGui::Indent();

    ImGui::Text("Select Model:");
    if (ImGui::BeginCombo("##selectmodel", s_modelNames[s_modelIdx].c_str())) {
      for (int i = 0; i < m_models.size(); ++i) {
        bool bSelected = s_modelIdx == i;
        if (ImGui::Selectable(s_modelNames[i].c_str(), bSelected)) {
          s_modelIdx = i;
        }
      }
      ImGui::EndCombo();
    }

    static int s_animationIdx = 0;
    static bool s_loop = false;
    Model& model = m_models[s_modelIdx];
    int animCount = model.getAnimationCount();

    ImGui::Text("Select Animation:");
    if (animCount > 0) {
      if (s_animationIdx >= animCount)
        s_animationIdx = 0;
      if (ImGui::BeginCombo(
              "##selectanimation",
              model.getAnimationName(s_animationIdx).c_str())) {
        for (int i = 0; i < animCount; ++i) {
          bool bSelected = s_animationIdx == i;
          if (ImGui::Selectable(model.getAnimationName(i).c_str(), bSelected)) {
            s_animationIdx = i;
          }
        }

        ImGui::EndCombo();
      }
    }

    if (ImGui::Button("Play")) {
      if (s_animationIdx >= 0 && s_animationIdx < animCount) {
        m_animationSystem.startAnimation(&model, s_animationIdx, s_loop);
      }
    }
    ImGui::SameLine();
    ImGui::Checkbox("Loop", &s_loop);

    if (ImGui::Button("Stop All")) {
      m_animationSystem.stopAllAnimations();
    }

    ImGui::Unindent();
  }
}
} // namespace RibCage