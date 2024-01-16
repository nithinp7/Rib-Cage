#include "RibCage.h"

#include <Althea/Application.h>
#include <Althea/Camera.h>
#include <Althea/Cubemap.h>
#include <Althea/DescriptorSet.h>
#include <Althea/GraphicsPipeline.h>
#include <Althea/Gui.h>
#include <Althea/InputManager.h>
#include <Althea/ModelViewProjection.h>
#include <Althea/Primitive.h>
#include <Althea/SingleTimeCommandBuffer.h>
#include <Althea/Skybox.h>
#include <Althea/Utilities.h>
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vulkan/vulkan.h>

#include <array>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace AltheaEngine;

namespace RibCage {
namespace {
struct ForwardPassPushConstants {
  glm::mat4 model;
  uint32_t primitiveIdx;

  uint32_t globalResources;
  uint32_t globalUniforms;
};

struct DeferredPassPushConstants {
  uint32_t globalResources;
  uint32_t globalUniforms;
  uint32_t reflectionBuffer;
};
} // namespace

RibCage::RibCage() {}

void RibCage::initGame(Application& app) {
  const VkExtent2D& windowDims = app.getSwapChainExtent();
  m_pCameraController = std::make_unique<CameraController>(
      app.getInputManager(),
      90.0f,
      (float)windowDims.width / (float)windowDims.height);
  m_pCameraController->setMaxSpeed(15.0f);

  // TODO: need to unbind these at shutdown
  InputManager& input = app.getInputManager();
  input.addKeyBinding(
      {GLFW_KEY_L, GLFW_PRESS, 0},
      [&adjustingExposure = m_adjustingExposure, &input]() {
        adjustingExposure = true;
        input.setMouseCursorHidden(false);
      });

  input.addKeyBinding(
      {GLFW_KEY_L, GLFW_RELEASE, 0},
      [&adjustingExposure = m_adjustingExposure, &input]() {
        adjustingExposure = false;
        input.setMouseCursorHidden(true);
      });

  // Recreate any stale pipelines (shader hot-reload)
  input.addKeyBinding(
      {GLFW_KEY_R, GLFW_PRESS, GLFW_MOD_CONTROL},
      [&app, that = this]() {
        that->m_pointLights.getShadowMapPass().tryRecompile(app);
        that->m_forwardPass.tryRecompile(app);
        that->m_deferredPass.tryRecompile(app);
      });

  input.addMousePositionCallback(
      [&adjustingExposure = m_adjustingExposure,
       &exposure = m_exposure](double x, double y, bool cursorHidden) {
        if (adjustingExposure) {
          exposure = static_cast<float>(y);
        }
      });
}

void RibCage::shutdownGame(Application& app) { m_pCameraController.reset(); }

void RibCage::createRenderState(Application& app) {
  const VkExtent2D& extent = app.getSwapChainExtent();
  m_pCameraController->getCamera().setAspectRatio(
      (float)extent.width / (float)extent.height);

  Gui::createRenderState(app);

  SingleTimeCommandBuffer commandBuffer(app);
  _createGlobalResources(app, commandBuffer);
  _createForwardPass(app);
  _createDeferredPass(app);
}

void RibCage::destroyRenderState(Application& app) {
  Primitive::resetPrimitiveIndexCount();

  Gui::destroyRenderState(app);

  m_models.clear();

  m_forwardPass = {};
  m_forwardFrameBuffer = {};
  m_primitiveConstantsBuffer = {};

  m_deferredPass = {};
  m_swapChainFrameBuffers = {};

  m_pointLights = {};

  m_SSR = {};
  m_globalUniforms = {};

  m_globalResources = {};
  m_globalHeap = {};

  m_skeletonEditor = {};
  m_debugScene = {};
}

void RibCage::tick(Application& app, const FrameContext& frame) {
  {
    Gui::startRecordingImgui();
    const ImGuiViewport* main_viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(main_viewport->WorkPos.x + 650, main_viewport->WorkPos.y + 20),
        ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(220, 100), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Debug Options")) {
      if (ImGui::CollapsingHeader("Lighting")) {
        ImGui::Text("Exposure:");
        ImGui::SliderFloat("##exposure", &m_exposure, 0.0f, 1.0f);
      }
    }

    ImGui::End();

    Gui::finishRecordingImgui();
  }

  m_pCameraController->tick(frame.deltaTime);
  const Camera& camera = m_pCameraController->getCamera();

  const glm::mat4& projection = camera.getProjection();

  const InputManager::MousePos& mPos =
      app.getInputManager().getCurrentMousePos();

  uint32_t prevInputMask = m_inputMask;
  m_inputMask = app.getInputManager().getCurrentInputMask();

  GlobalUniforms globalUniforms;
  globalUniforms.projection = camera.getProjection();
  globalUniforms.inverseProjection = glm::inverse(globalUniforms.projection);
  globalUniforms.view = camera.computeView();
  globalUniforms.inverseView = glm::inverse(globalUniforms.view);
  globalUniforms.lightCount = static_cast<int>(m_pointLights.getCount());
  globalUniforms.lightBufferHandle =
      m_pointLights.getCurrentLightBufferHandle(frame).index;
  globalUniforms.time = static_cast<float>(frame.currentTime);
  globalUniforms.exposure = m_exposure;
  globalUniforms.mouseUV.x =
      static_cast<float>(mPos.x / app.getSwapChainExtent().width);
  globalUniforms.mouseUV.y =
      static_cast<float>(mPos.y / app.getSwapChainExtent().height);
  globalUniforms.inputMask = m_inputMask;

  m_globalUniforms.getCurrentUniformBuffer(frame).updateUniforms(
      globalUniforms);

  for (uint32_t i = 0; i < m_pointLights.getCount(); ++i) {
    PointLight light = m_pointLights.getLight(i);

    light.position = 40.0f * glm::vec3(
                                 static_cast<float>(i / 3),
                                 -0.1f,
                                 (static_cast<float>(i % 3) - 1.5f) * 0.5f);

    light.position.x += 5.5f * cos(1.5f * frame.currentTime + i);
    light.position.z += 5.5f * sin(1.5 * frame.currentTime + i);

    m_pointLights.setLight(i, light);
  }

  m_pointLights.updateResource(frame);

  glm::vec2 mouseNdc = 2.0f * globalUniforms.mouseUV - glm::vec2(1.0f);
  glm::vec4 scrPosWorld = globalUniforms.inverseView *
                          globalUniforms.inverseProjection *
                          glm::vec4(mouseNdc, 0.0f, 1.0f);
  glm::vec3 camPos(globalUniforms.inverseView[3]);
  glm::vec3 cursorDir =
      glm::normalize(glm::vec3(scrPosWorld) / scrPosWorld.w - camPos);
  m_skeletonEditor.update(m_debugScene, camPos, cursorDir, prevInputMask, m_inputMask);
  m_debugScene.update(frame);
}

void RibCage::_createModels(
    Application& app,
    SingleTimeCommandBuffer& commandBuffer) {

  m_models.emplace_back(
      app,
      commandBuffer,
      GEngineDirectory + "/Content/Models/DamagedHelmet.glb");
  m_models.back().setModelTransform(glm::scale(
      glm::translate(glm::mat4(1.0f), glm::vec3(36.0f, 0.0f, 0.0f)),
      glm::vec3(4.0f)));

  m_models.emplace_back(
      app,
      commandBuffer,
      GEngineDirectory + "/Content/Models/FlightHelmet/FlightHelmet.gltf");
  m_models.back().setModelTransform(glm::scale(
      glm::translate(glm::mat4(1.0f), glm::vec3(50.0f, -1.0f, 0.0f)),
      glm::vec3(8.0f)));

  m_models.emplace_back(
      app,
      commandBuffer,
      GEngineDirectory + "/Content/Models/MetalRoughSpheres.glb");
  m_models.back().setModelTransform(glm::scale(
      glm::translate(glm::mat4(1.0f), glm::vec3(10.0f, 0.0f, 0.0f)),
      glm::vec3(4.0f)));

  // m_models.emplace_back(
  //     app,
  //     commandBuffer,
  //     GEngineDirectory + "/Content/Models/Sponza/glTF/Sponza.gltf");
  // m_models.back().setModelTransform(glm::translate(
  //     glm::scale(glm::mat4(1.0f), glm::vec3(10.0f)),
  //     glm::vec3(10.0f, -1.0f, 0.0f)));
}

void RibCage::_createGlobalResources(
    Application& app,
    SingleTimeCommandBuffer& commandBuffer) {
  m_globalHeap = GlobalHeap(app);
  m_globalUniforms = GlobalUniformsResource(app, m_globalHeap);

  // Create GLTF resource heaps
  {
    _createModels(app, commandBuffer);

    for (Model& model : m_models) {
      model.registerToHeap(m_globalHeap);
    }

    uint32_t primCount = 0;
    for (const Model& model : m_models)
      primCount += static_cast<uint32_t>(model.getPrimitivesCount());

    m_primitiveConstantsBuffer =
        StructuredBuffer<PrimitiveConstants>(app, primCount);

    for (const Model& model : m_models) {
      for (const Primitive& primitive : model.getPrimitives()) {
        m_primitiveConstantsBuffer.setElement(
            primitive.getConstants(),
            primitive.getPrimitiveIndex());
      }
    }

    // The primitive constant buffers contain all the bindless
    // indices for the primitive texture resources
    m_primitiveConstantsBuffer.upload(app, commandBuffer);
    m_primitiveConstantsBuffer.registerToHeap(m_globalHeap);

    // m_textureHeap = TextureHeap(m_models);
  }

  // Global resources
  {
    m_pointLights = PointLightCollection(
        app,
        commandBuffer,
        m_globalHeap,
        9,
        true,
        m_primitiveConstantsBuffer.getHandle());
    for (uint32_t i = 0; i < 3; ++i) {
      for (uint32_t j = 0; j < 3; ++j) {
        PointLight light;
        float t = static_cast<float>(i * 3 + j);

        light.position = 40.0f * glm::vec3(
                                     static_cast<float>(i),
                                     -0.1f,
                                     (static_cast<float>(j) - 1.5f) * 0.5f);
        light.emission =
            1000.0f * // / static_cast<float>(i + 1) *
            glm::vec3(cos(t) + 1.0f, sin(t + 1.0f) + 1.0f, sin(t) + 1.0f);

        m_pointLights.setLight(i * 3 + j, light);
      }
    }
  }

  m_globalResources = GlobalResources(
      app,
      commandBuffer,
      m_globalHeap,
      m_pointLights.getShadowMapHandle(),
      m_primitiveConstantsBuffer.getHandle());

  // Set up SSR resources
  m_SSR = ScreenSpaceReflection(
      app,
      commandBuffer,
      m_globalHeap.getDescriptorSetLayout());
  m_SSR.getReflectionBuffer().registerToHeap(m_globalHeap);

  m_debugScene = SelectableScene(app, commandBuffer);
}

void RibCage::_createForwardPass(Application& app) {
  std::vector<SubpassBuilder> subpassBuilders;

  //  FORWARD GLTF PASS
  {
    SubpassBuilder& subpassBuilder = subpassBuilders.emplace_back();
    // The GBuffer contains the following color attachments
    // 1. Position
    // 2. Normal
    // 3. Albedo
    // 4. Metallic-Roughness-Occlusion
    subpassBuilder.colorAttachments = {0, 1, 2, 3};
    subpassBuilder.depthAttachment = 4;

    Primitive::buildPipeline(subpassBuilder.pipelineBuilder);

    ShaderDefines defs;
    defs.emplace("BINDLESS_SET", "0");

    subpassBuilder
        .pipelineBuilder
        // Vertex shader
        .addVertexShader(
            GEngineDirectory + "/Shaders/GltfForwardBindless.vert",
            defs)
        // Fragment shader
        .addFragmentShader(
            GEngineDirectory + "/Shaders/GltfForwardBindless.frag",
            defs)

        // Pipeline resource layouts
        .layoutBuilder
        // Global resources (view, projection, environment map)
        .addDescriptorSet(m_globalHeap.getDescriptorSetLayout())
        .addPushConstants<ForwardPassPushConstants>(VK_SHADER_STAGE_ALL);
  }

  {
    SubpassBuilder& subpassBuilder = subpassBuilders.emplace_back();

    // The GBuffer contains the following color attachments
    // 1. Position
    // 2. Normal
    // 3. Albedo
    // 4. Metallic-Roughness-Occlusion
    subpassBuilder.colorAttachments = {0, 1, 2, 3};
    subpassBuilder.depthAttachment = 4;

    SelectableScene::buildPipeline(
        subpassBuilder.pipelineBuilder,
        m_globalHeap.getDescriptorSetLayout());
  }

  const GBufferResources& gBuffer = m_globalResources.getGBuffer();
  std::vector<Attachment> attachments = gBuffer.getAttachmentDescriptions();
  const VkExtent2D& extent = app.getSwapChainExtent();
  m_forwardPass = RenderPass(
      app,
      extent,
      std::move(attachments),
      std::move(subpassBuilders));

  m_forwardFrameBuffer =
      FrameBuffer(app, m_forwardPass, extent, gBuffer.getAttachmentViews());
}

void RibCage::_createDeferredPass(Application& app) {
  VkClearValue colorClear;
  colorClear.color = {{0.0f, 0.0f, 0.0f, 1.0f}};
  VkClearValue depthClear;
  depthClear.depthStencil = {1.0f, 0};

  std::vector<Attachment> attachments = {
      Attachment{
          ATTACHMENT_FLAG_COLOR,
          app.getSwapChainImageFormat(),
          colorClear,
          false, // forPresent is false since the imGUI pass follows the
                 // deferred pass
          false,
          true},

      // Depth buffer
      Attachment{
          ATTACHMENT_FLAG_DEPTH,
          app.getDepthImageFormat(),
          depthClear,
          false,
          true,
          true}};

  std::vector<SubpassBuilder> subpassBuilders;

  // DEFERRED PBR PASS
  {
    SubpassBuilder& subpassBuilder = subpassBuilders.emplace_back();
    subpassBuilder.colorAttachments.push_back(0);

    ShaderDefines defs;
    defs.emplace("BINDLESS_SET", "0");

    subpassBuilder.pipelineBuilder.setCullMode(VK_CULL_MODE_FRONT_BIT)
        .setDepthTesting(false)

        // Vertex shader
        .addVertexShader(GProjectDirectory + "/Shaders/DeferredPass.vert", defs)
        // Fragment shader
        .addFragmentShader(
            GProjectDirectory + "/Shaders/DeferredPass.frag",
            defs)

        // Pipeline resource layouts
        .layoutBuilder.addDescriptorSet(m_globalHeap.getDescriptorSetLayout())
        .addPushConstants<DeferredPassPushConstants>(VK_SHADER_STAGE_ALL);
  }

  // SHOW POINT LIGHTS (kinda hacky)
  // TODO: Really light objects should be rendered in the forward
  // pass as well and an emissive channel should be added to the
  // G-Buffer
  m_pointLights.setupPointLightMeshSubpass(
      subpassBuilders.emplace_back(),
      0,
      1,
      m_globalHeap.getDescriptorSetLayout());

  m_deferredPass = RenderPass(
      app,
      app.getSwapChainExtent(),
      std::move(attachments),
      std::move(subpassBuilders));

  m_swapChainFrameBuffers = SwapChainFrameBufferCollection(
      app,
      m_deferredPass,
      {app.getDepthImageView()});
}

namespace {
struct DrawableEnvMap {
  void draw(const DrawContext& context) const {
    context.bindDescriptorSets();
    context.draw(3);
  }
};
} // namespace

void RibCage::draw(
    Application& app,
    VkCommandBuffer commandBuffer,
    const FrameContext& frame) {

  m_pointLights.updateResource(frame);
  m_globalResources.getGBuffer().transitionToAttachment(commandBuffer);

  VkDescriptorSet heapDescriptorSet = m_globalHeap.getDescriptorSet();

  // Draw point light shadow maps
  m_pointLights.drawShadowMaps(
      app,
      commandBuffer,
      frame,
      m_models,
      heapDescriptorSet,
      m_globalResources.getHandle());

  // Forward pass
  {
    ForwardPassPushConstants push{};
    push.globalResources = m_globalResources.getConstants().getHandle().index;
    push.globalUniforms =
        m_globalUniforms.getCurrentBindlessHandle(frame).index;

    ActiveRenderPass pass =
        m_forwardPass.begin(app, commandBuffer, frame, m_forwardFrameBuffer);
    // Bind global descriptor sets
    pass.setGlobalDescriptorSets(gsl::span(&heapDescriptorSet, 1));
    pass.getDrawContext().bindDescriptorSets();

    // Draw models
    for (const Model& model : m_models) {
      for (const Primitive& primitive : model.getPrimitives()) {
        push.model = primitive.computeWorldTransform();
        push.primitiveIdx =
            static_cast<uint32_t>(primitive.getPrimitiveIndex());

        pass.getDrawContext().setFrontFaceDynamic(primitive.getFrontFace());
        pass.getDrawContext().updatePushConstants(push, 0);
        pass.getDrawContext().drawIndexed(
            primitive.getVertexBuffer(),
            primitive.getIndexBuffer());
      }
    }

    pass.nextSubpass();

    m_debugScene.drawSubpass(
        pass.getDrawContext(),
        m_globalUniforms.getCurrentBindlessHandle(frame));
  }

  m_globalResources.getGBuffer().transitionToTextures(commandBuffer);

  // Reflection buffer and convolution
  {
    m_SSR.captureReflection(
        app,
        commandBuffer,
        heapDescriptorSet,
        frame,
        m_globalUniforms.getCurrentBindlessHandle(frame),
        m_globalResources.getHandle());
    m_SSR.convolveReflectionBuffer(app, commandBuffer, frame);
  }

  // Deferred pass
  {
    DeferredPassPushConstants push{};
    push.globalResources = m_globalResources.getHandle().index;
    push.globalUniforms =
        m_globalUniforms.getCurrentBindlessHandle(frame).index;
    push.reflectionBuffer = m_SSR.getReflectionBuffer().getHandle().index;

    ActiveRenderPass pass = m_deferredPass.begin(
        app,
        commandBuffer,
        frame,
        m_swapChainFrameBuffers.getCurrentFrameBuffer(frame));
    // Bind global descriptor sets
    pass.setGlobalDescriptorSets(gsl::span(&heapDescriptorSet, 1));
    pass.getDrawContext().updatePushConstants(push, 0);

    {
      const DrawContext& context = pass.getDrawContext();
      context.bindDescriptorSets();
      context.draw(3);
    }

    pass.nextSubpass();
    pass.setGlobalDescriptorSets(gsl::span(&heapDescriptorSet, 1));
    m_pointLights.draw(
        pass.getDrawContext(),
        m_globalUniforms.getCurrentBindlessHandle(frame));
  }

  Gui::draw(app, frame, commandBuffer);
}
} // namespace RibCage