#include "RibCage.h"

#include "GltfTestScene.h"

#include <Althea/Application.h>
#include <Althea/Camera.h>
#include <Althea/Cubemap.h>
#include <Althea/DescriptorSet.h>
#include <Althea/GraphicsPipeline.h>
#include <Althea/Gui.h>
#include <Althea/InputManager.h>
#include <Althea/InputMask.h>
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

  m_orbitCamera = OrbitCamera(
      glm::vec3(0.0f),
      5.0f,
      90.0f,
      (float)windowDims.width / (float)windowDims.height);

  // TODO: need to unbind these at shutdown
  InputManager& input = app.getInputManager();
  input.setMouseCursorHidden(false);

  // Recreate any stale pipelines (shader hot-reload)
  input.addKeyBinding(
      {GLFW_KEY_R, GLFW_PRESS, GLFW_MOD_CONTROL},
      [&app, that = this]() {
        that->m_deferredPass.tryRecompile(app);
        that->m_debugScene.tryRecompileShaders(app);
        that->m_skeletonEditor.tryRecompileShaders(app);

        that->m_gBufferPass.tryRecompileShaders(app);

        for (auto& sceneElem : that->m_sceneElements)
          sceneElem->tryRecompileShaders(app);
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
  m_orbitCamera.getCamera().setAspectRatio(
      (float)extent.width / (float)extent.height);

  Gui::createRenderState(app);

  SingleTimeCommandBuffer commandBuffer(app);

  _createGlobalResources(app, commandBuffer);
  _createDeferredPass(app);
}

void RibCage::destroyRenderState(Application& app) {
  Gui::destroyRenderState(app);

  m_gBufferPass = {};

  m_deferredPass = {};
  m_swapChainFrameBuffers = {};

  m_SSR = {};
  m_globalUniforms = {};

  m_globalResources = {};
  m_globalHeap = {};

  m_skeletonEditor = {};
  m_debugScene = {};

  m_sceneElements.clear();
}

static int s_cameraMode = 1;

void RibCage::tick(Application& app, const FrameContext& frame) {
  {
    Gui::startRecordingImgui();

    if (!app.getInputManager().getMouseCursorHidden()) {
      const ImGuiViewport* main_viewport = ImGui::GetMainViewport();
      ImGui::SetNextWindowPos(
          ImVec2(main_viewport->WorkPos.x + 650, main_viewport->WorkPos.y + 20),
          ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(220, 100), ImGuiCond_FirstUseEver);

      if (ImGui::Begin("Rib Cage")) {
        // m_skeletonEditor.updateUI();
        for (auto& sceneElem : m_sceneElements)
          sceneElem->updateUI();

        if (ImGui::CollapsingHeader("Camera")) {
          static const char* cameraModes[] = {"Orbit", "Free"};
          ImGui::Text("Modes:");
          if (ImGui::Combo("##cameraModes", &s_cameraMode, cameraModes, 2)) {
            if (s_cameraMode == 0) {
              const Camera& freeCam = m_pCameraController->getCamera();
              const glm::mat4& freeCamTransform = freeCam.getTransform();
              m_orbitCamera.setTargetPosition(
                  glm::vec3(freeCamTransform[3]) +
                  m_orbitCamera.getSpacing() * glm::vec3(freeCamTransform[2]));

              m_orbitCamera.setRotation(
                  freeCam.computeYaw(),
                  freeCam.computePitch());
            } else {
              const Camera& orbitCam = m_orbitCamera.getCamera();
              const glm::mat4& orbitCamTransform = orbitCam.getTransform();
              m_pCameraController->getCamera().setPosition(
                  glm::vec3(orbitCamTransform[3]));
              m_pCameraController->getCamera().setRotationRadians(
                  orbitCam.computeYaw(),
                  orbitCam.computePitch());
            }
          }
        }
      }

      ImGui::End();
    }

    Gui::finishRecordingImgui();
  }

  // m_pCameraController->tick(frame.deltaTime);
  // const Camera& camera = m_pCameraController->getCamera();

  // const glm::mat4& projection = camera.getProjection();

  const InputManager::MousePos& mPos =
      app.getInputManager().getCurrentMousePos();

  uint32_t prevInputMask = m_inputMask;
  m_inputMask = app.getInputManager().getCurrentInputMask();

  if (m_inputMask & INPUT_BIT_SPACE && m_debugScene.isGizmoEnabled())
    m_orbitCamera.setTargetPosition(m_debugScene.getGizmoPosition());

  if (s_cameraMode == 0)
    m_orbitCamera.tick(frame.deltaTime, prevInputMask, m_inputMask);
  else
    m_pCameraController->tick(frame.deltaTime);

  const Camera& camera = s_cameraMode == 0 ? m_orbitCamera.getCamera()
                                           : m_pCameraController->getCamera();

  GlobalUniforms globalUniforms;
  globalUniforms.projection = camera.getProjection();
  globalUniforms.inverseProjection = glm::inverse(globalUniforms.projection);
  globalUniforms.view = camera.computeView();
  globalUniforms.inverseView = glm::inverse(globalUniforms.view);
  globalUniforms.lightCount = 0;
  globalUniforms.lightBufferHandle = INVALID_BINDLESS_HANDLE;
  globalUniforms.time = static_cast<float>(frame.currentTime);
  globalUniforms.exposure = m_exposure;
  globalUniforms.mouseUV.x =
      static_cast<float>(mPos.x / app.getSwapChainExtent().width);
  globalUniforms.mouseUV.y =
      static_cast<float>(mPos.y / app.getSwapChainExtent().height);
  globalUniforms.inputMask = m_inputMask;

  m_globalUniforms.getCurrentUniformBuffer(frame).updateUniforms(
      globalUniforms);

  glm::vec2 mouseNdc = 2.0f * globalUniforms.mouseUV - glm::vec2(1.0f);
  glm::vec4 scrPosWorld = globalUniforms.inverseView *
                          globalUniforms.inverseProjection *
                          glm::vec4(mouseNdc, 0.0f, 1.0f);
  glm::vec3 camPos(globalUniforms.inverseView[3]);
  glm::vec3 cursorDir =
      glm::normalize(glm::vec3(scrPosWorld) / scrPosWorld.w - camPos);
  m_skeletonEditor
      .update(m_debugScene, camPos, cursorDir, prevInputMask, m_inputMask);
  m_debugScene.update(frame);

  for (auto& sceneElem : m_sceneElements)
    sceneElem->update(frame);
}

void RibCage::_createGlobalResources(
    Application& app,
    SingleTimeCommandBuffer& commandBuffer) {
  m_globalHeap = GlobalHeap(app);
  m_globalUniforms = GlobalUniformsResource(app, m_globalHeap);

  m_globalResources = GlobalResources(app, commandBuffer, m_globalHeap, {}, {});

  // Set up SSR resources
  m_SSR = ScreenSpaceReflection(
      app,
      commandBuffer,
      m_globalHeap.getDescriptorSetLayout());
  m_SSR.getReflectionBuffer().registerToHeap(m_globalHeap);

  m_debugScene = SelectableScene(
      app,
      m_globalHeap,
      commandBuffer,
      m_globalResources.getGBuffer());

  m_skeletonEditor = SkeletonEditor(
      app,
      commandBuffer,
      m_globalHeap,
      m_globalResources.getGBuffer());

  m_sceneElements.emplace_back(makeIntrusive<GltfTestScene>());
  // m_sceneElements.emplace_back(makeIntrusive<ClothSim>());
  // m_sceneElements.emplace_back(makeIntrusive<ObjTestScene>());

  SceneToGBufferPassBuilder gBufferPassBuilder{};

  for (auto& sceneElem : m_sceneElements)
    sceneElem->init(app, commandBuffer, gBufferPassBuilder, m_globalHeap);

  m_gBufferPass = SceneToGBufferPass(
      app,
      m_globalResources.getGBuffer(),
      m_globalHeap.getDescriptorSetLayout(),
      std::move(gBufferPassBuilder));
}

void RibCage::_createDeferredPass(Application& app) {
  VkClearValue colorClear;
  colorClear.color = {{0.0f, 0.0f, 0.0f, 1.0f}};

  std::vector<Attachment> attachments = {Attachment{
      ATTACHMENT_FLAG_COLOR,
      app.getSwapChainImageFormat(),
      colorClear,
      false, // forPresent is false since the imGUI pass follows the
             // deferred pass
      false,
      true}};

  std::vector<SubpassBuilder> subpassBuilders;

  // DEFERRED PBR PASS
  {
    SubpassBuilder& subpassBuilder = subpassBuilders.emplace_back();
    subpassBuilder.colorAttachments.push_back(0);

    subpassBuilder.pipelineBuilder.setCullMode(VK_CULL_MODE_FRONT_BIT)
        .setDepthTesting(false)

        // Vertex shader
        .addVertexShader(GProjectDirectory + "/Shaders/DeferredPass.vert")
        // Fragment shader
        .addFragmentShader(GProjectDirectory + "/Shaders/DeferredPass.frag")

        // Pipeline resource layouts
        .layoutBuilder.addDescriptorSet(m_globalHeap.getDescriptorSetLayout())
        .addPushConstants<DeferredPassPushConstants>(VK_SHADER_STAGE_ALL);
  }

  m_deferredPass = RenderPass(
      app,
      app.getSwapChainExtent(),
      std::move(attachments),
      std::move(subpassBuilders));

  m_swapChainFrameBuffers =
      SwapChainFrameBufferCollection(app, m_deferredPass, {});
}

void RibCage::draw(
    Application& app,
    VkCommandBuffer commandBuffer,
    const FrameContext& frame) {

  m_globalResources.getGBuffer().transitionToAttachment(commandBuffer);

  VkDescriptorSet heapDescriptorSet = m_globalHeap.getDescriptorSet();

  m_debugScene.draw(
      app,
      commandBuffer,
      frame,
      heapDescriptorSet,
      m_globalUniforms.getCurrentBindlessHandle(frame));
  m_skeletonEditor.draw(
      app,
      commandBuffer,
      frame,
      heapDescriptorSet,
      m_globalUniforms.getCurrentBindlessHandle(frame));

  for (auto& sceneElem : m_sceneElements)
    sceneElem->preDraw(
        app,
        commandBuffer,
        frame,
        heapDescriptorSet,
        m_globalResources.getHandle(),
        m_globalUniforms.getCurrentBindlessHandle(frame));

  m_gBufferPass.begin(
      app,
      commandBuffer,
      frame,
      heapDescriptorSet,
      m_globalResources.getHandle(),
      m_globalUniforms.getCurrentBindlessHandle(frame));

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
  }

  Gui::draw(app, frame, commandBuffer);
}
} // namespace RibCage