#include "HairTestScene.h"

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
#include <Althea/ShapeUtilities.h>
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

#define GEN_SHADER_DEBUG_INFO

extern ::AltheaEngine::InputManager* ::AltheaEngine::GInputManager;

namespace RibCage {

void HairTestScene::registerGBufferSubpass(
    GraphicsPipelineBuilder& builder) const {

  // Render particles
  ShaderDefines defs{};
  builder.setPrimitiveType(PrimitiveType::TRIANGLES)
      .addVertexInputBinding<glm::vec3>(VK_VERTEX_INPUT_RATE_VERTEX)
      .addVertexAttribute(VertexAttributeType::VEC3, 0)
      .addVertexShader(
          GEngineDirectory + "/Shaders/Hair/Particles.vert",
          defs)
      .addFragmentShader(
          GEngineDirectory + "/Shaders/Hair/Particles.frag")
      .layoutBuilder.addPushConstants<PushConstants>();
}

void HairTestScene::tryRecompileShaders(Application& app) {
  for (auto& pipeline : m_computePasses)
    pipeline.tryRecompile(app);
}

void HairTestScene::init(
    Application& app,
    SingleTimeCommandBuffer& commandBuffer,
    SceneToGBufferPassBuilder& gBufferPassBuilder,
    GlobalHeap& heap) {

  GInputManager->addKeyBinding(
      {GLFW_KEY_P, GLFW_RELEASE, 0},
      [that = this, &app]() {
        vkDeviceWaitIdle(app.getDevice());
        that->_resetParticles(app, SingleTimeCommandBuffer(app));
      });

#ifdef GEN_SHADER_DEBUG_INFO
  Shader::setShouldGenerateDebugInfo(true);
#endif

  // TODO: Create LODs for particles
  ShapeUtilities::createSphere(
      app,
      commandBuffer,
      m_sphere.vertices,
      m_sphere.indices,
      6,
      1.0f);

  std::vector<glm::vec3> boxVerts = {
      glm::vec3(0.0f, 0.0f, 0.0f),
      glm::vec3(1.0f, 0.0f, 0.0f),
      glm::vec3(0.0f, 1.0f, 0.0f),
      glm::vec3(1.0f, 1.0f, 0.0f),
      glm::vec3(0.0f, 0.0f, 1.0f),
      glm::vec3(1.0f, 0.0f, 1.0f),
      glm::vec3(0.0f, 1.0f, 1.0f),
      glm::vec3(1.0f, 1.0f, 1.0f),
  };
  m_box.vertices = VertexBuffer<glm::vec3>(
      const_cast<const Application&>(app),
      commandBuffer,
      std::move(boxVerts));

  // clang-format off
  std::vector<uint32_t> boxIndices = {
    // z = 0
    0, 3, 1,
    0, 2, 3,
    // z = 1
    4, 5, 7,
    4, 7, 6,
    // x = 0
    0, 4, 6,
    0, 6, 2,
    // x = 1 
    1, 7, 5,
    1, 3, 7,
    // y = 0
    0, 1, 5,
    0, 5, 4,
    // y = 1
    2, 7, 3,
    2, 6, 7
  };
  // clang-format on
  m_box.indices = IndexBuffer(
      const_cast<const Application&>(app),
      commandBuffer,
      std::move(boxIndices));

  gBufferPassBuilder.registerSubpass(IntrusivePtr(this));
  
  m_floor = makeIntrusive<Floor>();
  gBufferPassBuilder.registerSubpass(m_floor);
  m_floor->m_floorHeight = FLOOR_HEIGHT;
  m_floor->m_floorHalfWidth = 500.0f;

  _createSimResources(app, commandBuffer, heap);
}

void HairTestScene::_createSimResources(
    Application& app,
    SingleTimeCommandBuffer& commandBuffer,
    GlobalHeap& heap) {
  uint32_t particleBufferCount =
      (PARTICLE_COUNT - 1) / PARTICLES_PER_BUFFER + 1;
  m_particleBuffer = StructuredBufferHeap<Particle>(
      app,
      particleBufferCount,
      PARTICLES_PER_BUFFER);
  m_particleBuffer.registerToHeap(heap);

  uint32_t gridBufferCount = (GRID_CELL_COUNT - 1) / GRID_CELLS_PER_BUFFER + 1;
  m_grid = StructuredBufferHeap<GridCell>(
      app,
      gridBufferCount,
      GRID_CELLS_PER_BUFFER);
  m_grid.registerToHeap(heap);

  uint32_t strandBufferCount = (STRANDS_COUNT - 1) / STRANDS_PER_BUFFER + 1;
  m_strandBuffer =
      StructuredBufferHeap<Strand>(app, strandBufferCount, STRANDS_PER_BUFFER);
  m_strandBuffer.registerToHeap(heap);

  _resetParticles(app, commandBuffer);

  m_simUniforms = TransientUniforms<SimUniforms>(app);
  m_simUniforms.registerToHeap(heap);

  for (const PassInfo& info : m_shaderInfos) {
    char fullShaderName[256];
    sprintf(
        fullShaderName,
        "%s/Shaders/%s",
        GEngineDirectory.c_str(),
        info.shaderName);

    ShaderDefines defs{};
    defs.emplace(info.entryPoint, "main");
    ComputePipelineBuilder builder{};
    builder.setComputeShader(fullShaderName, defs);
    builder.layoutBuilder.addDescriptorSet(heap.getDescriptorSetLayout());
    builder.layoutBuilder.addPushConstants<PushConstants>(
        VK_SHADER_STAGE_COMPUTE_BIT);

    m_computePasses.emplace_back(app, std::move(builder));
  }
}

static LiveValues s_liveValues;

void HairTestScene::update(const FrameContext& frame) {
  // Use fixed timestep for physics
  float deltaTime = 1.0f / 15.0f / float(TIME_SUBSTEPS);

  uint32_t inputMask = GInputManager->getCurrentInputMask();

  SimUniforms simUniforms{};

  // TODO: Just use spacing scale param??
  // can assume grid is world axis aligned and uniformly scaled on each dim
  // don't care how many cells there are, due to spatial hash
  simUniforms.gridToWorld =
      glm::scale(glm::mat4(1.0f), glm::vec3(GRID_CELL_WIDTH));
  simUniforms.worldToGrid = glm::inverse(simUniforms.gridToWorld);

  if (m_flagReset) {
    m_flagReset = false;
    simUniforms.addedParticles = m_activeParticleCount;
  } else if (inputMask & INPUT_BIT_RIGHT_MOUSE) {
    simUniforms.addedParticles = 1000;

    m_activeParticleCount += simUniforms.addedParticles;
    if (m_activeParticleCount > PARTICLE_COUNT) {
      simUniforms.addedParticles = PARTICLE_COUNT - m_activeParticleCount;
      m_activeParticleCount = PARTICLE_COUNT;
    }
  } else {
    simUniforms.addedParticles = 0;
  }

  simUniforms.particleCount = m_activeParticleCount;
  simUniforms.time = frame.currentTime;

  simUniforms.particlesHeap = m_particleBuffer.getFirstBufferHandle().index;
  simUniforms.gridHeap = m_grid.getFirstBufferHandle().index;
  simUniforms.strandBuffer = m_strandBuffer.getFirstBufferHandle().index;

  simUniforms.liveValues = s_liveValues;

  m_simUniforms.updateUniforms(simUniforms, frame);
}

void HairTestScene::preDraw(
    const Application& app,
    VkCommandBuffer commandBuffer,
    const FrameContext& frame,
    VkDescriptorSet set,
    BufferHandle globalResourcesHandle,
    UniformHandle globalUniformsHandle) {

  m_push.globalResourcesHandle = globalResourcesHandle.index;
  m_push.globalUniformsHandle = globalUniformsHandle.index;
  m_push.simUniformsHandle = m_simUniforms.getCurrentHandle(frame).index;

  auto dispatchCompute = [&](uint32_t passIdx, uint32_t groupCount) {
    m_computePasses[passIdx].bindPipeline(commandBuffer);
    m_computePasses[passIdx].bindDescriptorSet(commandBuffer, set);
    m_computePasses[passIdx].setPushConstants(commandBuffer, m_push);
    vkCmdDispatch(commandBuffer, groupCount, 1, 1);
  };

  uint32_t gridBufferCount = m_grid.getSize();
  uint32_t particleBufferCount = m_particleBuffer.getSize();

  for (uint32_t substep = 0; substep < TIME_SUBSTEPS; ++substep) {
    // Reset the spatial hash and prepare the buffers for read/write
    {
      m_grid.barrier(
          commandBuffer,
          VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
          VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT,
          VK_ACCESS_TRANSFER_WRITE_BIT,
          VK_PIPELINE_STAGE_TRANSFER_BIT);

      for (uint32_t bufferIdx = 0; bufferIdx < gridBufferCount; ++bufferIdx) {
        const auto& buffer = m_grid.getBuffer(bufferIdx);

        vkCmdFillBuffer(
            commandBuffer,
            buffer.getAllocation().getBuffer(),
            0,
            buffer.getSize(),
            0xFFFFFFFF);
      }

      m_grid.barrier(
          commandBuffer,
          VK_ACCESS_TRANSFER_WRITE_BIT,
          VK_PIPELINE_STAGE_TRANSFER_BIT,
          VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT,
          VK_PIPELINE_STAGE_ALL_GRAPHICS_BIT);
    }

    // Particle simulation
    // - Update particles with new positions
    {
      uint32_t groupCountX = (m_activeParticleCount - 1) / LOCAL_SIZE_X + 1;
      dispatchCompute(ADVECT_PARTICLES_PASS, groupCountX);
      m_particleBuffer.rwBarrier(commandBuffer);
    }

    // Solve strands
    {
      uint32_t groupCountX = (STRANDS_COUNT - 1) / LOCAL_SIZE_X + 1;
      dispatchCompute(FTL_STRANDS_PASS, groupCountX);
      m_particleBuffer.rwBarrier(commandBuffer);
    }

    // Dispatch jacobi iterations for collision resolution
    //   {
    //     uint32_t groupCountX = (m_activeParticleCount - 1) / LOCAL_SIZE_X +
    //     1;

    //     m_computePasses[JACOBI_STEP_PASS].bindPipeline(commandBuffer);
    //     m_computePasses[JACOBI_STEP_PASS].bindDescriptorSet(commandBuffer,
    //     set);

    //     for (uint32_t iter = 0; iter < JACOBI_ITERS; ++iter) {
    //       m_push.iteration = iter;
    //       m_computePasses[JACOBI_STEP_PASS].setPushConstants(
    //           commandBuffer,
    //           m_push);
    //       vkCmdDispatch(commandBuffer, groupCountX, 1, 1);
    //     }
    //   }
    // }
  }
}

void HairTestScene::beginGBufferSubpass(
    const DrawContext& context,
    BufferHandle globalResourcesHandle,
    UniformHandle globalUniformsHandle) {

  uint32_t inputMask = GInputManager->getCurrentInputMask();
  {
    context.bindDescriptorSets();
    context.updatePushConstants(m_push, 0);

    context.bindIndexBuffer(m_sphere.indices);
    context.bindVertexBuffer(m_sphere.vertices);
    context.drawIndexed(
        m_sphere.indices.getIndexCount(),
        m_activeParticleCount);
  }
}

void HairTestScene::updateUI() {
  bool show = true;
  if (show) {
    const ImGuiViewport* main_viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(main_viewport->WorkPos.x + 650, main_viewport->WorkPos.y + 20),
        ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(440, 200), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Live Edit")) {
      ImGui::Text("Slider1:");
      ImGui::SliderFloat("##slider1", &s_liveValues.slider1, 0.0, 1.0);
      ImGui::Text("Slider2:");
      ImGui::SliderFloat("##slider2", &s_liveValues.slider2, 0.0, 1.0);
      ImGui::Text("Checkbox1:");
      ImGui::Checkbox("##checkbox1", &s_liveValues.checkbox1);
      ImGui::Text("Checkbox2:");
      ImGui::Checkbox("##checkbox2", &s_liveValues.checkbox2);
    }
  }

  ImGui::End(); // ??
}

void HairTestScene::_resetParticles(
    Application& app,
    VkCommandBuffer commandBuffer) {
  for (uint32_t particleIdx = 0; particleIdx < PARTICLE_COUNT; ++particleIdx) {
    glm::vec3 position(rand() % 30, rand() % 300, rand() % 30);
    position.y += FLOOR_HEIGHT;

    m_particleBuffer.setElement(
        Particle{// position
                 position,
                 0,
                 position,
                 // debug value
                 0xfc3311},
        particleIdx);
  }

  uint32_t strandParticleCount = 0;
  for (uint32_t strandIdx = 0; strandIdx < STRANDS_COUNT; ++strandIdx) {
    glm::vec3 position(rand() % 30, rand() % 300 + FLOOR_HEIGHT, rand() % 30);

    Strand strand{};
    for (uint32_t i = 0; i < STRAND_PARTICLE_COUNT; ++i) {
      strand.indices[i] = strandParticleCount;
      m_particleBuffer.setElement(
          Particle{// position
                   position,
                   0,
                   position,
                   // debug value
                   0xfc3311},
          strandParticleCount++);

      position.y += STRAND_PARTICLE_SEPARATION;
      position.x += 0.1 * sin(i);
      position.z += 0.1 * cos(i);
    }

    m_strandBuffer.setElement(strand, strandIdx);
  }

  m_activeParticleCount = strandParticleCount;

  m_particleBuffer.upload(app, commandBuffer);
  m_strandBuffer.upload(app, commandBuffer);

  m_flagReset = true;
}

} // namespace RibCage