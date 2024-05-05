#include "ClothSim.h"

#include <Althea/DeferredRendering.h>
#include <Althea/Gui.h>
#include <glm/gtc/constants.hpp>

using namespace AltheaEngine;

#define MAX_NODES 10000 // 100x100
#define MAX_DISTANCE_CONSTRAINTS 100000

#define SHEET_WIDTH 10

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
    m_nodePositions = DynamicVertexBuffer<glm::vec3>(app, MAX_NODES, true);
    m_nodePositions.registerToHeap(heap);

    m_nodeFlags = DynamicVertexBuffer<uint32_t>(app, MAX_NODES, true);
    m_nodeFlags.registerToHeap(heap);

    // TODO: Should this be in a dynamic vert buffer as well...
    m_prevPositions.resize(MAX_NODES);
  }

  // Set up cloth section
  {
    float cellSize = 1.0f;
    float cellDiagonal = cellSize * glm::sqrt(2.0f);

    uint32_t width = SHEET_WIDTH;

    for (uint32_t i = 0; i < width; ++i) {
      for (uint32_t j = 0; j < width; ++j) {
        uint32_t flatIdx = i * width + j;
        glm::vec3 position = glm::vec3(i * cellSize, 0.0f, j * cellSize);

        m_nodePositions.setVertex(position, flatIdx);
        m_prevPositions[flatIdx] = position;
      }
    }

    m_positionConstraints.resize(2 * width);
    m_clothTop = gsl::span(&m_positionConstraints[0], width);
    for (uint32_t i = 0; i < width; ++i) {
      FixedPositionConstraint& c = m_clothTop[i];
      c.nodeIdx = i * width;
      c.position = m_prevPositions[c.nodeIdx];
    }

    m_clothBottom = gsl::span(&m_positionConstraints[width], width);
    for (uint32_t i = 0; i < width; ++i) {
      FixedPositionConstraint& c = m_clothBottom[i];
      c.nodeIdx = i * width + width - 1;
      c.position = m_prevPositions[c.nodeIdx];
    }

    m_distanceConstraints.reserve(MAX_DISTANCE_CONSTRAINTS);

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
          m_distanceConstraints.push_back(
              DistanceConstraint{idx0, idx1, cellSize, 0.0f});
        if (j == 0)
          m_distanceConstraints.push_back(
              DistanceConstraint{idx0, idx3, cellSize, 0.0f});

        m_distanceConstraints.push_back(
            DistanceConstraint{idx1, idx2, cellSize, 0.0f});
        m_distanceConstraints.push_back(
            DistanceConstraint{idx2, idx3, cellSize, 0.0f});

        m_distanceConstraints.push_back(
            DistanceConstraint{idx0, idx2, cellDiagonal, 0.0f});
        m_distanceConstraints.push_back(
            DistanceConstraint{idx1, idx3, cellDiagonal, 0.0f});
      }
    }

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
static int s_stepFrameCounter = 0;

static bool s_bDebugColoring = false;

static int s_solverSubsteps = 1;
static float s_maxSpeed = 0.1f;
static float s_damping = 0.02f;
static float s_k = 0.125f;
static float s_collisionStrength = 1.0f;
static float s_gravity = 1.0f;

static bool s_resolveCollisions = false; // true;
static int s_collisionIterations = 3;

static bool s_fixTop = true;
static bool s_fixBottom = true;
static float s_twist = 0.0f;

void ClothSim::_updateConstraints() {
  uint32_t width = SHEET_WIDTH;
  float cellSize = 1.0f;

  // Fixed bottom row
  glm::vec3 targetCenter(0.5f * width * cellSize, 0.0f, (width - 1) * cellSize);
  float theta = s_twist * 2.0f * glm::pi<float>();
  float cosTheta = glm::cos(theta);
  float sinTheta = glm::sin(theta);

  for (uint32_t i = 0; i < width; ++i) {
    FixedPositionConstraint& c = m_clothBottom[i];
    float x = i * cellSize - 0.5f * width;
    c.position = targetCenter + glm::vec3(cosTheta * x, sinTheta * x, 0.0f);
  }
}

void ClothSim::_pgsSolve() {
  const std::vector<uint32_t>& indices =
      m_clothSections[0].indices.getIndices();

  for (uint32_t solverStep = 0; solverStep < s_solverSubsteps; ++solverStep) {
    if (s_fixTop) {
      for (const FixedPositionConstraint& c : m_clothTop) {
        glm::vec3& pos = m_nodePositions.getVertex(c.nodeIdx);
        glm::vec3 diff = c.position - pos;
        pos += s_k * diff;
      }
    }

    if (s_fixBottom) {
      for (const FixedPositionConstraint& c : m_clothBottom) {
        glm::vec3& pos = m_nodePositions.getVertex(c.nodeIdx);
        glm::vec3 diff = c.position - pos;
        pos += s_k * diff;
      }
    }

    for (const DistanceConstraint& c : m_distanceConstraints) {
      glm::vec3& p0 = m_nodePositions.getVertex(c.a);
      glm::vec3& p1 = m_nodePositions.getVertex(c.b);

      glm::vec3 diff = p1 - p0;
      float dist = glm::length(diff);
      diff /= dist;

      // if (dist < c.restLength && dist > 0.5f * c.restLength)
      //   continue;

      if (dist < 0.000001f)
        diff = glm::vec3(1.0f, 0.0f, 0.0f);

      glm::vec3 disp = 0.5f * s_k * (c.restLength - dist) * diff;
      p0 -= disp;
      p1 += disp;
    }

    // Collisions
    if (s_resolveCollisions) {

      float thresholdDistance = m_collisions.getThresholdDistance();

      for (int colIter = 0; colIter < s_collisionIterations; ++colIter) {
        for (const PointTriangleCollision& col :
             m_collisions.getCollisions().getTriangleCollisions()) {
          glm::vec3& a =
              m_nodePositions.getVertex(indices[3 * col.triangleIdx + 0]);
          glm::vec3& b =
              m_nodePositions.getVertex(indices[3 * col.triangleIdx + 1]);
          glm::vec3& c =
              m_nodePositions.getVertex(indices[3 * col.triangleIdx + 2]);
          glm::vec3& p = m_nodePositions.getVertex(col.pointIdx);

          glm::vec3 ab = b - a;
          glm::vec3 ac = c - a;
          glm::vec3 ap = p - a;

          // TODO: handle degenerate case

          glm::vec3 n = glm::normalize(glm::cross(ab, ac));
          if (col.bBackFace)
            n = -n;

          float d = glm::dot(ap, n);
          if (d < thresholdDistance) {
            glm::vec3 diff =
                0.25f * s_collisionStrength * (thresholdDistance - d) * n;

            p += 3.0f * diff;
            a -= diff;
            b -= diff;
            c -= diff;
          }
        }

        for (const EdgeCollision& col :
             m_collisions.getCollisions().getEdgeCollisions()) {
          glm::vec3& a = m_nodePositions.getVertex(
              indices[3 * col.triangleAIdx + col.edgeAIdx]);
          glm::vec3& b = m_nodePositions.getVertex(
              indices[3 * col.triangleAIdx + (col.edgeAIdx + 1) % 3]);
          glm::vec3& c = m_nodePositions.getVertex(
              indices[3 * col.triangleBIdx + col.edgeBIdx]);
          glm::vec3& d = m_nodePositions.getVertex(
              indices[3 * col.triangleBIdx + (col.edgeBIdx + 1) % 3]);

          glm::vec3 sep = glm::mix(c, d, col.v) - glm::mix(a, b, col.u);

          float sepDist = glm::dot(sep, col.normal);

          if (sepDist < thresholdDistance) {
            glm::vec3 diff = 0.5f * s_collisionStrength *
                             (thresholdDistance - sepDist) * col.normal;
            a -= diff * (1.0f - col.u);
            b -= diff * col.u;
            c += diff * (1.0f - col.v);
            d += diff * col.v;
          }
        }
      }
    }
  }
}

void ClothSim::update(const FrameContext& frame) {
  const std::vector<uint32_t>& indices =
      m_clothSections[0].indices.getIndices();

  uint32_t width = SHEET_WIDTH;
  float cellSize = 1.0f;

  _updateConstraints();

  // Iterative solver
  if (!s_simPaused || s_stepFrameCounter > 0) {
    if (s_stepFrameCounter > 0)
      --s_stepFrameCounter;

    // TODO: Sim solver step
    float dt = 1.0f / 30.0f;

    // TODO: time substeps
    // Apply gravity
    glm::vec3 gravity(0.0f, -s_gravity, 0.0f);
    for (uint32_t nodeIdx = 0; nodeIdx < m_nodePositions.getVertexCount();
         ++nodeIdx) {
      glm::vec3& pos = m_nodePositions.getVertex(nodeIdx);
      glm::vec3 velDt = pos - m_prevPositions[nodeIdx];
      float speed = glm::length(velDt);
      if (speed > s_maxSpeed)
        velDt *= s_maxSpeed / speed;

      m_prevPositions[nodeIdx] = pos;

      pos += 0.5f * gravity * dt * dt + velDt * (1.0f - s_damping);
    }

    _pgsSolve();
  }

  m_nodePositions.upload(frame.frameRingBufferIndex);

  // Fix-up AABB
  {
    const std::vector<glm::vec3>& positions = m_nodePositions.getVertices();

    m_aabb.update(indices, positions, m_prevPositions, frame);
    m_collisions
        .update(frame, indices, positions, m_prevPositions, m_aabb.getTree());

    for (uint32_t i = 0; i < MAX_NODES; ++i)
      m_nodeFlags.setVertex(0, i);

    if (s_bDebugColoring) {
      for (const PointTriangleCollision& c :
           m_collisions.getCollisions().getTriangleCollisions()) {
        m_nodeFlags.setVertex(1, c.pointIdx);

        m_nodeFlags.setVertex(1, indices[3 * c.triangleIdx + 0]);
        m_nodeFlags.setVertex(1, indices[3 * c.triangleIdx + 1]);
        m_nodeFlags.setVertex(1, indices[3 * c.triangleIdx + 2]);
      }
    }

    m_nodeFlags.upload(frame.frameRingBufferIndex);
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

    uniforms.nodesCount = MAX_NODES;
    uniforms.nodePositions =
        m_nodePositions.getCurrentBufferHandle(frame.frameRingBufferIndex)
            .index;
    uniforms.nodeFlags =
        m_nodeFlags.getCurrentBufferHandle(frame.frameRingBufferIndex).index;

    uniforms.distanceConstraints = INVALID_BINDLESS_HANDLE;
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
    ImGui::Indent();
    ImGui::Text("Paused:");
    ImGui::Checkbox("##paused", &s_simPaused);
    ImGui::Text("Step Frames:");
    if (ImGui::Button(">"))
      s_stepFrameCounter = 1;
    ImGui::SameLine();
    if (ImGui::Button(">>"))
      s_stepFrameCounter = 8;
    ImGui::SameLine();
    if (ImGui::Button(">>>"))
      s_stepFrameCounter = 30;

    ImGui::Text("Debug Coloring:");
    ImGui::SameLine();
    ImGui::Checkbox("##debugcoloring", &s_bDebugColoring);

    ImGui::Text("Resolve Collisions:");
    ImGui::Checkbox("##resolvecollisions", &s_resolveCollisions);

    if (ImGui::CollapsingHeader(
            "Constraints",
            ImGuiTreeNodeFlags_DefaultOpen)) {
      ImGui::Indent();

      ImGui::Text("Twist Percentage:");
      ImGui::SliderFloat("##twistamt", &s_twist, 0.0f, 1.0f);

      ImGui::Text("Fix Top:");
      ImGui::SameLine();
      ImGui::Checkbox("##fixtop", &s_fixTop);

      ImGui::Text("Fix Bottom:");
      ImGui::SameLine();
      ImGui::Checkbox("##fixbottom", &s_fixBottom);

      ImGui::Unindent();
    }

    if (ImGui::CollapsingHeader("Iterations")) {
      ImGui::Indent();

      ImGui::Text("Substeps:");
      ImGui::SliderInt("##substeps", &s_solverSubsteps, 1, 8);
      ImGui::Text("Collision Iterations:");
      ImGui::SliderInt("##colIters", &s_collisionIterations, 1, 8);

      ImGui::Unindent();
    }

    if (ImGui::CollapsingHeader("Params")) {
      ImGui::Indent();

      ImGui::Text("Collision Strength:");
      ImGui::SliderFloat("##colstrength", &s_collisionStrength, 0.0f, 1.0f);
      ImGui::Text("Max Speed:");
      ImGui::SliderFloat("##maxSpeed", &s_maxSpeed, 0.01f, 0.5f);
      ImGui::Text("Damping:");
      ImGui::SliderFloat("##damping", &s_damping, 0.0f, 1.0f);
      ImGui::Text("K:");
      ImGui::SliderFloat("##springstrength", &s_k, 0.01f, 1.0f);
      ImGui::Text("Gravity:");
      ImGui::SliderFloat("##gravity", &s_gravity, 0.25f, 4.0f);

      ImGui::Unindent();
    }

    ImGui::Unindent();
  }

  m_aabb.updateUI();
  m_collisions.updateUI();
}
} // namespace RibCage