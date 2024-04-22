#include "Collisions.h"

#include <Althea/Containers/StackVector.h>
#include <Althea/Gui.h>

#include <glm/gtc/matrix_inverse.hpp>

using namespace AltheaEngine;

#define EPSILON 0.00001f

namespace RibCage {
struct LeafCollision {
  uint32_t leafA;
  uint32_t leafB;
};
// TODO: stack-free impl?
static void staticCollisionBroadphase(
    StackVector<const AABBLeaf*>& filteredPairs,
    const AABBLeaf* box,
    const AABBInnerNode* currentNode,
    const AABBTree& aabb) {
  if (currentNode->intersect(box->min, box->max)) {
    if (currentNode->flags & InnerNodeFlag_HasLeafChildren) {
      if (aabb.hasChildA(currentNode)) {
        const AABBLeaf* leaf = aabb.getLeafA(currentNode);
        if (leaf != box && leaf->intersect(box->min, box->max))
          filteredPairs.push_back(leaf);
      }

      if (aabb.hasChildB(currentNode)) {
        const AABBLeaf* leaf = aabb.getLeafB(currentNode);
        if (leaf != box && leaf->intersect(box->min, box->max))
          filteredPairs.push_back(leaf);
      }
    } else {
      if (aabb.hasChildA(currentNode)) {
        const AABBInnerNode* nextNode = aabb.getChildA(currentNode);
        if (nextNode->intersect(box->min, box->max))
          staticCollisionBroadphase(filteredPairs, box, nextNode, aabb);
      }

      if (aabb.hasChildB(currentNode)) {
        const AABBInnerNode* nextNode = aabb.getChildB(currentNode);
        if (nextNode->intersect(box->min, box->max))
          staticCollisionBroadphase(filteredPairs, box, nextNode, aabb);
      }
    }
  }
}

bool pointPointCCD(
    const glm::vec3& a0,
    const glm::vec3& a1,
    const glm::vec3& b0,
    const glm::vec3& b1,
    float thresholdDistanceSq,
    glm::vec3& n) {

  // want closest distance between the two points along the interval
  // i.e., argmin(t) || (b0 - a0) (1 - t) + (b1 - a1) t ||
  // reformulated with:
  glm::vec3 d = b0 - a0;
  glm::vec3 v = (b1 - b0) - (a1 - a0);

  // we have:
  // argmin(t) || d + vt || =
  // argmin(t) (||d||^2 + d * vt + ||v||^2)
  // ==> 0 = d * v + 2 ||v||^2 t
  // ==> t = -d * v / (2 ||v||^2)

  float dMagSq = glm::dot(d, d);
  float vMagSq = glm::dot(v, v);

  // If the points are within the threshold at the start of the interval
  // can declare a collision
  if (dMagSq < thresholdDistanceSq) {
    // TODO: Consider case that dMagSq < EPSILON and the below is unsafe...
    // need to detect this and report for debugging. Ideally the point
    // can't get that close to a triangle without being forced away
    n = d / sqrt(dMagSq);
    return true;
  }

  // If there already exists enough separation and there is no relative velocity
  // can declare there will be no collision
  if (vMagSq < EPSILON)
    return false;

  // Compute time at which the points are closest, clamping at the ends of the
  // interval
  float t = -glm::dot(d, v) / (2.0f * vMagSq);
  t = glm::clamp(t, 0.0f, 1.0f);

  glm::vec3 closestDiff = d + v * t;
  float closestDistSq = glm::dot(closestDiff, closestDiff);

  if (closestDistSq >= thresholdDistanceSq)
    return false;

  n = d / sqrt(dMagSq);
  return true;
}

static bool pointTriangleCCD(
    const glm::vec3& p0,
    const glm::vec3& p1,
    const glm::vec3& a0,
    const glm::vec3& a1,
    const glm::vec3& b0,
    const glm::vec3& b1,
    const glm::vec3& c0,
    const glm::vec3& c1,
    float thresholdDistanceSq,
    bool bBackFace) {
  // find closest point on triangle at t0
  glm::vec3 ap0 = p0 - a0;
  glm::vec3 ab0 = b0 - a0;
  glm::vec3 ac0 = c0 - a0;
  glm::vec3 n0 = glm::cross(ab0, ac0);

  // float n0MagSq = glm::dot(n0, n0);
  // if (n0MagSq < EPSILON)
  // TODO: Add debug degenerate triangle detection here...

  glm::mat3 barycentricToRelativeWorld(ab0, ac0, n0);
  glm::mat3 relativeWorldToBarycentric = glm::inverse(barycentricToRelativeWorld);

  glm::vec3 barycentricCoords = relativeWorldToBarycentric * ap0;
  bBackFace = barycentricCoords.z < 0.0f;

  // clamp barycentric coords within triangle 
  barycentricCoords.x = glm::clamp(barycentricCoords.x, 0.0f, 1.0f);
  barycentricCoords.y = glm::clamp(barycentricCoords.y, 0.0f, 1.0f);
  barycentricCoords.z = 0.0f;

  glm::vec3 closestPoint_t0 = barycentricToRelativeWorld * barycentricCoords;

  glm::vec3 ab1 = b1 - a1;
  glm::vec3 ac1 = c1 - a1;
  glm::vec3 ap1 = p1 - a1;

  glm::vec3 closestPoint_t1 = barycentricCoords.x * ab1 + barycentricCoords.y * ac1;

  glm::vec3 n; // TODO: Should we not use this n??
  return pointPointCCD(ap0, ap1, closestPoint_t0, closestPoint_t1, thresholdDistanceSq, n);
}

int s_collisionMode = 0;
float s_thresholdDistance = 0.1f;
bool s_bVisualizeCollisions = true;

void Collisions::update(
    const StridedView<uint32_t>& indices,
    const StridedView<glm::vec3>& positions,
    const StridedView<glm::vec3>& prevPositions,
    float thresholdDistance,
    const AABBTree& aabb) {

  m_pointCollisions.clear();

  ALTHEA_STACK_VECTOR(filteredCollisions, const AABBLeaf*, 256);

  float thresholdDistSq = thresholdDistance * thresholdDistance;

  for (uint32_t leafIdxA = 0; leafIdxA < aabb.getLeafCount(); ++leafIdxA) {
    const AABBLeaf* leafA = aabb.getLeaf(leafIdxA);
    staticCollisionBroadphase(filteredCollisions, leafA, aabb.getRoot(), aabb);

    for (const AABBLeaf* leafB : filteredCollisions) {
      if (s_collisionMode == 0) {
        // Triangle-Triangle collision
        
      } else {
        // Point-Point collision
        for (uint32_t i = 0; i < 3; ++i) {
          for (uint32_t j = 0; j < 3; ++j) {
            uint32_t ia = indices[3 * leafA->triIdx + i];
            uint32_t ib = indices[3 * leafB->triIdx + j];

            if (ia == ib)
              continue;

            const glm::vec3& a0 = prevPositions[ia];
            const glm::vec3& a1 = positions[ia];
            const glm::vec3& b0 = prevPositions[ib];
            const glm::vec3& b1 = positions[ib];

            glm::vec3 n;
            if (!pointPointCCD(a0, a1, b0, b1, thresholdDistSq, n))
              continue;

            PointPointCollision& c = m_pointCollisions.emplace_back();
            c.normal = n;
            c.triIdxA = leafA->triIdx;
            c.pointIdxA = i;
            c.triIdxB = leafB->triIdx;
            c.pointIdxB = j;
          }
        }
      }
    }

    filteredCollisions.clear();
  }
}

CollisionsManager::CollisionsManager(
    Application& app,
    VkCommandBuffer commandBuffer,
    const GBufferResources& gBuffer,
    GlobalHeap& heap) {
  m_scene = SelectableScene(app, heap, commandBuffer, gBuffer);
}

void CollisionsManager::updateUI() {
  if (ImGui::CollapsingHeader("Collision", ImGuiTreeNodeFlags_DefaultOpen)) {
    static const char* modes[] = {"Triangle-Triangle", "Point-Point"};
    ImGui::Text("Collision Mode:");
    ImGui::Combo("##collisionmode", &s_collisionMode, modes, 2);
    ImGui::Text("Threshold Distance:");
    ImGui::SliderFloat(
        "##thresholddistance",
        &s_thresholdDistance,
        0.01f,
        0.5f);

    ImGui::Separator();
    ImGui::Text("Visualize Collisions:");
    ImGui::Checkbox("##visualizecollisions", &s_bVisualizeCollisions);
  }
}

void CollisionsManager::update(
    const FrameContext& frame,
    const StridedView<uint32_t>& indices,
    const StridedView<glm::vec3>& positions,
    const StridedView<glm::vec3>& prevPositions,
    const AABBTree& aabb) {
  m_collisions
      .update(indices, positions, prevPositions, s_thresholdDistance, aabb);

  if (s_bVisualizeCollisions) {
    m_scene.clear();
    for (const PointPointCollision& c : m_collisions.getPointCollisions()) {
      const glm::vec3& p0 = positions[indices[3 * c.triIdxA + c.pointIdxA]];
      const glm::vec3& p1 = positions[indices[3 * c.triIdxB + c.pointIdxB]];
      m_scene.addVertex(p0);
      m_scene.addVertex(p1);
    }
    m_scene.update(frame);
  }
}

void CollisionsManager::draw(
    const Application& app,
    VkCommandBuffer commandBuffer,
    const FrameContext& frame,
    VkDescriptorSet heapSet,
    UniformHandle globalUniformsHandle) {
  if (s_bVisualizeCollisions) {
    m_scene.draw(
        app,
        commandBuffer,
        frame,
        heapSet,
        globalUniformsHandle,
        s_thresholdDistance);
  }
}

} // namespace RibCage