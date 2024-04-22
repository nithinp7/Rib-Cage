#include "Collisions.h"

#include <Althea/Containers/StackVector.h>
#include <Althea/Gui.h>

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

float pointPointCCD(
    const glm::vec3& a0,
    const glm::vec3& a1,
    const glm::vec3& b0,
    const glm::vec3& b1,
    float thresholdDistanceSq) {

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
  if (dMagSq < thresholdDistanceSq)
    return 0.0f;

  // If there already exists enough separation and there is no relative velocity
  // can declare there will be no collision
  if (vMagSq < EPSILON)
    return -1.0f;

  // Compute time at which the points are closest, clamping at the ends of the
  // interval
  float t = -glm::dot(d, v) / (2.0f * vMagSq);
  t = glm::clamp(t, 0.0f, 1.0f);

  glm::vec3 closestDiff = d + v * t;
  float closestDistSq = glm::dot(closestDiff, closestDiff);

  return closestDistSq < thresholdDistanceSq ? t : -1.0f;
}

Collisions::Collisions(
    const StridedView<uint32_t>& indices,
    const StridedView<glm::vec3>& positions,
    const StridedView<glm::vec3>& prevPositions,
    float thresholdDistance,
    const AABBTree& aabb) {

  m_collisions.clear();

  ALTHEA_STACK_VECTOR(leafCollisions, LeafCollision, indices.getCount() / 3);
  ALTHEA_STACK_VECTOR(filteredCollisions, const AABBLeaf*, 256);

  float thresholdDistSq = thresholdDistance * thresholdDistance;

  for (uint32_t leafIdxA = 0; leafIdxA < aabb.getLeafCount(); ++leafIdxA) {
    const AABBLeaf* leafA = aabb.getLeaf(leafIdxA);
    staticCollisionBroadphase(filteredCollisions, leafA, aabb.getRoot(), aabb);

    for (const AABBLeaf* leafB : filteredCollisions) {
      // TODO: Temporary point-point collisions
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

          float t = pointPointCCD(a0, a1, b0, b1, thresholdDistSq);

          if (t < 0.0)
            continue;

          CollisionConstraint& c = m_collisions.emplace_back();
          c.normal = glm::normalize(b0 - a0);
          c.primA = leafA->triIdx;
          c.primB = leafB->triIdx;
        }
      }
    }

    filteredCollisions.clear();
  }
}

CollisionsManager::CollisionsManager(
    Application& app,
    const GBufferResources& gBuffer,
    GlobalHeap& heap) {

}

float s_thresholdDistance = 0.01f;

void CollisionsManager::updateUI() {
  if (ImGui::CollapsingHeader("Collision", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Threshold Distance:");
    ImGui::SliderFloat("##thresholddistance", &s_thresholdDistance, 0.01f, 0.1f);
  }
}

void CollisionsManager::update(
      const StridedView<uint32_t>& indices,
      const StridedView<glm::vec3>& positions,
      const StridedView<glm::vec3>& prevPositions,
      const AABBTree& aabb) {
  m_collisions = Collisions(indices, positions, prevPositions, s_thresholdDistance, aabb);
}

} // namespace RibCage