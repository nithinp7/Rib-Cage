#include "Collisions.h"

#include <Althea/Containers/StackVector.h>
#include <Althea/Gui.h>
#include <glm/gtc/matrix_inverse.hpp>

using namespace AltheaEngine;

#define EPSILON 0.000001f

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
    if (dMagSq < EPSILON)
      return false; // ???
    
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

// a,b,c are verts of triangle, p is the other point
static bool pointTriangleCCD(
    const glm::mat2x3& b2rw_t0, // barycentric to relative-world at t0
    const glm::mat3& rw2b_t0,   // relative-world to barycentric at t0
    const glm::mat2x3& b2rw_t1, // barycentric to relative-world at t1
    const glm::vec3& ap0,
    const glm::vec3& ap1,
    float thresholdDistanceSq,
    bool& bBackFace) {
  // barycentric coords
  glm::vec3 bc = rw2b_t0 * ap0;
  bBackFace = bc.z < 0.0f;

  // clamp barycentric coords within triangle
  glm::vec2 clampedBc(
      glm::clamp(bc.x, 0.0f, 1.0f),
      glm::clamp(bc.y, 0.0f, 1.0f));
  float uv = bc.x + bc.y;
  if (uv > 1.0f)
    clampedBc /= uv;

  glm::vec3 closestPoint_t0 = b2rw_t0 * clampedBc;
  glm::vec3 closestPoint_t1 = b2rw_t1 * clampedBc;

  glm::vec3 n; // TODO: Should we not use this n??
  return pointPointCCD(
      ap0,
      ap1,
      closestPoint_t0,
      closestPoint_t1,
      thresholdDistanceSq,
      n);
}

static void computeBarycentrics(
    const glm::vec3& a0,
    const glm::vec3& a1,
    const glm::vec3& b0,
    const glm::vec3& b1,
    const glm::vec3& c0,
    const glm::vec3& c1,
    glm::mat2x3& b2rw_t0, // barycentric to relative-world at t0
    glm::mat3& rw2b_t0,   // relative-world to barycentric at t0
    glm::mat2x3& b2rw_t1  // barycentric to relative-world at t1
) {

  glm::mat3 b2rw;
  b2rw[0] = b2rw_t0[0] = b0 - a0;
  b2rw[1] = b2rw_t0[1] = c0 - a0;
  // TODO: Add debug detection of degenerate triangles here...
  b2rw[2] = glm::cross(b2rw[0], b2rw[1]);

  rw2b_t0 = glm::inverse(b2rw);

  b2rw_t1[0] = b1 - a1;
  b2rw_t1[1] = c1 - a1;
}

static bool edgeEdgeCCD(
    const glm::vec3& a0,
    const glm::vec3& a1,
    const glm::vec3& b0,
    const glm::vec3& b1,
    const glm::vec3& c0,
    const glm::vec3& c1,
    const glm::vec3& d0,
    const glm::vec3& d1,
    float thresholdDistanceSq,
    float& u,
    float& v,
    glm::vec3& n) {
  
  // Find closest pair of points on each line at t0

  glm::vec3 ab = b0 - a0;
  glm::vec3 cd = d0 - c0;
  float cdMag = glm::length(cd); // TODO: Can we avoid this sqrt...
  glm::vec3 cdNorm = cd / cdMag; // TODO: Handle degenerate case...

  glm::vec3 ca = a0 - c0;

  // Want argmin(u,v) ||a + abu - c - cdv||^2 
  // where u and v are clamped to [0,1]
  // Solve unconstrained minimization first, then clamp u
  // then recompute v, then clamp v.
  // TODO: Is the above a bad assumption??
  
  // For given u, corresponding v is computed as follows
  // v = (ca + ab u) * cd / ||cd||

  // TODO: Need to document offline notes behind this equation
  glm::vec3 A = glm::dot(ab, cdNorm) * (cd - ab);
  glm::vec3 B = glm::dot(ca, cdNorm) * cd - ca;

  // argmin(u) ||Au + B||^2
  // 0 = 2 ||A||^2 u + 2 A * B
  u = -glm::dot(A,B)/glm::dot(A,A); // TODO: Degenerate case handling...
  u = glm::clamp(u, 0.0f, 1.0f); // TODO: Clamp before or after computing v??

  v = glm::dot(ca + ab * u, cdNorm) / cdMag;

  u = glm::clamp(u, 0.0f, 1.0f);
  v = glm::clamp(v, 0.0f, 1.0f);

  glm::vec3 closestPoint_ab_t0 = glm::mix(a0, b0, u);
  glm::vec3 closestPoint_ab_t1 = glm::mix(a1, b1, u);
  glm::vec3 closestPoint_cd_t0 = glm::mix(c0, d0, v);
  glm::vec3 closestPoint_cd_t1 = glm::mix(c1, d1, v);

  return pointPointCCD(
      closestPoint_ab_t0,
      closestPoint_ab_t1,
      closestPoint_cd_t0,
      closestPoint_cd_t1,
      thresholdDistanceSq,
      n);
}

static bool edgeEdgeCCD2(
    const glm::vec3& a0,
    const glm::vec3& a1,
    const glm::vec3& b0,
    const glm::vec3& b1,
    const glm::vec3& c0,
    const glm::vec3& c1,
    const glm::vec3& d0,
    const glm::vec3& d1,
    float thresholdDistanceSq,
    float& u,
    float& v,
    glm::vec3& n) {
  // Note: This routine in particular is largely taken from Pies (my previous
  // soft body simulation)
  // TODO: Worth reviewing this code at least to re-understand what is going on
  // here

  glm::vec3 ab0 = b0 - a0;
  glm::vec3 cd0 = d0 - a0;

  glm::vec3 ac0 = c0 - a0;
  glm::vec3 ad0 = d0 - a0;
  // Find the shortest distance between two lines

  float abMagSq = glm::dot(ab0, ab0);
  float cdMagSq = glm::dot(cd0, cd0);
  float abDotCd = glm::dot(ab0, ab0);

  float acDotAb = glm::dot(ac0, ab0);
  float acDotCd = glm::dot(ac0, cd0);

  float det = abMagSq * -cdMagSq + abDotCd * abDotCd;
  u = 0.0f;
  v = 0.0f;
  if (det != 0.0f) {
    det = 1.0f / det;
    u = (acDotAb * -cdMagSq + abDotCd * acDotCd) * det;
    v = (abMagSq * acDotCd - acDotAb * abDotCd) * det;
  } else {
    float u0 = 0.0f;
    float u1 = 1.0f;
    float v0 = glm::dot(ac0, ab0);
    float v1 = glm::dot(ad0, ab0);

    bool flip0 = false;
    bool flip1 = false;

    if (u0 > u1) {
      std::swap(u0, u1);
      flip0 = true;
    }

    if (v0 > v1) {
      std::swap(v0, v1);
      flip1 = true;
    }

    if (u0 >= v1) {
      u = flip0 ? 1.0f : 0.0f;
      v = flip1 ? 0.0f : 1.0f;
    } else if (v0 >= u1) {
      u = flip0 ? 0.0f : 1.0f;
      v = flip1 ? 1.0f : 0.0f;
    } else {
      float mid = (u0 > v0) ? (u0 + v1) * 0.5f : (v0 + u1) * 0.5f;
      u = (u0 == u1) ? 0.5f : (mid - u0) / (u1 - u0);
      v = (v0 == v1) ? 0.5f : (mid - v0) / (v1 - v0);
    }
  }

  u = glm::clamp(u, 0.0f, 1.0f);
  v = glm::clamp(v, 0.0f, 1.0f);

  glm::vec3 closestPoint_ab_t0 = glm::mix(a0, b0, u);
  glm::vec3 closestPoint_ab_t1 = glm::mix(a1, b1, u);
  glm::vec3 closestPoint_cd_t0 = glm::mix(c0, d0, v);
  glm::vec3 closestPoint_cd_t1 = glm::mix(c1, d1, v);

  return pointPointCCD(
      closestPoint_ab_t0,
      closestPoint_ab_t1,
      closestPoint_cd_t0,
      closestPoint_cd_t1,
      thresholdDistanceSq,
      n);
}

int s_collisionMode = 0;
float s_thresholdDistance = 0.1f;
bool s_bVisualizeCollisions = true;
int s_visualizationMode = 0;

void Collisions::update(
    const StridedView<uint32_t>& indices,
    const StridedView<glm::vec3>& positions,
    const StridedView<glm::vec3>& prevPositions,
    float thresholdDistance,
    const AABBTree& aabb) {

  m_pointCollisions.clear();
  m_triangleCollisions.clear();
  m_edgeCollisions.clear();

  ALTHEA_STACK_VECTOR(filteredCollisions, const AABBLeaf*, 256);

  float thresholdDistSq = thresholdDistance * thresholdDistance;

  for (uint32_t leafIdxA = 0; leafIdxA < aabb.getLeafCount(); ++leafIdxA) {
    const AABBLeaf* leafA = aabb.getLeaf(leafIdxA);
    staticCollisionBroadphase(filteredCollisions, leafA, aabb.getRoot(), aabb);

    for (const AABBLeaf* leafB : filteredCollisions) {
      if (s_collisionMode == 0) {
        // Triangle-Triangle collision
        {
          // Triangle A against all the points in triangle B
          uint32_t ia = indices[3 * leafA->triIdx + 0];
          uint32_t ib = indices[3 * leafA->triIdx + 1];
          uint32_t ic = indices[3 * leafA->triIdx + 2];

          glm::mat2x3 b2rw_t0;
          glm::mat3 rw2b_t0;
          glm::mat2x3 b2rw_t1;
          computeBarycentrics(
              prevPositions[ia],
              positions[ia],
              prevPositions[ib],
              positions[ib],
              prevPositions[ic],
              positions[ic],
              b2rw_t0,
              rw2b_t0,
              b2rw_t1);

          for (uint32_t j = 0; j < 3; ++j) {
            uint32_t ip = indices[3 * leafB->triIdx + j];

            // Skip if this vertex is part of the triangle
            if (ip == ia || ip == ib || ip == ic)
              continue;

            bool bBackFace;
            if (!pointTriangleCCD(
                    b2rw_t0,
                    rw2b_t0,
                    b2rw_t1,
                    prevPositions[ip] - prevPositions[ia],
                    positions[ip] - positions[ia],
                    thresholdDistSq,
                    bBackFace))
              continue;

            PointTriangleCollision& c = m_triangleCollisions.emplace_back();
            c.bBackFace = bBackFace;
            c.pointIdx = ip;
            c.triangleIdx = leafA->triIdx;
          }
        }

        {
          // Triangle B against all the points in triangle A
          uint32_t ia = indices[3 * leafB->triIdx + 0];
          uint32_t ib = indices[3 * leafB->triIdx + 1];
          uint32_t ic = indices[3 * leafB->triIdx + 2];

          glm::mat2x3 b2rw_t0;
          glm::mat3 rw2b_t0;
          glm::mat2x3 b2rw_t1;
          computeBarycentrics(
              prevPositions[ia],
              positions[ia],
              prevPositions[ib],
              positions[ib],
              prevPositions[ic],
              positions[ic],
              b2rw_t0,
              rw2b_t0,
              b2rw_t1);

          for (uint32_t j = 0; j < 3; ++j) {
            uint32_t ip = indices[3 * leafA->triIdx + j];

            // Skip if this vertex is part of the triangle
            if (ip == ia || ip == ib || ip == ic)
              continue;

            bool bBackFace;
            if (!pointTriangleCCD(
                    b2rw_t0,
                    rw2b_t0,
                    b2rw_t1,
                    prevPositions[ip] - prevPositions[ia],
                    positions[ip] - positions[ia],
                    thresholdDistSq,
                    bBackFace))
              continue;

            PointTriangleCollision& c = m_triangleCollisions.emplace_back();
            c.bBackFace = bBackFace;
            c.pointIdx = ip;
            c.triangleIdx = leafB->triIdx;
          }
        }

        // Check every combination of lines
        for (uint32_t i = 0; i < 3; ++i) {
          for (uint32_t j = 0; j < 3; ++j) {
            uint32_t ia = indices[3 * leafA->triIdx + i];
            uint32_t ib = indices[3 * leafA->triIdx + (i + 1) % 3];
            uint32_t ic = indices[3 * leafB->triIdx + j];
            uint32_t id = indices[3 * leafB->triIdx + (j + 1) % 3];

            // skip any line combinations with common vertices
            if (ia == ic || ia == id || ib == ic || ib == id)
              continue;

            glm::vec3 n;
            float u, v;
            if (!edgeEdgeCCD(
                    prevPositions[ia],
                    positions[ia],
                    prevPositions[ib],
                    positions[ib],
                    prevPositions[ic],
                    positions[ic],
                    prevPositions[id],
                    positions[id],
                    thresholdDistSq,
                    u,
                    v,
                    n))
              continue;

            EdgeCollision& c = m_edgeCollisions.emplace_back();
            c.triangleAIdx = leafA->triIdx;
            c.edgeAIdx = i;
            c.triangleBIdx = leafB->triIdx;
            c.edgeBIdx = j;
            c.u = u;
            c.v = v;
            c.normal = n;
          }
        }
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
    ImGui::Indent();
    // static const char* modes[] = {"Triangle-Triangle", "Point-Point"};
    // ImGui::Text("Collision Mode:");
    // ImGui::Combo("##collisionmode", &s_collisionMode, modes, 2);
    ImGui::Text("Threshold Distance:");
    ImGui::SliderFloat(
        "##thresholddistance",
        &s_thresholdDistance,
        0.01f,
        0.5f);

    ImGui::Separator();
    ImGui::Text("Show Contacts:");
    ImGui::SameLine();
    ImGui::Checkbox("##visualizecollisions", &s_bVisualizeCollisions);
    static const char* modes[] = {"Edge-Edge", "Point-Triangle"};
    ImGui::Text("Viz Mode:");
    ImGui::SameLine();
    ImGui::Combo("##visualizationmode", &s_visualizationMode, modes, 2);
    ImGui::Unindent();
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

    if (s_collisionMode == 0) {
      if (s_visualizationMode == 1) {
        for (const PointTriangleCollision& col :
             m_collisions.getTriangleCollisions()) {
          const glm::vec3& p = positions[col.pointIdx];
          m_scene.addVertex(p);
        }
      } else {
        for (const EdgeCollision& col : m_collisions.getEdgeCollisions()) {
          const glm::vec3& a = 
              positions[indices[3 * col.triangleAIdx + col.edgeAIdx]];
          const glm::vec3& b =
              positions[indices[3 * col.triangleAIdx + (col.edgeAIdx + 1) % 3]];
          const glm::vec3& c = 
              positions[indices[3 * col.triangleBIdx + col.edgeBIdx]];
          const glm::vec3& d =
              positions[indices[3 * col.triangleBIdx + (col.edgeBIdx + 1) % 3]];

          m_scene.addVertex(glm::mix(a, b, col.u));
          m_scene.addVertex(glm::mix(c, d, col.v));
        }
      }
    } else {
      for (const PointPointCollision& c : m_collisions.getPointCollisions()) {
        const glm::vec3& p0 = positions[indices[3 * c.triIdxA + c.pointIdxA]];
        const glm::vec3& p1 = positions[indices[3 * c.triIdxB + c.pointIdxB]];
        m_scene.addVertex(p0);
        m_scene.addVertex(p1);
      }
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

bool CollisionsManager::shouldVisualizeCollisions() const {
  return s_bVisualizeCollisions;
}

float CollisionsManager::getThresholdDistance() const {
  return s_thresholdDistance;
}
} // namespace RibCage