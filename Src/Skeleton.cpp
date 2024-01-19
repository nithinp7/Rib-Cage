#include "Skeleton.h"

#include <Althea/Application.h>
#include <Althea/InputMask.h>
#include <Althea/Utilities.h>

#include <memory>

using namespace AltheaEngine;

namespace RibCage {

/*static*/
bool SkeletonLoader::load(const std::string& path, Skeleton& result) {
  if (!Utilities::checkFileExists(path))
    return false;

  std::vector<char> data = Utilities::readFile(path);
  if (data.size() != sizeof(Skeleton))
    return false;

  memcpy(&result, data.data(), sizeof(Skeleton));
  return true;
}

/*static*/
bool SkeletonLoader::save(const std::string& path, const Skeleton& skeleton) {
  return Utilities::writeFile(
      path,
      gsl::span((const char*)&skeleton, sizeof(skeleton)));
}

void SkeletonEditor::update(
    SelectableScene& scene,
    const glm::vec3& cameraPos,
    const glm::vec3& cursorDir,
    uint32_t prevInputMask,
    uint32_t inputMask) {
  bool bLastPressed = prevInputMask & INPUT_BIT_LEFT_MOUSE;
  bool bPressed = inputMask & INPUT_BIT_LEFT_MOUSE;

  bool bNewPress = !bLastPressed && bPressed;
  bool bHeldDown = bLastPressed && bPressed;
  bool bRelease = bLastPressed && !bPressed;

  bool bAlt = inputMask & INPUT_BIT_ALT;
  bool bCtrl = inputMask & INPUT_BIT_CTRL;

  if (bNewPress) {
    SceneQueryResult query = scene.query(cameraPos, cursorDir);

    if (bAlt) {
      // Alt-click spawns new bone attached to the last
      // selected joint
      int selected = scene.getLastSelected();

      if (selected == -1) {
        m_constraintPlane = cursorDir;
        m_constraintPoint = glm::vec3(0.0f);
      } else {
        const SelectableVertex& prevJoint = scene.getVertex(selected);
        m_constraintPoint = prevJoint.position;
      }

      // Intersect the camera ray with the constraint plane to compute
      // a spawn position
      float distToPlane =
          glm::dot(m_constraintPoint - cameraPos, m_constraintPlane);
      float dxdt = glm::dot(m_constraintPlane, cursorDir);
      if (abs(dxdt) > 0.01) {
        float t = distToPlane / dxdt;
        glm::vec3 spawnPos = cameraPos + cursorDir * t;
        scene.addVertex(spawnPos);
      }
    } else if (bCtrl) {
      // Ctrl-click adds to selection
      if (query.hitType == QHT_NONE) {
        scene.clearSelection();
        scene.disableGizmo();
      } else if (query.hitType == QHT_VERTEX) {
        scene.addToSelection(query.index);
        scene.enableGizmo();
        glm::vec3 gizmoPos(0.0f);
        for (int idx : scene.getSelection())
          gizmoPos += scene.getVertex(idx).position;
        gizmoPos /= scene.getSelection().size();
        scene.setGizmoPosition(gizmoPos);
      }
    } else {
      // Regular click replaces the current selection
      if (query.hitType == QHT_NONE) {
        scene.clearSelection();
        scene.disableGizmo();
      } else if (query.hitType == QHT_VERTEX) {
        scene.selectVertex(query.index);
        scene.enableGizmo();
        scene.setGizmoPosition(scene.getVertex(query.index).position);
      }
    }

    // Any type of click on a gizmo starts a drag
    if (query.hitType == QHT_GIZMO_HANDLE) {
      m_bDraggingGizmo = true;
      m_gizmoAxis = query.index;
      m_originalGizmoPosition = scene.getGizmoVertexPosition(m_gizmoAxis);
    }
  } else if (bHeldDown) {
    if (m_bDraggingGizmo) {
      // Update the gizmo position if this is a gizmo drag
      const glm::vec3& gizmoPos = scene.getGizmoVertexPosition(m_gizmoAxis);
      glm::vec3 axis(
          (float)(m_gizmoAxis == 0),
          (float)(m_gizmoAxis == 1),
          (float)(m_gizmoAxis == 2));
      glm::vec3 n = glm::normalize(glm::cross(axis, cursorDir));
      glm::vec3 c = cameraPos - gizmoPos;

      // can simplify dot with axis obviously
      float v0v1 = glm::dot(cursorDir, axis);
      float x0v0 = glm::dot(c, cursorDir);
      float x0v1 = glm::dot(c, axis);
      float denom = 1.0f - v0v1 * v0v1;
      if (denom > 0.001f) {
        // What is the interpretation of denom < EPS? Parallel lines?

        float s = (x0v1 * v0v1 - x0v0) / denom;
        float t = x0v1 + s * v0v1;

        glm::vec3 newGizmoPos = gizmoPos + t * axis;
        scene.setGizmoVertexPosition(newGizmoPos, m_gizmoAxis);
      } else {
        // Degenerate case, stop dragging to avoid glitches
        m_bDraggingGizmo = false;
      }
    }
  } else if (bRelease) {
    if (m_bDraggingGizmo) {
      // Release gizmo drag, apply translation on selected vertices
      m_bDraggingGizmo = false;
      glm::vec3 translation = scene.getGizmoVertexPosition(m_gizmoAxis) - m_originalGizmoPosition;
      for (int idx : scene.getSelection())
        scene.getVertexRef(idx).position += translation;
    }
  }
}
} // namespace RibCage