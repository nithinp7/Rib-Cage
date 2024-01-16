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
  bool bPressed = inputMask & INPUT_BIT_RIGHT_MOUSE;

  bool bNewPress = !bLastPressed && bPressed;
  bool bRelease = bLastPressed && !bPressed;

  bool bAlt = inputMask & INPUT_BIT_ALT;
  bool bCtrl = inputMask & INPUT_BIT_CTRL;

  if (bNewPress) {
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
      float distToPlane = glm::dot(m_constraintPoint - cameraPos, m_constraintPlane);
      float dxdt = glm::dot(m_constraintPlane, cursorDir);
      if (abs(dxdt) > 0.01) {
        float t = distToPlane / dxdt;
        glm::vec3 spawnPos = cameraPos + cursorDir * t;
        scene.addVertex(spawnPos);
      }
    } else if (bCtrl) {
      // Ctrl-click adds to selection
      if (!scene.trySelect(cameraPos, cursorDir, true))
        scene.clearSelection();
    } else {
      // Regular click replaces the current selection
      if (!scene.trySelect(cameraPos, cursorDir, false))
        scene.clearSelection();
    }
  }
}
} // namespace RibCage