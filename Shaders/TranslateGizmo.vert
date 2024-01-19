#version 460 core 

layout(location=0) in vec3 inLocalVertPos;

layout(location=0) out vec3 outPosition;
layout(location=1) out vec3 outNormal;
layout(location=2) out vec3 outColor;
layout(location=3) out vec3 outMetallicRoughnessOcclusion;

#include "Common.glsl"

#include <Global/GlobalUniforms.glsl>

layout(push_constant) uniform PushConstants {
  vec3 gizmoPos;
  float gizmoScale;
  float gizmoLength;
  uint gizmoPart; // 0 for the cylinders 1 for sphere
  uint globalUniformsHandle;
} pushConstants;

#define globals RESOURCE(globalUniforms, pushConstants.globalUniformsHandle)

void main() {
  uint axis = gl_InstanceIndex % 3;

  // Hacky normal, does not look great for cylinders...
  vec3 normal = inLocalVertPos;
  vec3 localVertPos = inLocalVertPos;
  if (pushConstants.gizmoPart == 0) {
    localVertPos *= vec3(0.25, 0.25, 0.5 * pushConstants.gizmoLength);
    localVertPos += vec3(0.0, 0.0, 0.5 * pushConstants.gizmoLength);
  } else {
    localVertPos *= pushConstants.gizmoScale;
    localVertPos += vec3(0.0, 0.0, pushConstants.gizmoLength);
  }

  // Reorient cylinder depending on which axis we are drawing.
  // The cylinder coords point in the z-direction by default, so
  // we swizzle the local-z to point in the correct axis direction.
  if (axis == 0) 
  {
    outColor = vec3(1.0, 0.0, 0.0);
    localVertPos = localVertPos.zxy;
    normal = normal.zxy;
  } else if (axis == 1) {
    outColor = vec3(0.0, 1.0, 0.0);
    localVertPos = localVertPos.yzx;
    normal = normal.yzx;
  } else {
    outColor = vec3(0.0, 0.0, 1.0);
  }

  // overwrite color if the node is not selected 
  // ??
  // outAlpha = bool(selectable.infoMask & INFO_BIT_SELECTED) ? 1.0 : 0.0;

  outNormal = normalize(normal);
  outPosition = pushConstants.gizmoPos +  localVertPos;
  gl_Position = globals.projection * globals.view * vec4(outPosition, 1.0);

  outMetallicRoughnessOcclusion = vec3(0.0, 1.0, 1.0);
}