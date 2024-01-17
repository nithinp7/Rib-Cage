#version 460 core 

layout(location=0) in vec3 gizmoPos;

layout(location=2) in vec3 localVertPos;

layout(location=0) out vec3 outPosition;
layout(location=1) out vec3 outNormal;
layout(location=2) out vec4 outColor;
layout(location=3) out vec4 outMetallicRoughnessOcclusion;

#include <Global/GlobalUniforms.glsl>

layout(push_constant) uniform PushConstants {
  uint globalUniformsHandle;
  float selectionRadius;
} pushConstants;

#define globals RESOURCE(globalUniforms, pushConstants.globalUniformsHandle)

void main() {
  // rough approximation, probably won't look great
  outNormal = normalize(localVertPos);

  gl_InstanceIndex % 3
  outPosition = spherePos + pushConstants.selectionRadius * localVertPos;
  gl_Position = globals.projection * globals.view * vec4(outPosition, 1.0);

  if (bool(infoMask & INFO_BIT_SELECTED)) 
  {
    // Selected nodes yellow
    outColor = vec4(1.0, 1.0, 0.0, 1.0);
  } else {
    // All other nodes red
    outColor = vec4(1.0, 0.0, 0.0, 1.0);
  }

  outMetallicRoughnessOcclusion = vec3(0.0, 1.0, 1.0);
}