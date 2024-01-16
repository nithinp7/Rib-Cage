#version 460 core 

layout(location=0) in vec3 spherePos;
layout(location=1) in uint infoMask;

layout(location=2) in vec3 localVertPos;

layout(location=0) out vec3 outPosition;
layout(location=1) out vec3 outNormal;
layout(location=2) out vec4 outColor;

#include <Global/GlobalUniforms.glsl>

layout(push_constant) uniform PushConstants {
  uint globalUniformsHandle;
  float selectionRadius;
} pushConstants;

#define globals RESOURCE(globalUniforms, pushConstants.globalUniformsHandle)

#define INFO_BIT_SELECTED 1

void main() {
  outNormal = normalize(localVertPos);
  outPosition = spherePos + pushConstants.selectionRadius * localVertPos;
  gl_Position = globals.projection * globals.view * vec4(outPosition, 1.0);

  if (bool(infoMask & INFO_BIT_SELECTED)) {
    // Selected nodes yellow
    outColor = vec4(1.0, 1.0, 0.0, 1.0);
  } else {
    // All other nodes red
    outColor = vec4(1.0, 0.0, 0.0, 1.0);
  }
}