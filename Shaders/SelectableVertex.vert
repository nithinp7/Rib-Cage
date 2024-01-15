#version 460 core 

layout(location=0) in vec3 spherePos;
layout(location=1) in float sphereRadius;

layout(location=2) in vec3 localVertPos;

layout(location=0) out vec3 outPosition;
layout(location=1) out vec3 outNormal;
layout(location=2) out vec4 outColor;

#include <Global/GlobalUniforms.glsl>

layout(push_constant) uniform PushConstants {
  uint globalUniformsHandle;
  int selectedIdx;
} pushConstants;

#include globals RESOURCE(globalUniforms, pushConstants.globalUniformsHandle)

void main() {
  outNormal = normalize(localVertPos);
  outPosition = spherePos + sphereRadius * localVertPos;
  gl_Position = globals.projection * globals.view * vec4(outPosition, 1.0);

  if (pushConstants.selectedIdx == gl_InstanceIndex) {
    outColor = vec4(10.0, 10.0, 0.0, 1.0);
  } else {
    outColor = vec4(10.0, 0.0, 0.0, 1.0);
  }
}