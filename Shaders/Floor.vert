#version 460 core 

layout(location=0) out vec3 outPosition;
layout(location=1) out vec3 outNormal;
layout(location=2) out vec3 outColor;
layout(location=3) out vec3 outMetallicRoughnessOcclusion;

#include "Common.glsl"

#include <Global/GlobalUniforms.glsl>

layout(push_constant) uniform PushConstants {
  uint globalUniformsHandle;
  uint skeletonUniformsHandle;
} pushConstants;

#define globals RESOURCE(globalUniforms, pushConstants.globalUniformsHandle)
#define skeleton RESOURCE(skeletonUniforms, pushConstants.skeletonUniformsHandle)

void main() {
  float halfWidth = 100.0;
  float floorHeight = -5.0;
  if (gl_VertexIndex == 0 || gl_VertexIndex == 3) {
    outPosition = vec3(-halfWidth, floorHeight, -halfWidth);
  } else if (gl_VertexIndex == 1 || gl_VertexIndex == 5) {
    outPosition = vec3(halfWidth, floorHeight, halfWidth);
  } else if (gl_VertexIndex == 2) {
    outPosition = vec3(halfWidth, floorHeight, -halfWidth);
  } else if (gl_VertexIndex == 4) {
    outPosition = vec3(-halfWidth, floorHeight, halfWidth);
  }

  outNormal = vec3(0.0, 1.0, 0.0);
  outColor = vec3(1.0, 0.0, 0.0);
  outMetallicRoughnessOcclusion = vec3(0.0, 0.3, 1.0);

  gl_Position = globals.projection * globals.view * vec4(outPosition, 1.0);
}