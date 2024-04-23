#version 460 core 

#include <Misc/Sampling.glsl>
#include <Cloth/ClothCommon.glsl>

layout(location=0) out vec3 outPosition;
layout(location=1) out vec3 outNormal;
layout(location=2) out vec3 outColor;
layout(location=3) out vec3 outMetallicRoughnessDebug;

void main() {
  uint flags = getNodeFlags(gl_VertexIndex);
  
  outPosition = getNodePosition(gl_VertexIndex);
  outNormal = vec3(0.0); // Will fallback to screen-space normals
  
  outColor = vec3(1.0, 0.8, 0.7);
  outMetallicRoughnessDebug = vec3(0.0, 0.3, 0.0);
  if (flags == 1) {
    outColor = vec3(0.0, 0.0, 1000.0);
    outMetallicRoughnessDebug.z = 1.0;
  }

  gl_Position = globals.projection * globals.view * vec4(outPosition, 1.0);
}