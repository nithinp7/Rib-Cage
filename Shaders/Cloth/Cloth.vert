#version 460 core 

#include <Misc/Sampling.glsl>
#include <Cloth/ClothCommon.glsl>

layout(location=0) out vec3 outPosition;
layout(location=1) out vec3 outNormal;
layout(location=2) out vec3 outColor;
layout(location=3) out vec3 outMetallicRoughnessOcclusion;

void main() {
  Node node = getNode(gl_VertexIndex);
  
  outPosition = getNodePosition(gl_VertexIndex);
  outNormal = vec3(0.0); // Will fallback to screen-space normals
  
  uvec2 colorSeed = uvec2(gl_VertexIndex, gl_VertexIndex+1);
  // uvec2 colorSeed = uvec2(node.objectIdx, node.objectIdx+1);
  outColor = randVec3(colorSeed);
  outMetallicRoughnessOcclusion = vec3(0.0, 0.3, 1.0);

  gl_Position = globals.projection * globals.view * vec4(outPosition, 1.0);
}