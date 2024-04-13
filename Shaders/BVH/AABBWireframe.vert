#version 460 core 

#include <Misc/Sampling.glsl>
#include <BVH/BVHCommon.glsl>

layout(location=0) out vec3 outPosition;
layout(location=1) out vec3 outNormal;
layout(location=2) out vec3 outColor;
layout(location=3) out vec3 outMetallicRoughnessOcclusion;

void main() {
  AABBLeaf leaf = getAABBLeafNode(gl_InstanceIndex);

  // 12 lines in a box (24 verts)
  uint lineIdx = gl_VertexIndex / 2;
  uint pointIdx = gl_VertexIndex % 2;
  // 4 lines along each axis
  uint axis0 = lineIdx / 4;
  // the other axes
  uint axis1 = (axis0 + 1) % 3;
  uint axis2 = (axis0 + 2) % 3;
  
  // The sub-index of lines along this axis [0-3]
  uint axisLineIdx = lineIdx % 4;

  outPosition[axis0] = bool(pointIdx) ? leaf.min[axis0] : leaf.max[axis0];
  outPosition[axis1] = bool(axisLineIdx & 1) ? leaf.min[axis1] : leaf.max[axis1];
  outPosition[axis2] = bool(axisLineIdx & 2) ? leaf.min[axis2] : leaf.max[axis2];

  // TODO: REMOVE THIS
  // outPosition = pointIdx == 0 ? leaf.min : leaf.max;

  outNormal = vec3(0.0); // Will fallback to screen-space normals
  
  uvec2 colorSeed = uvec2(gl_InstanceIndex, gl_InstanceIndex+1);
  // uvec2 colorSeed = uvec2(node.objectIdx, node.objectIdx+1);
  outColor = randVec3(colorSeed);
  outMetallicRoughnessOcclusion = vec3(0.0, 0.3, 1.0);

  gl_Position = globals.projection * globals.view * vec4(outPosition, 1.0);
}