#ifndef _CLOTHCOMMON_
#define _CLOTHCOMMON_

#include <Bindless/GlobalHeap.glsl>
#include <Global/GlobalResources.glsl>
#include <Global/GlobalUniforms.glsl>

layout(push_constant) uniform PushConstants {
  uint clothUniforms;
} pushConstants;

UNIFORM_BUFFER(_clothUniforms, ClothUniforms{
  uint globalResources;
  uint globalUniforms;

  float deltaTime;
  
  uint nodesCount;
  uint nodePositions;
  uint nodeFlags;

  uint distanceConstraints;
  uint distanceConstraintsCount;
});
#define clothUniforms _clothUniforms[pushConstants.clothUniforms]

#define resources globalResources[clothUniforms.globalResources]
#define globals globalUniforms[clothUniforms.globalUniforms]

BUFFER_RW_PACKED(_nodePositions, NodePositionsHeap{
  vec3 verts[];
});
#define getNodePosition(nodeIdx) _nodePositions[clothUniforms.nodePositions].verts[nodeIdx]

BUFFER_RW_PACKED(_nodeFlags, NodeFlags{
  uint flags[];
});
#define getNodeFlags(nodeIdx) _nodeFlags[clothUniforms.nodeFlags].flags[nodeIdx]

struct DistanceConstraint {
  uint a;
  uint b;
  float restLength;
  float padding;
};
// TODO: Needs to be paged eventually
BUFFER_RW(_distanceConstraints, DistanceConstraintHeap{
  DistanceConstraint constraints[];
});
#define getDistanceConstraint(idx) \
    _distanceConstraints[clothUniforms.distanceConstraints].constraints[idx]

#endif // _CLOTHCOMMON_