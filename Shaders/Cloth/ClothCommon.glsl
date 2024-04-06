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
  
  uint nodes;
  uint nodePositions;
  uint nodesCount;

  uint distanceConstraints;
  uint distanceConstraintsCount;
});
#define clothUniforms _clothUniforms[pushConstants.clothUniforms]

#define resources globalResources[clothUniforms.globalResources]
#define globals globalUniforms[clothUniforms.globalUniforms]

struct Node {
  vec3 position;
  uint objectIdx;
};
// TODO: Needs to be paged eventually
BUFFER_RW(_nodes, NodesHeap{
  Node nodes[];
});
#define getNode(nodeIdx) _nodes[clothUniforms.nodes].nodes[nodeIdx]

BUFFER_RW(_nodePositions, NodePositionsHeap{
  vec4 verts[];
});
#define getNodePosition(nodeIdx) _nodePositions[clothUniforms.nodePositions].verts[nodeIdx]

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