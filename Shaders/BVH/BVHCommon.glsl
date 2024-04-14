#ifndef _BVHCOMMON_
#define _BVHCOMMON_

#include <Global/GlobalUniforms.glsl>
#include <Global/GlobalResources.glsl>
#include <Bindless/GlobalHeap.glsl>

struct AABBHandles {
  uint innerNodes;
  uint leaves;
};
layout(push_constant) uniform PushConstant {
  AABBHandles handles;  
  uint globalResources;
  uint globalUniforms;
  uint flags;
} push;

#define resources globalResources[push.globalResources]
#define globals globalUniforms[push.globalUniforms]

struct AABBInnerNode {
  vec3 min;
  uint childA;
  vec3 max;
  uint childB;
  uint flags;
  uint padding;
};
BUFFER_R(_aabbInnerNodes, AABBInnerNodes{
  AABBInnerNode nodes[];
});
#define getAABBInnerNode(nodeIdx) \
    _aabbInnerNodes[push.handles.innerNodes].nodes[nodeIdx]

struct AABBLeaf {
  vec3 min;
  uint triIdx;
  vec3 max;
  uint padding;
};
BUFFER_R(_aabbLeaves, AABBLeaves{
  AABBLeaf leaves[];
});
#define getAABBLeafNode(leafIdx) \
    _aabbLeaves[push.handles.leaves].leaves[leafIdx]

#endif // _BVHCOMMON_