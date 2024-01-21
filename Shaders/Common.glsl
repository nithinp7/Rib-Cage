#ifndef _RIBCAGE_COMMON_
#define _RIBCAGE_COMMON_

#include <Bindless/GlobalHeap.glsl>

#define INFO_BIT_SELECTED 1

struct Selectable {
  vec3 position;
  uint infoMask; 
};

BUFFER_R(selectableVB, SelectableVertexBuffer{
  Selectable vertices[];
});

#define INVALID_BONE_IDX 0xFF
#define INVALID_JOINT_IDX 0xFF
#define MAX_BONE_COUNT 32
#define MAX_JOINT_COUNT 32

struct JointChildren {
  uint children; // packed with 4 uint8
};

UNIFORM_BUFFER(skeletonUniforms, SkeletonUniforms{
  mat4 localTransforms[MAX_JOINT_COUNT];
  mat4 worldTransforms[MAX_JOINT_COUNT];
  JointChildren jointChildren[MAX_JOINT_COUNT];
  uint info;
  // info contains the following layout
  // jointCount : 8
  // rootJoint : 8
  // padding : 16
});

#endif // _RIBCAGE_COMMON_