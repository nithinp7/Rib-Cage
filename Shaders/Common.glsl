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

#endif // _RIBCAGE_COMMON_