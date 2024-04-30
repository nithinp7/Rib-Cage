#ifndef _DEBUGCOMMON_
#define _DEBUGCOMMON_

#include <Global/GlobalUniforms.glsl>

layout(push_constant) uniform PushConstants {
  uint globalUniforms;
} pushConstants;

#define globals globalUniforms[pushConstants.globalUniforms]

#endif // _DEBUGCOMMON_