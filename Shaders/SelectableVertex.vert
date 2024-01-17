#version 460 core 

layout(location=0) in vec3 localVertPos;

layout(location=0) out vec3 outPosition;
layout(location=1) out vec3 outNormal;
layout(location=2) out vec3 outColor;
layout(location=3) out vec3 outMetallicRoughnessOcclusion;

#include "Common.glsl"

#include <Global/GlobalUniforms.glsl>

layout(push_constant) uniform PushConstants {
  uint globalUniformsHandle;
  uint selectableVBHandle;
  float selectionRadius;
} pushConstants;

#define selectables RESOURCE(selectableVB, pushConstants.selectableVBHandle).vertices
#define globals RESOURCE(globalUniforms, pushConstants.globalUniformsHandle)

void main() {
  Selectable selectable = selectables[gl_InstanceIndex];

  outNormal = normalize(localVertPos);
  outPosition = selectable.position + pushConstants.selectionRadius * localVertPos;
  gl_Position = globals.projection * globals.view * vec4(outPosition, 1.0);

  if (bool(selectable.infoMask & INFO_BIT_SELECTED)) 
  {
    // Selected nodes yellow
    outColor = vec3(1.0, 1.0, 0.0);
  } else {
    // All other nodes red
    outColor = vec3(1.0, 0.0, 0.0);
  }

  outMetallicRoughnessOcclusion = vec3(0.0, 1.0, 1.0);
}