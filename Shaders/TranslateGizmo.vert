#version 460 core 

layout(location=0) in vec3 inLocalVertPos;

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
  // Expect instance count = 3 * selectables count
  // (one for each axis on the gizmo)
  uint selectableIdx = gl_InstanceIndex / 3;
  uint axis = gl_InstanceIndex % 3;
  Selectable selectable = selectables[selectableIdx];

  // Hacky normal, does not look great for cylinders...
  vec3 normal = inLocalVertPos;
  vec3 localVertPos = inLocalVertPos;
  localVertPos += vec3(0.0, 0.0, 1.0);
  localVertPos *= vec3(0.25, 0.25, 2.5);

  // Reorient cylinder depending on which axis we are drawing.
  // The cylinder coords point in the z-direction by default, so
  // we swizzle the local-z to point in the correct axis direction.
  if (axis == 0) 
  {
    outColor = vec3(1.0, 0.0, 0.0);
    localVertPos = localVertPos.zxy;
    normal = normal.zxy;
  } else if (axis == 1) {
    outColor = vec3(0.0, 1.0, 0.0);
    localVertPos = localVertPos.yzx;
    normal = normal.yzx;
  } else {
    outColor = vec3(0.0, 0.0, 1.0);
  }

  // overwrite color if the node is not selected 
  // ??
  // outAlpha = bool(selectable.infoMask & INFO_BIT_SELECTED) ? 1.0 : 0.0;

  outNormal = normalize(normal);
  outPosition = selectable.position + pushConstants.selectionRadius * localVertPos;
  gl_Position = globals.projection * globals.view * vec4(outPosition, 1.0);

  outMetallicRoughnessOcclusion = vec3(0.0, 1.0, 1.0);
}