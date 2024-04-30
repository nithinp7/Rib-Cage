#version 460 core

#include "DebugCommon.glsl"

layout(location = 0) in vec3 inPosition;
layout(location = 1) in uint inColor;

layout(location=0) out vec3 outPosition;
layout(location=1) out vec3 outNormal;
layout(location=2) out vec3 outColor;
layout(location=3) out vec3 outMetallicRoughnessDebug;

void main() {
  outPosition = inPosition;
  outNormal = normalize(globals.view[3].xyz - inPosition);
  outColor = vec3(inColor >> 24, (inColor >> 16) & 0xff, (inColor >> 8) & 0xff) / 255.0;
  outMetallicRoughnessDebug = vec3(0.0, 0.3, 1.0);

  gl_Position = globals.projection * globals.view * vec4(inPosition, 1.0);
}