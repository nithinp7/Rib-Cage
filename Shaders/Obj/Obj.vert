#version 460 core 

#include <Global/GlobalUniforms.glsl>
#include <Global/GlobalResources.glsl>

layout(push_constant) uniform PushConstants {
  uint globalResources;
  uint globalUniforms;
} pushConstants;

#define globals globalUniforms[pushConstants.globalUniforms]

layout(location=0) in vec3 inPosition;
layout(location=1) in vec3 inNormal;
layout(location=2) in vec2 inUV;

layout(location=0) out vec3 outNormal;
layout(location=1) out vec2 outUV;

void main() {
  // TODO: model transform...
  gl_Position = globals.projection * globals.view * vec4(inPosition, 1.0);
  outNormal = inNormal;
  outUV = inUV;
}