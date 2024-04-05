
#version 460 core

layout(location=0) in vec3 inPosition;
layout(location=1) in vec3 inNormal;
layout(location=2) in vec3 inColor;
layout(location=3) in vec3 inMetallicRoughnessOcclusion;

layout(location=0) out vec4 GBuffer_Normal;
layout(location=1) out vec4 GBuffer_Albedo;
layout(location=2) out vec4 GBuffer_MetallicRoughnessOcclusion;

void main() {
  GBuffer_Normal = vec4(inNormal, 1.0);
  GBuffer_Albedo = vec4(inColor, 1.0);
  GBuffer_MetallicRoughnessOcclusion = vec4(inMetallicRoughnessOcclusion, 1.0);
}
