
#version 460 core

layout(location=0) in vec3 inNormal;
layout(location=1) in vec2 inUv;

layout(location=0) out vec4 GBuffer_Normal;
layout(location=1) out vec4 GBuffer_Albedo;
layout(location=2) out vec4 GBuffer_MetallicRoughnessOcclusion;

void main() {
  GBuffer_Normal = vec4(inNormal, 1.0);
  GBuffer_Albedo = vec4(1.0, 0.0, 0.0, 1.0);
  
  // Since we use SSAO, we use the occlusion channel as a debug stencil
  GBuffer_MetallicRoughnessOcclusion = vec4(0.0, 0.3, 0.0, 1.0);
}
