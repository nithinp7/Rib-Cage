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
  uint skeletonUniformsHandle;
} pushConstants;

#define globals RESOURCE(globalUniforms, pushConstants.globalUniformsHandle)
#define skeleton RESOURCE(skeletonUniforms, pushConstants.skeletonUniformsHandle)

// Useful functions for transforming directions
void coordinateSystem(in vec3 v1, out vec3 v2, out vec3 v3) {
    if (abs(v1.x) > abs(v1.y))
      v2 = vec3(-v1.z, 0, v1.x) / sqrt(v1.x * v1.x + v1.z * v1.z);
    else
      v2 = vec3(0, v1.z, -v1.y) / sqrt(v1.y * v1.y + v1.z * v1.z);
    v3 = cross(v1, v2);
}

mat3 LocalToWorld(vec3 nor) {
    vec3 tan, bit;
    coordinateSystem(nor, tan, bit);
    return mat3(tan, bit, nor);
}

void main() {
  mat4 worldTransform = skeleton.worldTransforms[gl_InstanceIndex];
  mat4 localTransform = skeleton.localTransforms[gl_InstanceIndex];

  vec3 bone = -localTransform[3].xyz;
  float boneLength = max(length(bone), 0.001);
  mat4 model = mat4(LocalToWorld(bone / boneLength));
  model[3] = worldTransform[3];

  // Hacky normal, does not look great for cylinders...
  vec4 worldNormal = model * vec4(inLocalVertPos, 0.0);

  vec4 localVertPos = vec4(inLocalVertPos, 1.0);
  localVertPos.xy *= 0.25; // shrink radius of cylinder
  localVertPos.z = (localVertPos.z + 1.0) * 0.5 * boneLength;
  vec4 worldVertPos = model * localVertPos;
  gl_Position = globals.projection * globals.view * worldVertPos;

  outPosition = worldVertPos.xyz;
  outNormal = normalize(worldNormal.xyz);
  outColor = vec3(0.5, 0.0, 0.5);
  outMetallicRoughnessOcclusion = vec3(0.0, 1.0, 1.0);
}