#pragma once

#include <Althea/ImageResource.h>
#include <Althea/IndexBuffer.h>
#include <Althea/SingleTimeCommandBuffer.h>
#include <Althea/VertexBuffer.h>
#include <Althea/DeferredRendering.h>
#include <Althea/FrameContext.h>
#include <Althea/RenderPass.h>
#include <Althea/GlobalHeap.h>
#include <glm/glm.hpp>

#include <vector>

using namespace AltheaEngine;

namespace RibCage {
namespace SimpleObjLoader {

struct ObjVert {
  glm::vec3 position;
  glm::vec3 normal;
  glm::vec2 uv;
};

struct ObjMesh {
  char name[128] = {0};
  VertexBuffer<ObjVert> m_vertices;
  int m_albedo = -1;
  int m_normal = -1;
  int m_metallicRoughness = -1;
};

struct LoadedObj {
  std::vector<ImageResource> m_images;
  std::vector<ObjMesh> m_meshes;
};

bool loadObj(
    const Application& app,
    SingleTimeCommandBuffer& commandBuffer,
    const char* fileName,
    LoadedObj& result);
    
} // namespace SimpleObjLoader

class ObjTestScene {
public:
  ObjTestScene() = default;
  ObjTestScene(
      Application& app,
      SingleTimeCommandBuffer& commandBuffer,
      const GBufferResources& gBuffer,
      GlobalHeap& heap);

  void tryRecompileShaders(Application& app);

  void update(const FrameContext& frame);

  void draw(
      const Application& app,
      VkCommandBuffer commandBuffer,
      const FrameContext& frame,
      VkDescriptorSet heapSet,
      BufferHandle globalResourcesHandle,
      UniformHandle globalUniformsHandle);

  void updateUI();

private:
  RenderPass m_gbufferPass;
  std::vector<SimpleObjLoader::LoadedObj> m_objects;
};
} // namespace RibCage