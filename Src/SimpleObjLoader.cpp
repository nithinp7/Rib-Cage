#include "SimpleObjLoader.h"

#include <Althea/Containers/StackVector.h>

#include <cstdint>
#include <cstdio>
#include <fstream>

namespace RibCage {
namespace SimpleObjLoader {

bool loadObj(
    const Application& app,
    SingleTimeCommandBuffer& commandBuffer,
    const char* fileName,
    LoadedObj& result) {
  std::ifstream file(fileName, std::ios::ate | std::ios::binary);

  if (!file.is_open()) {
    return false;
  }

  size_t fileSize = (size_t)file.tellg();

  file.seekg(0);

  // TODO: need to be able to fall-back if we blow this capacity...
  ALTHEA_STACK_VECTOR(positions, glm::vec3, 4096);
  ALTHEA_STACK_VECTOR(uvs, glm::vec2, 4096);
  ALTHEA_STACK_VECTOR(normals, glm::vec3, 4096);

  ObjMesh* mesh = &result.m_meshes.emplace_back();
  uint32_t indexCounter = 0;
  ALTHEA_STACK_VECTOR(vertices, ObjVert, 4096);

  auto createTriangle = [&](uint32_t v0,
                            uint32_t vt0,
                            uint32_t vn0,
                            uint32_t v1,
                            uint32_t vt1,
                            uint32_t vn1,
                            uint32_t v2,
                            uint32_t vt2,
                            uint32_t vn2) {
    ObjVert& vert0 = vertices.emplace_back();
    vert0.position = positions[v0];
    vert0.uv = uvs[vt0];
    vert0.normal = normals[vn0];

    ObjVert& vert1 = vertices.emplace_back();
    vert1.position = positions[v1];
    vert1.uv = uvs[vt1];
    vert1.normal = normals[vn1];

    ObjVert& vert2 = vertices.emplace_back();
    vert2.position = positions[v2];
    vert2.uv = uvs[vt2];
    vert2.normal = normals[vn2];
  };

  char lineBuf[1024];
  while (true) {
    file.getline(lineBuf, 1024);
    if (file.gcount() == 0)
      break;

    switch (lineBuf[0]) {
    case 'v':
      switch (lineBuf[1]) {
      case ' ':
        // position
        {
          glm::vec3& pos = positions.emplace_back();
          std::sscanf(&lineBuf[2], "%f %f %f", &pos.x, &pos.y, &pos.z);
        }
        break;
      case 't':
        // uv
        {
          glm::vec2& uv = uvs.emplace_back();
          std::sscanf(&lineBuf[2], "%f %f", &uv.x, &uv.y);
        }
        break;
      case 'n':
        // normal
        {
          glm::vec3& normal = normals.emplace_back();
          std::sscanf(&lineBuf[3], "%f %f %f", &normal.x, &normal.y, &normal.z);
        }
        break;
      }
      break;
    case 'g':
      // start new mesh
      {
        if (vertices.size() > 0) {
          // The last mesh was valid so finalize it and start a new one
          mesh->m_vertices = VertexBuffer<ObjVert>(
              app,
              commandBuffer,
              std::vector<ObjVert>(vertices.begin_ptr(), vertices.end_ptr()));
          vertices.clear();

          mesh = &result.m_meshes.emplace_back();
        }

        std::strncpy(mesh->name, &lineBuf[2], 128);
      }
      break;
    case 'f':
      // face
      {
        uint32_t v0, vt0, vn0, v1, vt1, vn1, v2, vt2, vn2, v3, vt3, vn3;
        int ret = sscanf(
            &lineBuf[2],
            "%u/%u/%u %u/%u/%u %u/%u/%u %u/%u/%u",
            &v0,
            &vt0,
            &vn0,
            &v1,
            &vt1,
            &vn1,
            &v2,
            &vt2,
            &vn2,
            &v3,
            &vt3,
            &vn3);

        // assumes the references verts have all been
        // specified earlier in the file
        if (ret == 9) {
          // triangle
          createTriangle(v0, vt0, vn0, v1, vt1, vn1, v2, vt2, vn2);
        } else if (ret == 12) {
          // quad
          createTriangle(v0, vt0, vn0, v1, vt1, vn1, v2, vt2, vn2);
          createTriangle(v0, vt0, vn0, v2, vt2, vn2, v3, vt3, vn3);
        } else {
          // unsupported face type
          assert(false);
        }
      }
      break;
    case '#':
    default:
      break;
    }
  }

  file.close();

  return true;
}

} // namespace SimpleObjLoader

using namespace SimpleObjLoader;

ObjTestScene::ObjTestScene(
    Application& app,
    SingleTimeCommandBuffer& commandBuffer,
    const GBufferResources& gBuffer,
    GlobalHeap& heap) {
  std::string path = GProjectDirectory + "/Data/ImportedModels/testExport.obj";
  loadObj(app, commandBuffer, path.c_str(), m_objects.emplace_back());
}

void ObjTestScene::tryRecompileShaders(Application& app) {}

void ObjTestScene::update(const FrameContext& frame) {}

void ObjTestScene::draw(
    const Application& app,
    VkCommandBuffer commandBuffer,
    const FrameContext& frame,
    VkDescriptorSet heapSet,
    BufferHandle globalResourcesHandle,
    UniformHandle globalUniformsHandle) {}

void ObjTestScene::updateUI() {}
} // namespace RibCage