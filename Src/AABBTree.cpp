#include "AABBTree.h"

#include <gsl/span>

#include <algorithm>
#include <limits>

using namespace AltheaEngine;

namespace RibCage {

namespace {
struct PushConstants {
  uint32_t globalResources;
  uint32_t globalUniforms;
  uint32_t clothUniforms;
  AABBHandles handles;
};

// From Mathias Muller
uint32_t hashGridCell(int32_t x, int32_t y, int32_t z) {
  return uint32_t(glm::abs((x * 92837111) ^ (y * 689287499) ^ (z * 283923481)));
}

} // namespace

AABBTree::AABBTree(Application& app, GlobalHeap& heap, uint32_t leafCount) {
  m_innerNodesBuffer =
      DynamicVertexBuffer<AABBInnerNode>(app, leafCount, false);
  m_leavesBuffer = DynamicVertexBuffer<AABBLeaf>(app, leafCount, false);
}

void AABBTree::upload(uint32_t ringBufferIndex) {
  m_innerNodesBuffer.updateVertices(ringBufferIndex, m_innerNodes);
  m_leavesBuffer.updateVertices(ringBufferIndex, m_leaves);
}

AABBHandles AABBTree::getHandles(uint32_t ringBufferIndex) const {
  return AABBHandles{
      m_innerNodesBuffer.getCurrentBufferHandle(ringBufferIndex).index,
      m_leavesBuffer.getCurrentBufferHandle(ringBufferIndex).index};
}

void AABBTree::refitTriangles(
    const StridedView<uint32_t>& tris,
    const StridedView<glm::vec3>& verts,
    float padding) {
  // TODO: Need to verify that the tri count has not changed
  uint32_t triCount = tris.getCount() / 3;

  m_leaves.clear();
  m_leaves.reserve(triCount);
  // TODO: re-evaluate required sizes of inner nodes
  m_innerNodes.clear();
  m_innerNodes.reserve(triCount);

  glm::vec3 treeMin(std::numeric_limits<float>::max());
  glm::vec3 treeMax(std::numeric_limits<float>::lowest());

  // create leaf nodes
  for (uint32_t i = 0; i < triCount; ++i) {
    const glm::vec3& v0 = verts[tris[3 * i]];
    const glm::vec3& v1 = verts[tris[3 * i + 1]];
    const glm::vec3& v2 = verts[tris[3 * i + 2]];

    AABBLeaf& leaf = m_leaves.emplace_back();
    leaf.min = glm::min(glm::min(v0, v1), v2) - glm::vec3(padding);
    leaf.max = glm::max(glm::max(v0, v1), v2) + glm::vec3(padding);
    leaf.triIdx = i;
    
    treeMin = glm::min(treeMin, leaf.min);
    treeMax = glm::max(treeMax, leaf.max);
  }

  uint32_t* pSortedLeaves =
      reinterpret_cast<uint32_t*>(alloca(triCount * sizeof(uint32_t)));
  gsl::span<uint32_t> sortedLeaves(pSortedLeaves, triCount);
  for (uint32_t i = 0; i < triCount; ++i)
    sortedLeaves[i] = i;

  populateInnerNode(treeMin, treeMax, sortedLeaves);
}

void AABBTree::populateInnerNode(
    const glm::vec3& min,
    const glm::vec3& max,
    gsl::span<uint32_t> sortedLeaves) {
  AABBInnerNode& node = m_innerNodes.emplace_back();
  node.min = min;
  node.max = max;
  node.childA = ~0;
  node.childB = ~0;
  node.flags = 0;

  if (sortedLeaves.size() <= 2) {
    if (sortedLeaves.size() > 0)
      node.childA = sortedLeaves[0];
    if (sortedLeaves.size() > 1)
      node.childB = sortedLeaves[1];

    node.flags |= InnerNodeFlag_HasLeafChildren;
    return;
  }

  // split largest axis
  uint32_t midpointIdx = sortedLeaves.size() / 2;

  glm::vec3 dims = node.max - node.min;
  uint32_t axis = 0;
  if (dims.y > dims.x && dims.y > dims.z)
    axis = 1;
  else if (dims.z > dims.x)
    axis = 2;

  // partial sort s.t., the nth element (midpoint in this case) is in the
  // correct place
  auto beginIt = sortedLeaves.begin();
  auto midpointIt = beginIt + midpointIdx;
  auto endIt = sortedLeaves.end();

  glm::vec3 minA, minB, maxA, maxB;
  minA = minB = node.min;
  maxA = maxB = node.max;

  // TODO: Is comparing box mins sufficient?
  auto cmpX = [&](uint32_t i0, uint32_t i1) -> bool {
    // float c0 = 0.5f * m_nodes[i0].m_min.x + 0.5f * m_nodes[i0].m_max.x;
    // float c1 = 0.5f * m_nodes[i1].m_min.x + 0.5f * m_nodes[i1].m_max.x;
    return m_leaves[i0].min.x < m_leaves[i1].min.x;
  };

  auto cmpY = [&](uint32_t i0, uint32_t i1) -> bool {
    // float c0 = 0.5f * m_nodes[i0].m_min.y + 0.5f * m_nodes[i0].m_max.y;
    // float c1 = 0.5f * m_nodes[i1].m_min.y + 0.5f * m_nodes[i1].m_max.y;
    return m_leaves[i0].min.y < m_leaves[i1].min.y;
  };

  auto cmpZ = [&](uint32_t i0, uint32_t i1) -> bool {
    // float c0 = 0.5f * m_nodes[i0].m_min.z + 0.5f * m_nodes[i0].m_max.z;
    // float c1 = 0.5f * m_nodes[i1].m_min.z + 0.5f * m_nodes[i1].m_max.z;
    return m_leaves[i0].min.z < m_leaves[i1].min.z;
  };

  if (axis == 0) {
    std::nth_element(beginIt, midpointIt, endIt, cmpX);
    uint32_t pivotIdx = *midpointIt;
    float pivot = m_leaves[pivotIdx].min.x;
    maxA.x = minB.x = pivot;
  } else if (axis == 1) {
    std::nth_element(beginIt, midpointIt, endIt, cmpY);
    uint32_t pivotIdx = *midpointIt;
    float pivot = m_leaves[pivotIdx].min.y;
    maxA.y = minB.y = pivot;
  } else {
    std::nth_element(beginIt, midpointIt, endIt, cmpZ);
    uint32_t pivotIdx = *midpointIt;
    float pivot = m_leaves[pivotIdx].min.z;
    maxA.z = minB.z = pivot;
  }

  populateInnerNode(minA, maxA, gsl::span(&sortedLeaves[0], midpointIdx));
  populateInnerNode(
      minB,
      maxB,
      gsl::span(&sortedLeaves[midpointIdx], sortedLeaves.size() - midpointIdx));
}

AABBManager::AABBManager(
    Application& app,
    const GBufferResources& gBuffer,
    GlobalHeap& heap) {
  std::vector<SubpassBuilder> builders;
  {
    SubpassBuilder& builder = builders.emplace_back();

    GBufferResources::setupAttachments(builder);

    builder.pipelineBuilder.setPrimitiveType(PrimitiveType::LINES)
        .addVertexShader(GProjectDirectory + "/Shaders/Cloth/ClothAABB.vert")
        .addFragmentShader(
            GProjectDirectory + "/Shaders/GBufferPassThrough.frag")
        .layoutBuilder.addDescriptorSet(heap.getDescriptorSetLayout())
        .addPushConstants<PushConstants>(VK_SHADER_STAGE_ALL);
  }

  std::vector<Attachment> attachments = gBuffer.getAttachmentDescriptions();
  m_dbgRenderPass = RenderPass(
      app,
      app.getSwapChainExtent(),
      std::move(attachments),
      std::move(builders));

  m_dbgFrameBuffer = FrameBuffer(
      app,
      m_dbgRenderPass,
      app.getSwapChainExtent(),
      gBuffer.getAttachmentViewsA());
}

void AABBManager::update(const FrameContext& frame) {
  
}

} // namespace RibCage