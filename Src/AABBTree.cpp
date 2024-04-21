#include "AABBTree.h"

#include <Althea/Gui.h>
#include <gsl/span>

#include <algorithm>
#include <limits>

using namespace AltheaEngine;

namespace RibCage {
// TODO: Rename to KDTree??

const AABBInnerNode* AABBTree::getRoot() const { return &m_innerNodes[0]; }
const AABBInnerNode* AABBTree::getChildA(const AABBInnerNode* pNode) const {
  return &m_innerNodes[pNode->childA];
}
const AABBInnerNode* AABBTree::getChildB(const AABBInnerNode* pNode) const {
  return &m_innerNodes[pNode->childB];
}
bool AABBTree::isTerminalNode(const AABBInnerNode* pNode) const {
  return bool(pNode->flags & InnerNodeFlag_HasLeafChildren);
}
const AABBLeaf* AABBTree::getLeafA(const AABBInnerNode* pNode) const {
  return &m_leaves[pNode->childA];
}
const AABBLeaf* AABBTree::getLeafB(const AABBInnerNode* pNode) const {
  return &m_leaves[pNode->childB];
}
const AABBInnerNode* AABBTree::getInnerNode(uint32_t nodeIdx) const {
  return &m_innerNodes[nodeIdx];
}
const AABBLeaf* AABBTree::getLeaf(uint32_t leafIdx) const {
  return &m_leaves[leafIdx];
}
bool AABBTree::hasChildA(const AABBInnerNode* pNode) const {
  return pNode->childA != ~0u;
}
bool AABBTree::hasChildB(const AABBInnerNode* pNode) const {
  return pNode->childB != ~0u;
}

// TODO: Some sort of SSE way to do this via glm?
static bool intersect(float minA, float maxA, float minB, float maxB) {
  minA <= minB && maxA >= minB;
  minA >= minB && maxA <= maxB;

  if (minA <= minB) {
    return maxA >= minB;
  } else {
    return minA <= maxB;
  }
}

static bool intersect(
    const glm::vec3& minA,
    const glm::vec3& maxA,
    const glm::vec3& minB,
    const glm::vec3& maxB) {
  return 
    intersect(minA.x, maxA.x, minB.x, maxB.x) &&
    intersect(minA.y, maxA.y, minB.y, maxB.y) &&
    intersect(minA.z, maxA.z, minB.z, maxB.z);
}

bool AABBInnerNode::intersect(const glm::vec3& minB, const glm::vec3& maxB) const {
  return RibCage::intersect(min, max, minB, maxB);
}

bool AABBLeaf::intersect(const glm::vec3& minB, const glm::vec3& maxB) const {
  return RibCage::intersect(min, max, minB, maxB);
}

AABBTree::AABBTree(Application& app, GlobalHeap& heap, uint32_t leafCount) {
  // TODO: Pre-sizing inner nodes?
  m_innerNodesBuffer =
      DynamicVertexBuffer<AABBInnerNode>(app, 3 * leafCount, false);
  m_innerNodesBuffer.registerToHeap(heap);

  m_leavesBuffer = DynamicVertexBuffer<AABBLeaf>(app, leafCount, false);
  m_leavesBuffer.registerToHeap(heap);
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
    const StridedView<glm::vec3>& positions,
    const StridedView<glm::vec3>& prevPositions,
    float padding) {
  // TODO: Need to verify that the tri count has not changed
  uint32_t triCount = tris.getCount() / 3;

  m_leaves.clear();
  m_leaves.reserve(triCount);
  // TODO: re-evaluate required sizes of inner nodes
  m_innerNodes.clear();
  m_innerNodes.reserve(3 * triCount);

  // create leaf nodes
  for (uint32_t i = 0; i < triCount; ++i) {

    AABBLeaf& leaf = m_leaves.emplace_back();
    leaf.min = glm::vec3(std::numeric_limits<float>::max());
    leaf.max = glm::vec3(std::numeric_limits<float>::lowest()); 
    for (uint32_t j = 0; j < 3; ++j) {
      const glm::vec3& p0 = positions[tris[3 * i + j]];
      const glm::vec3& p1 = prevPositions[tris[3 * i + j]];
      leaf.min = glm::min(glm::min(p0, p1), leaf.min);
      leaf.max = glm::max(glm::max(p0, p1), leaf.max);
    }

    leaf.min -= glm::vec3(padding);
    leaf.max += glm::vec3(padding);
    
    leaf.triIdx = i;
  }

  uint32_t* pSortedLeaves =
      reinterpret_cast<uint32_t*>(alloca(triCount * sizeof(uint32_t)));
  gsl::span<uint32_t> sortedLeaves(pSortedLeaves, triCount);
  for (uint32_t i = 0; i < triCount; ++i)
    sortedLeaves[i] = i;

  populateInnerNode(sortedLeaves);
}

void AABBTree::populateInnerNode(gsl::span<uint32_t> sortedLeaves) {
  AABBInnerNode& node = m_innerNodes.emplace_back();

  node.min = glm::vec3(std::numeric_limits<float>::max());
  node.max = glm::vec3(std::numeric_limits<float>::lowest());
  for (uint32_t leafIdx : sortedLeaves) {
    const AABBLeaf& leaf = m_leaves[leafIdx];
    node.min = glm::min(node.min, leaf.min);
    node.max = glm::max(node.max, leaf.max);
  }

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

  // TODO: Is comparing box mins sufficient?
  auto cmpX = [&](uint32_t i0, uint32_t i1) -> bool {
    return m_leaves[i0].min.x < m_leaves[i1].min.x;
  };

  auto cmpY = [&](uint32_t i0, uint32_t i1) -> bool {
    return m_leaves[i0].min.y < m_leaves[i1].min.y;
  };

  auto cmpZ = [&](uint32_t i0, uint32_t i1) -> bool {
    return m_leaves[i0].min.z < m_leaves[i1].min.z;
  };

  // TODO: Does nth_element at the midpoint guarantee the lower
  // half is less than / eq to upper half?
  if (axis == 0) {
    std::nth_element(beginIt, midpointIt, endIt, cmpX);
  } else if (axis == 1) {
    std::nth_element(beginIt, midpointIt, endIt, cmpY);
  } else {
    std::nth_element(beginIt, midpointIt, endIt, cmpZ);
  }

  // Child A is unnecessary to mark down in a depth-first ordering...
  node.childA = m_innerNodes.size();
  populateInnerNode(gsl::span(&sortedLeaves[0], midpointIdx));
  node.childB = m_innerNodes.size();
  populateInnerNode(
      gsl::span(&sortedLeaves[midpointIdx], sortedLeaves.size() - midpointIdx));
}

AABBManager::AABBManager(
    Application& app,
    const GBufferResources& gBuffer,
    GlobalHeap& heap,
    uint32_t leafCount) {
  std::vector<SubpassBuilder> builders;
  {
    SubpassBuilder& builder = builders.emplace_back();

    GBufferResources::setupAttachments(builder);

    builder.pipelineBuilder.setPrimitiveType(PrimitiveType::LINES)
        .setLineWidth(2.0f)
        .addVertexShader(GProjectDirectory + "/Shaders/BVH/AABBWireframe.vert")
        .addFragmentShader(
            GProjectDirectory + "/Shaders/GBufferPassThrough.frag")
        .layoutBuilder.addDescriptorSet(heap.getDescriptorSetLayout())
        .addPushConstants<AABBPushConstants>(VK_SHADER_STAGE_ALL);
  }

  std::vector<Attachment> attachments = gBuffer.getAttachmentDescriptions();
  for (auto& attachment : attachments)
    attachment.load = true;

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

  m_tree = AABBTree(app, heap, leafCount);
}

static bool s_showAABBLeaves = true;
static bool s_showAABBInnerNodes = true;
static float s_padding = 0.0f;

void AABBManager::update(
    const StridedView<uint32_t>& tris,
    const StridedView<glm::vec3>& positions,
    const StridedView<glm::vec3>& prevPositions,
    const FrameContext& frame) {
  // TODO: Add padding, add swept prims, etc
  m_tree.refitTriangles(tris, positions, prevPositions, s_padding);
  m_tree.upload(frame.frameRingBufferIndex);
}

void AABBManager::updateUI() {
  if (ImGui::CollapsingHeader("BVH", ImGuiTreeNodeFlags_DefaultOpen)) {
    ImGui::Text("Padding:");
    ImGui::SliderFloat("##padding", &s_padding, 0.0f, 2.5f);
    ImGui::Text("Show AABB Leaves:");
    ImGui::Checkbox("##showleaves", &s_showAABBLeaves);
    ImGui::Text("Show AABB Inner Nodes:");
    ImGui::Checkbox("##showinnernodes", &s_showAABBInnerNodes);
  }
}

void AABBManager::debugDraw(
    const Application& app,
    VkCommandBuffer commandBuffer,
    const FrameContext& frame,
    VkDescriptorSet heapSet,
    BufferHandle globalResources,
    UniformHandle globalUniforms) const {

  AABBPushConstants push{};
  push.globalResources = globalResources.index;
  push.globalUniforms = globalUniforms.index;
  push.handles = m_tree.getHandles(frame.frameRingBufferIndex);

  {
    ActiveRenderPass pass =
        m_dbgRenderPass.begin(app, commandBuffer, frame, m_dbgFrameBuffer);
    pass.setGlobalDescriptorSets(gsl::span(&heapSet, 1));
    pass.getDrawContext().bindDescriptorSets();

    if (s_showAABBLeaves) {
      push.flags = 0; // leaf nodes
      pass.getDrawContext().updatePushConstants(push, 0);
      pass.getDrawContext().draw(24, m_tree.getLeafCount());
    }

    if (s_showAABBInnerNodes) {
      push.flags = 1; // inner nodes
      pass.getDrawContext().updatePushConstants(push, 0);
      pass.getDrawContext().draw(24, m_tree.getInnerNodeCount());
    }
  }
}

} // namespace RibCage