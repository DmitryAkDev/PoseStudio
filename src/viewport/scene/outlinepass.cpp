/**
 * @file outlinepass.cpp
 * @brief The selection-outline mask pass. See outlinepass.h.
 */

#include "outlinepass.h"

#include "camera.h"
#include "mesh.h" // ShadowPushConstants
#include "model.h"
#include "outlinemask.h"
#include "vertex.h"
#include "vulkanpipeline.h"

#include <array>

namespace pose {

OutlinePass::OutlinePass(VulkanContext& context, VkRenderPass maskPass,
                         VkDescriptorSetLayout poseSetLayout, const std::vector<char>& vertSpirv,
                         const std::vector<char>& fragSpirv) {
    // Selection-outline mask: the shadow pass's skinned position-only vertex shader again, now
    // projected by the camera into the OutlineMask's coverage pass (its MSAA count matches the
    // context's, so the silhouette edge resolves to fractional coverage), writing constant 1.0.
    // No depth attachment in that pass — the mask is the object's whole projected silhouette.
    PipelineConfig outlineConfig;
    outlineConfig.pushConstantSize = sizeof(ShadowPushConstants);
    outlineConfig.pushConstantStages = VK_SHADER_STAGE_VERTEX_BIT;
    outlineConfig.depthTestEnable = false;
    outlineConfig.depthWriteEnable = false;
    outlineConfig.blendEnable = false;
    const VkVertexInputBindingDescription binding = Vertex::bindingDescription();
    const auto attrs = Vertex::attributeDescriptions();
    outlineConfig.vertexBindings.assign(1, binding);
    outlineConfig.vertexAttributes.assign(attrs.begin(), attrs.end());
    outlineConfig.descriptorSetLayouts = {poseSetLayout};
    m_pipeline = std::make_unique<VulkanPipeline>(context, maskPass, vertSpirv, fragSpirv,
                                                  outlineConfig);
}

OutlinePass::~OutlinePass() = default;

bool OutlinePass::record(VkCommandBuffer cmd, const Camera& camera, uint32_t frameIndex,
                         const OutlineMask& mask, Model& selected) {
    // Clear to 0 ("not the selected object"); with MSAA the second slot is the resolve target,
    // whose load is DONT_CARE (the clear value is simply unused).
    std::array<VkClearValue, 2> clears{};
    clears[0].color = {{0.0f, 0.0f, 0.0f, 0.0f}};
    clears[1].color = {{0.0f, 0.0f, 0.0f, 0.0f}};
    const VkExtent2D extent = mask.extent();
    VkRenderPassBeginInfo rp{};
    rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rp.renderPass = mask.renderPass();
    rp.framebuffer = mask.framebuffer();
    rp.renderArea.extent = extent;
    rp.clearValueCount = mask.attachmentCount();
    rp.pClearValues = clears.data();
    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport viewport{};
    viewport.width = static_cast<float>(extent.width);
    viewport.height = static_cast<float>(extent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    VkRect2D scissor{};
    scissor.extent = extent;
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipeline->handle());
    selected.recordSilhouette(cmd, m_pipeline->layout(), camera.viewProjection(), frameIndex);
    vkCmdEndRenderPass(cmd);
    return true;
}

} // namespace pose
