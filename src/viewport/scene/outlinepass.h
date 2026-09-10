/**
 * @file outlinepass.h
 * @brief The selection-outline mask pass: the selected model's silhouette into the OutlineMask.
 *
 * The outline the viewport draws around the selected model is a screen-space effect on a
 * coverage mask, not geometry: this pass renders the selected model — skinned and projected by
 * the camera, through the shadow pass's position-only vertex shader — as constant 1.0 into the
 * renderer's OutlineMask (an R8 target at the context's MSAA count with a single-sample resolve,
 * no depth: the mask is the object's whole projected silhouette, so the outline shows through
 * occluders like the joints pick through the mesh). The composite dilates it into the outline.
 * Runs only while something is selected; the Scene decides that and hands the model in.
 * Qt-free (Vulkan + std + GLM).
 */

#ifndef OUTLINEPASS_H
#define OUTLINEPASS_H

#include <vulkan/vulkan.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace pose {

class Camera;
class Model;
class OutlineMask;
class VulkanContext;
class VulkanPipeline;

class OutlinePass {
public:
    /// Builds the mask pipeline against @p maskPass (the OutlineMask's render pass) from
    /// shadow.vert (@p vertSpirv — its push matrix is a generic viewProj: the light's in the
    /// shadow pass, the camera's here) + outlinemask.frag (@p fragSpirv). Its only descriptor
    /// set is the per-model POSE layout (@p poseSetLayout, at index 0).
    OutlinePass(VulkanContext& context, VkRenderPass maskPass, VkDescriptorSetLayout poseSetLayout,
                const std::vector<char>& vertSpirv, const std::vector<char>& fragSpirv);
    ~OutlinePass();

    OutlinePass(const OutlinePass&) = delete;
    OutlinePass& operator=(const OutlinePass&) = delete;

    /// Records the pass — begins @p mask's render pass, draws @p selected's silhouette through
    /// @p camera, ends it. Returns true (the composite then draws the outline).
    bool record(VkCommandBuffer cmd, const Camera& camera, uint32_t frameIndex,
                const OutlineMask& mask, Model& selected);

private:
    std::unique_ptr<VulkanPipeline> m_pipeline;
};

} // namespace pose

#endif // OUTLINEPASS_H
