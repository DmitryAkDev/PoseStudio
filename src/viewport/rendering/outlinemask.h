/**
 * @file outlinemask.h
 * @brief The selection-outline mask: a screen-sized single-channel coverage target the selected
 *        object's silhouette is rendered into, sampled by the composite to draw the outline.
 *
 * The selected model is drawn (skinned, camera-projected, no depth, no materials) into an R8
 * coverage image — multisampled at the context's MSAA count and resolved, so the mask's edge
 * carries the same fractional coverage the scene's anti-aliased silhouette does. The composite
 * pass (composite.frag) dilates that mask by the outline width and paints the theme's selection
 * blue where the dilation exceeds the coverage: outside the object, hugging its silhouette, with
 * the inner edge blending against the object's own anti-aliased fringe. Depth-independent by
 * design — the outline traces the object's whole projected silhouette even where another model
 * occludes it, like the joints, which are pickable through the mesh.
 *
 * Screen-sized, so the renderer resizes it with the swapchain (resize() rebuilds images +
 * framebuffer; the render pass depends only on the format and is kept, so the scene's mask
 * pipeline never rebuilds). At creation and after every resize the sampled image is moved to
 * the read layout, so the composite can always bind it — the mask pass runs only while something
 * is selected, and the composite ignores the (stale) contents when nothing is. Pure Vulkan +
 * std, no Qt.
 */

#ifndef OUTLINEMASK_H
#define OUTLINEMASK_H

#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <cstdint>

namespace pose {

class VulkanContext;

class OutlineMask {
public:
    OutlineMask(VulkanContext& context, VkExtent2D extent);
    ~OutlineMask();

    OutlineMask(const OutlineMask&) = delete;
    OutlineMask& operator=(const OutlineMask&) = delete;

    /// Recreates the images/framebuffer at @p extent (the render pass is kept). The caller must
    /// have made the device idle (the renderer's resize path already does).
    void resize(VkExtent2D extent);

    VkRenderPass  renderPass() const { return m_renderPass; }
    VkFramebuffer framebuffer() const { return m_framebuffer; }
    VkExtent2D    extent() const { return m_extent; }

    /// Number of attachments in the pass (2 with MSAA: coverage + resolve; 1 without) — the
    /// render-pass-begin clear array must cover them.
    uint32_t attachmentCount() const { return m_attachmentCount; }

    /// The resolved (single-sample) coverage image, in SHADER_READ_ONLY layout, through a linear
    /// clamp-to-edge sampler — the composite's outline binding.
    VkDescriptorImageInfo descriptorInfo() const;

    static constexpr VkFormat kFormat = VK_FORMAT_R8_UNORM;

private:
    void createImages();
    void destroyImages();

    VulkanContext& m_context;
    VkExtent2D     m_extent{};
    VkRenderPass   m_renderPass = VK_NULL_HANDLE;
    VkSampler      m_sampler = VK_NULL_HANDLE;
    uint32_t       m_attachmentCount = 0;

    // MSAA coverage (transient) + its single-sample resolve; single-sample fallback: the
    // coverage image itself is the sampleable result.
    VkImage       m_colorImage = VK_NULL_HANDLE;
    VmaAllocation m_colorAlloc = VK_NULL_HANDLE;
    VkImageView   m_colorView = VK_NULL_HANDLE;
    VkImage       m_resolveImage = VK_NULL_HANDLE;
    VmaAllocation m_resolveAlloc = VK_NULL_HANDLE;
    VkImageView   m_resolveView = VK_NULL_HANDLE;
    VkFramebuffer m_framebuffer = VK_NULL_HANDLE;
};

} // namespace pose

#endif // OUTLINEMASK_H
