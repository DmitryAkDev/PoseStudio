/**
 * @file postprocess.h
 * @brief The post-processing chain over the HDR scene target: bloom + the fullscreen composite
 *        that tonemaps into the swapchain.
 *
 * Bloom is the classic three small passes at half resolution — bright-extract (soft threshold on
 * the resolved HDR), then a separable Gaussian blur (H into a second target, V back into the
 * first) — and the composite adds it over the HDR image and applies ACES (PBR mode with the
 * tonemap dial on; every other mode passes through untouched, since the stylized modes still
 * author display-ready values). The composite draws through the existing swapchain MSAA render
 * pass (a fully-covered fullscreen triangle — MSAA is a no-op on it), so VulkanSwapchain needed
 * no changes. All four passes share one fullscreen-triangle vertex shader. Resolution-dependent
 * images/descriptors rebuild on resize(); the render pass + pipelines are format-only and stay.
 * Pure Vulkan + std, no Qt.
 */

#ifndef POSTPROCESS_H
#define POSTPROCESS_H

#include "vulkancommon.h" // kSelectionAccentLinear

#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <memory>
#include <vector>

namespace pose {

class VulkanContext;
class VulkanPipeline;

class PostProcess {
public:
    /// @param swapchainRenderPass The pass the composite draws through (the swapchain's MSAA pass).
    /// @param hdrResolve          The HDR target's resolved-texture descriptor (re-fed on resize).
    /// @param hdrSpecResolve      The HDR target's resolved SPECULAR descriptor — sampled by the
    ///                            SSS V-pass, which adds it back after diffusing (the blur must
    ///                            never smear glints; smeared specular reads as wet skin).
    /// @param outlineMask         The selection-outline coverage mask (OutlineMask) the composite
    ///                            dilates into the outline (re-fed on resize).
    PostProcess(VulkanContext& context, VkRenderPass swapchainRenderPass,
                const VkDescriptorImageInfo& hdrResolve,
                const VkDescriptorImageInfo& hdrSpecResolve,
                const VkDescriptorImageInfo& outlineMask, VkExtent2D extent,
                const std::vector<char>& fullscreenVertSpirv,
                const std::vector<char>& brightFragSpirv, const std::vector<char>& blurFragSpirv,
                const std::vector<char>& compositeFragSpirv,
                const std::vector<char>& sssBlurFragSpirv);
    ~PostProcess();

    PostProcess(const PostProcess&) = delete;
    PostProcess& operator=(const PostProcess&) = delete;

    /// Rebuilds the half-res bloom targets + all descriptor sets for a new size / recreated HDR
    /// target and outline mask. Caller must have made the device idle.
    void resize(VkExtent2D extent, const VkDescriptorImageInfo& hdrResolve,
                const VkDescriptorImageInfo& hdrSpecResolve,
                const VkDescriptorImageInfo& outlineMask);

    /// Records the screen-space subsurface scattering blur: two full-res masked passes, HDR
    /// resolve -> scratch -> back into the HDR resolve (so everything downstream — bloom, the
    /// composite — just sees the diffused image). Call right after the HDR scene pass, PBR only.
    void recordSss(VkCommandBuffer cmd);

    /// Records the bloom chain (bright -> blur H -> blur V), three small render passes at half
    /// resolution. Call between the HDR scene pass and the swapchain pass, PBR mode only.
    void recordBloom(VkCommandBuffer cmd);

    /// Records the fullscreen composite draw. The caller has begun the swapchain render pass and
    /// set the full-extent viewport/scissor. @p tonemap applies ACES (PBR + dial on); @p bloom
    /// adds the bloom target (PBR only — must match whether recordBloom ran this frame);
    /// @p outlineWidthPx > 0 paints the selection outline that wide (physical pixels) around
    /// the coverage in the outline mask — pass 0 whenever the mask pass didn't run this frame
    /// (the mask's contents are then stale).
    void recordComposite(VkCommandBuffer cmd, bool tonemap, bool bloom, float outlineWidthPx);

    /// The selection outline's colour, LINEAR rgb. Defaults to the app's accent blue.
    void setOutlineColor(float r, float g, float b) { m_outlineColor[0] = r; m_outlineColor[1] = g; m_outlineColor[2] = b; }

private:
    struct BloomTarget {
        VkImage       image = VK_NULL_HANDLE;
        VmaAllocation alloc = VK_NULL_HANDLE;
        VkImageView   view = VK_NULL_HANDLE;
        VkFramebuffer framebuffer = VK_NULL_HANDLE;
    };

    void createTargets();
    void destroyTargets();
    void updateDescriptors(const VkDescriptorImageInfo& hdrResolve,
                           const VkDescriptorImageInfo& hdrSpecResolve,
                           const VkDescriptorImageInfo& outlineMask);
    void runFullscreenPass(VkCommandBuffer cmd, VkFramebuffer framebuffer, VkExtent2D extent,
                           const VulkanPipeline& pipeline, VkDescriptorSet set,
                           const float params[4]);

    VulkanContext& m_context;
    VkExtent2D     m_extent{};      // full resolution; bloom runs at half
    VkExtent2D     m_bloomExtent{}; // max(extent/2, 1)
    VkImageView    m_hdrResolveView = VK_NULL_HANDLE; // for the SSSSS V pass's framebuffer

    VkRenderPass m_bloomPass = VK_NULL_HANDLE; // single RGBA16F attachment -> SHADER_READ_ONLY
    VkSampler    m_sampler = VK_NULL_HANDLE;   // clamp+linear, shared by the bloom targets
    BloomTarget  m_bloomA; // bright output, then final (post blur-V) bloom
    BloomTarget  m_bloomB; // blur-H intermediate

    // SSSSS: a full-res scratch target for the H pass, and a framebuffer ON the HDR resolve image
    // so the V pass writes the diffused result back in place (the bloom render pass is reused —
    // same format/structure, size-agnostic).
    BloomTarget   m_sssScratch;               // full resolution
    VkFramebuffer m_sssResolveFb = VK_NULL_HANDLE; // targets the HDR resolve view

    // One three-sampler set layout serves every pass (a pass's unused bindings = its binding-0
    // image again, so every declared binding is valid).
    VkDescriptorSetLayout m_setLayout = VK_NULL_HANDLE;
    VkDescriptorPool      m_pool = VK_NULL_HANDLE;
    VkDescriptorSet       m_brightSet = VK_NULL_HANDLE;    // 0 = HDR resolve
    VkDescriptorSet       m_blurHSet = VK_NULL_HANDLE;     // 0 = bloom A
    VkDescriptorSet       m_blurVSet = VK_NULL_HANDLE;     // 0 = bloom B
    VkDescriptorSet       m_compositeSet = VK_NULL_HANDLE; // 0 = HDR resolve, 1 = bloom A, 2 = outline mask
    VkDescriptorSet       m_sssHSet = VK_NULL_HANDLE;      // 0 = HDR resolve
    VkDescriptorSet       m_sssVSet = VK_NULL_HANDLE;      // 0 = SSS scratch, 1 = spec resolve

    // Selection-outline colour (linear rgb), pushed to the composite. Default: the app's accent
    // blue (kSelectionAccentLinear) — the composite paints it after tonemapping and the sRGB
    // swapchain re-encodes it exactly.
    float m_outlineColor[3] = {kSelectionAccentLinear[0], kSelectionAccentLinear[1],
                               kSelectionAccentLinear[2]};

    std::unique_ptr<VulkanPipeline> m_brightPipeline; // bloom pass
    std::unique_ptr<VulkanPipeline> m_blurPipeline;   // bloom pass (direction via push constant)
    std::unique_ptr<VulkanPipeline> m_sssPipeline;    // SSSSS masked blur (bloom pass, full res)
    std::unique_ptr<VulkanPipeline> m_compositePipeline; // swapchain pass
};

} // namespace pose

#endif // POSTPROCESS_H
