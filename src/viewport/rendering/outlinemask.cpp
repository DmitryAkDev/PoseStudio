/**
 * @file outlinemask.cpp
 * @brief Implementation of OutlineMask. See outlinemask.h.
 */

#include "outlinemask.h"

#include "vulkancommands.h"
#include "vulkancommon.h"
#include "vulkancontext.h"

#include <array>

namespace pose {

OutlineMask::OutlineMask(VulkanContext& context, VkExtent2D extent)
    : m_context(context), m_extent(extent) {
    VkDevice device = m_context.device();
    const VkSampleCountFlagBits samples = m_context.sampleCount();
    const bool msaa = samples != VK_SAMPLE_COUNT_1_BIT;

    // Render pass: one coverage attachment at the context's MSAA count (cleared to 0 = "not the
    // selected object"), resolved into a single-sample image left SHADER_READ_ONLY for the
    // composite. No depth: the mask is the object's full projected silhouette. Single-sample
    // fallback: the one attachment is stored and sampled directly.
    std::array<VkAttachmentDescription, 2> attachments{};
    attachments[0].format = kFormat;
    attachments[0].samples = samples;
    attachments[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attachments[0].storeOp = msaa ? VK_ATTACHMENT_STORE_OP_DONT_CARE : VK_ATTACHMENT_STORE_OP_STORE;
    attachments[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[0].finalLayout = msaa ? VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL
                                      : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    attachments[1].format = kFormat; // resolve (MSAA only)
    attachments[1].samples = VK_SAMPLE_COUNT_1_BIT;
    attachments[1].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attachments[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attachments[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attachments[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attachments[1].finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    m_attachmentCount = msaa ? 2u : 1u;

    const VkAttachmentReference colorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    const VkAttachmentReference resolveRef{1, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;
    subpass.pResolveAttachments = msaa ? &resolveRef : nullptr;

    // The previous frame's composite read must finish before this pass overwrites the mask, and
    // this pass's writes must land before this frame's composite samples it.
    std::array<VkSubpassDependency, 2> deps{};
    deps[0].srcSubpass = VK_SUBPASS_EXTERNAL;
    deps[0].dstSubpass = 0;
    deps[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    deps[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
    deps[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    deps[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    deps[1].srcSubpass = 0;
    deps[1].dstSubpass = VK_SUBPASS_EXTERNAL;
    deps[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    deps[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    deps[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    deps[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    VkRenderPassCreateInfo rpInfo{};
    rpInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rpInfo.attachmentCount = m_attachmentCount;
    rpInfo.pAttachments = attachments.data();
    rpInfo.subpassCount = 1;
    rpInfo.pSubpasses = &subpass;
    rpInfo.dependencyCount = static_cast<uint32_t>(deps.size());
    rpInfo.pDependencies = deps.data();
    VK_CHECK(vkCreateRenderPass(device, &rpInfo, nullptr, &m_renderPass));

    // Linear filtering gives the composite's dilation taps a soft transition across the mask's
    // edge (the outer edge of the outline); clamp-to-edge means an object cut by the viewport
    // border gets no outline along the cut, the way it has no silhouette there.
    VkSamplerCreateInfo samplerInfo{};
    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samplerInfo.magFilter = VK_FILTER_LINEAR;
    samplerInfo.minFilter = VK_FILTER_LINEAR;
    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    VK_CHECK(vkCreateSampler(device, &samplerInfo, nullptr, &m_sampler));

    createImages();
}

OutlineMask::~OutlineMask() {
    destroyImages();
    VkDevice device = m_context.device();
    if (m_sampler != VK_NULL_HANDLE) {
        vkDestroySampler(device, m_sampler, nullptr);
    }
    if (m_renderPass != VK_NULL_HANDLE) {
        vkDestroyRenderPass(device, m_renderPass, nullptr);
    }
}

void OutlineMask::resize(VkExtent2D extent) {
    m_extent = extent;
    destroyImages();
    createImages();
}

void OutlineMask::createImages() {
    VkDevice device = m_context.device();
    const VkSampleCountFlagBits samples = m_context.sampleCount();
    const bool msaa = samples != VK_SAMPLE_COUNT_1_BIT;

    const auto makeImage = [&](VkSampleCountFlagBits sampleCount, VkImageUsageFlags usage,
                               VkImage& image, VmaAllocation& alloc, VkImageView& view) {
        VkImageCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        info.imageType = VK_IMAGE_TYPE_2D;
        info.format = kFormat;
        info.extent = {m_extent.width, m_extent.height, 1};
        info.mipLevels = 1;
        info.arrayLayers = 1;
        info.samples = sampleCount;
        info.tiling = VK_IMAGE_TILING_OPTIMAL;
        info.usage = usage;
        info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        VmaAllocationCreateInfo allocInfo{};
        allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
        VK_CHECK(vmaCreateImage(m_context.allocator(), &info, &allocInfo, &image, &alloc, nullptr));

        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = kFormat;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.layerCount = 1;
        VK_CHECK(vkCreateImageView(device, &viewInfo, nullptr, &view));
    };

    // MSAA coverage: transient (tile-local on many GPUs) and resolved; single-sample: the
    // coverage image is the sampleable result itself.
    makeImage(samples,
              msaa ? (VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT)
                   : (VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT),
              m_colorImage, m_colorAlloc, m_colorView);
    if (msaa) {
        makeImage(VK_SAMPLE_COUNT_1_BIT, VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                  m_resolveImage, m_resolveAlloc, m_resolveView);
    }

    const VkImageView views[2] = {m_colorView, m_resolveView};
    VkFramebufferCreateInfo fbInfo{};
    fbInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fbInfo.renderPass = m_renderPass;
    fbInfo.attachmentCount = m_attachmentCount;
    fbInfo.pAttachments = views;
    fbInfo.width = m_extent.width;
    fbInfo.height = m_extent.height;
    fbInfo.layers = 1;
    VK_CHECK(vkCreateFramebuffer(device, &fbInfo, nullptr, &m_framebuffer));

    // Move the sampled image to the read layout once: the composite binds it every frame, but
    // the mask pass only runs while something is selected — a fresh image that no pass has
    // rendered yet would otherwise be bound in UNDEFINED layout (a validation violation, and UB
    // on hardware). Construction/resize run on an idle device, so the blocking submit is fine.
    const VkImage sampled = msaa ? m_resolveImage : m_colorImage;
    submitImmediate(m_context, [&](VkCommandBuffer cmd) {
        VkImageMemoryBarrier barrier{};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        barrier.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = sampled;
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.layerCount = 1;
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                             VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1,
                             &barrier);
    });
}

void OutlineMask::destroyImages() {
    VkDevice device = m_context.device();
    if (m_framebuffer != VK_NULL_HANDLE) {
        vkDestroyFramebuffer(device, m_framebuffer, nullptr);
        m_framebuffer = VK_NULL_HANDLE;
    }
    const auto destroy = [&](VkImage& image, VmaAllocation& alloc, VkImageView& view) {
        if (view != VK_NULL_HANDLE) {
            vkDestroyImageView(device, view, nullptr);
            view = VK_NULL_HANDLE;
        }
        if (image != VK_NULL_HANDLE) {
            vmaDestroyImage(m_context.allocator(), image, alloc);
            image = VK_NULL_HANDLE;
            alloc = VK_NULL_HANDLE;
        }
    };
    destroy(m_resolveImage, m_resolveAlloc, m_resolveView);
    destroy(m_colorImage, m_colorAlloc, m_colorView);
}

VkDescriptorImageInfo OutlineMask::descriptorInfo() const {
    VkDescriptorImageInfo info{};
    info.sampler = m_sampler;
    info.imageView = (m_resolveView != VK_NULL_HANDLE) ? m_resolveView : m_colorView;
    info.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    return info;
}

} // namespace pose
