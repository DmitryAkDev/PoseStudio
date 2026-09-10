/**
 * @file vulkancontext.h
 * @brief The "device layer": owns the long-lived Vulkan objects that outlive any
 *        single swapchain or frame.
 *
 * One VulkanContext is created per window once a VkSurfaceKHR exists. It picks a
 * physical device with a queue family that can both render and present to that
 * surface, creates the logical device + queues, stands up a VMA allocator for the
 * rest of the subsystem to allocate buffers/images through, and owns the shared
 * SamplerCache (every texture and render target borrows its sampler from it).
 *
 * Everything here is created once and torn down only when the viewport is destroyed.
 * Per-resolution objects (swapchain images/framebuffers, the offscreen targets) live in
 * VulkanSwapchain and the target classes; per-frame objects (command buffers, sync)
 * live in VulkanRenderer.
 */

#ifndef VULKANCONTEXT_H
#define VULKANCONTEXT_H

#include "vulkancommon.h"

#include <vk_mem_alloc.h>

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace pose {

class SamplerCache;

/**
 * @class VulkanContext
 * @brief Logical device, queues, and memory allocator for one rendering surface.
 *
 * Non-copyable, non-movable: it owns raw Vulkan handles whose lifetime is tied to
 * this object. Construct it on the heap and hold it via unique_ptr in VulkanWindow.
 */
class VulkanContext {
public:
    /**
     * @brief Selects a device and brings up queues + the VMA allocator.
     * @param instance       The VkInstance owned by the app's QVulkanInstance.
     * @param surface        The window surface to ensure the queue family can present to.
     * @param apiVersion     Vulkan API version the instance was created with (e.g. VK_API_VERSION_1_2).
     * @throws VulkanError if no suitable device is found or device creation fails.
     */
    VulkanContext(VkInstance instance, VkSurfaceKHR surface, uint32_t apiVersion);
    ~VulkanContext();

    VulkanContext(const VulkanContext&) = delete;
    VulkanContext& operator=(const VulkanContext&) = delete;

    // --- Accessors (handles are owned by this object; callers must not destroy them) ---
    VkSurfaceKHR     surface()         const { return m_surface; }
    VkPhysicalDevice physicalDevice()  const { return m_physicalDevice; }
    VkDevice         device()          const { return m_device; }
    VkQueue          graphicsQueue()   const { return m_graphicsQueue; }
    VkQueue          presentQueue()    const { return m_presentQueue; }
    uint32_t         graphicsFamily()  const { return m_graphicsFamily; }
    VmaAllocator     allocator()       const { return m_allocator; }

    /// The shared sampler cache (see samplercache.h): textures and targets borrow their
    /// VkSampler from here instead of creating one each.
    SamplerCache& samplers() { return *m_samplers; }

    /// The physical device's format properties, queried once per format and cached (the texture
    /// upload path asks for every map whether the format supports linear blits for mip generation).
    const VkFormatProperties& formatProperties(VkFormat format) const;

    /// The MSAA sample count the viewport renders at — min(4x, device max) supported for both colour
    /// and depth attachments. VK_SAMPLE_COUNT_1_BIT when the device can't multisample (then the
    /// offscreen scene target uses its plain single-sample path). Chosen once at device creation.
    VkSampleCountFlagBits sampleCount() const { return m_sampleCount; }

    /// Whether the device rasterizes LINE polygon mode (the fillModeNonSolid feature, enabled at
    /// device creation when present). False on an exotic device: no wireframe pipelines then.
    bool supportsWireframe() const { return m_supportsWireframe; }

    /// Whether the device supports per-attachment blend/write-mask state (the independentBlend
    /// feature, enabled at device creation when present). VulkanPipeline consults it: without
    /// the feature every colour attachment of a pipeline must share one blend state.
    bool supportsIndependentBlend() const { return m_supportsIndependentBlend; }

private:
    void pickPhysicalDevice();
    void createLogicalDevice();
    void createAllocator(uint32_t apiVersion);

    /// Returns true and fills the out-params if @p device has a single queue family
    /// supporting both graphics and presentation to our surface.
    bool findQueueFamilies(VkPhysicalDevice device, uint32_t& graphicsFamily, uint32_t& presentFamily) const;

    VkInstance       m_instance       = VK_NULL_HANDLE; // borrowed (owned by QVulkanInstance)
    VkSurfaceKHR     m_surface        = VK_NULL_HANDLE; // borrowed (owned by QVulkanInstance/window)
    VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE; // not destroyed (implicitly owned by instance)
    VkDevice         m_device         = VK_NULL_HANDLE;
    VkQueue          m_graphicsQueue  = VK_NULL_HANDLE;
    VkQueue          m_presentQueue   = VK_NULL_HANDLE;
    uint32_t         m_graphicsFamily = 0;
    uint32_t         m_presentFamily  = 0; // == m_graphicsFamily today (single-family selection)
    VmaAllocator     m_allocator      = VK_NULL_HANDLE;
    VkSampleCountFlagBits m_sampleCount = VK_SAMPLE_COUNT_1_BIT; // MSAA level (see sampleCount())
    bool             m_supportsWireframe = false;        // fillModeNonSolid enabled (see supportsWireframe())
    bool             m_supportsIndependentBlend = false; // independentBlend enabled (see supportsIndependentBlend())

    std::unique_ptr<SamplerCache> m_samplers; // destroyed explicitly before the device
    mutable std::unordered_map<VkFormat, VkFormatProperties> m_formatProperties; // see formatProperties()
};

} // namespace pose

#endif // VULKANCONTEXT_H
