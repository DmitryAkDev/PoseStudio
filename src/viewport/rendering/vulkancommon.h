/**
 * @file vulkancommon.h
 * @brief Small, header-only utilities shared across every Vulkan module.
 *
 * Deliberately tiny: just the things every rendering file needs (error checking,
 * a human-readable VkResult string, the global frames-in-flight constant). Keep
 * heavyweight helpers in their own translation units so this stays cheap to include.
 *
 * NOTE: This subsystem links the Vulkan loader directly (CMake's Vulkan::Vulkan),
 * so we call vkXxx functions normally. QVulkanInstance is used only to create the
 * VkInstance and the per-window VkSurfaceKHR — see VulkanContext / VulkanWindow.
 */

#ifndef VULKANCOMMON_H
#define VULKANCOMMON_H

#include <vulkan/vulkan.h>

#include <stdexcept>
#include <string>

namespace pose {

/// How many frames the CPU is allowed to record ahead of the GPU. Two gives good
/// overlap without the input latency of triple-buffering the command stream.
inline constexpr int kMaxFramesInFlight = 2;

/// The viewport's selection accent: the app's QSS accent blue #5b87cc (hover borders, the
/// DragNumberBox glyphs), sRGB-decoded to LINEAR — the swapchain's sRGB store re-encodes it
/// exactly. The one colour for everything the engine draws as "selected" (today the selection
/// outline in PostProcess). The darker selection FILL blue (#314D7A) is deliberately not used
/// here: against the viewport grey it doesn't read as a line.
inline constexpr float kSelectionAccentLinear[3] = {0.1046f, 0.2423f, 0.6038f};

/// Maps a VkResult to its enum name for diagnostics. Only the codes this app can
/// realistically hit are spelled out; anything else falls back to the raw integer.
inline std::string vkResultString(VkResult result) {
    switch (result) {
        case VK_SUCCESS:                        return "VK_SUCCESS";
        case VK_NOT_READY:                      return "VK_NOT_READY";
        case VK_TIMEOUT:                        return "VK_TIMEOUT";
        case VK_SUBOPTIMAL_KHR:                 return "VK_SUBOPTIMAL_KHR";
        case VK_ERROR_OUT_OF_HOST_MEMORY:       return "VK_ERROR_OUT_OF_HOST_MEMORY";
        case VK_ERROR_OUT_OF_DEVICE_MEMORY:     return "VK_ERROR_OUT_OF_DEVICE_MEMORY";
        case VK_ERROR_INITIALIZATION_FAILED:    return "VK_ERROR_INITIALIZATION_FAILED";
        case VK_ERROR_DEVICE_LOST:              return "VK_ERROR_DEVICE_LOST";
        case VK_ERROR_SURFACE_LOST_KHR:         return "VK_ERROR_SURFACE_LOST_KHR";
        case VK_ERROR_OUT_OF_DATE_KHR:          return "VK_ERROR_OUT_OF_DATE_KHR";
        case VK_ERROR_EXTENSION_NOT_PRESENT:    return "VK_ERROR_EXTENSION_NOT_PRESENT";
        case VK_ERROR_FEATURE_NOT_PRESENT:      return "VK_ERROR_FEATURE_NOT_PRESENT";
        case VK_ERROR_INCOMPATIBLE_DRIVER:      return "VK_ERROR_INCOMPATIBLE_DRIVER";
        default:                                return "VkResult(" + std::to_string(static_cast<int>(result)) + ")";
    }
}

/// Thrown by VK_CHECK on any non-success VkResult. Caught at the VulkanWindow boundaries —
/// initializeVulkan() for setup failures, renderFrame() and the environment-bake apply for
/// runtime failures (e.g. device lost) — each of which tears the renderer down and reports
/// the failure rather than letting the exception escape Qt's event loop and abort the app.
class VulkanError : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/**
 * @brief Aborts the current operation (by throwing VulkanError) if @p result is not
 *        VK_SUCCESS, tagging the message with the call site.
 *
 * Use the VK_CHECK macro rather than calling this directly so __FILE__/__LINE__ are
 * captured automatically.
 */
inline void vkCheckImpl(VkResult result, const char* expr, const char* file, int line) {
    if (result != VK_SUCCESS) {
        throw VulkanError(std::string("Vulkan call failed: ") + expr + " -> " +
                          vkResultString(result) + " (" + file + ":" + std::to_string(line) + ")");
    }
}

} // namespace pose

// NOLINTNEXTLINE(cppcoreguidelines-macro-usage): a macro is the only way to capture the call site.
#define VK_CHECK(expr) ::pose::vkCheckImpl((expr), #expr, __FILE__, __LINE__)

#endif // VULKANCOMMON_H
