/**
 * @file vulkanrenderer.cpp
 * @brief Implementation of the per-frame rendering loop. See vulkanrenderer.h.
 */

#include "vulkanrenderer.h"

#include "hdrtarget.h"
#include "outlinemask.h"
#include "postprocess.h"
#include "shaderlibrary.h"
#include "vulkancontext.h"
#include "vulkanswapchain.h"

#include "grid.h"
#include "scene.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>

namespace pose {

namespace {

UniqueSemaphore makeSemaphore(VkDevice device) {
    VkSemaphoreCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    VkSemaphore semaphore = VK_NULL_HANDLE;
    VK_CHECK(vkCreateSemaphore(device, &info, nullptr, &semaphore));
    return UniqueSemaphore(device, semaphore);
}

UniqueFence makeSignalledFence(VkDevice device) {
    VkFenceCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    info.flags = VK_FENCE_CREATE_SIGNALED_BIT; // so the first wait doesn't block forever
    VkFence fence = VK_NULL_HANDLE;
    VK_CHECK(vkCreateFence(device, &info, nullptr, &fence));
    return UniqueFence(device, fence);
}

} // namespace

VulkanRenderer::VulkanRenderer(VulkanContext& context, VkExtent2D initialExtent,
                               std::string shaderDir)
    : m_context(context), m_windowExtent(initialExtent) {
    // Every compiled shader is read up front: a missing .spv fails here with one clear message
    // (the usual cause is the shader-compile build step not having run) rather than partway
    // through pipeline construction. The blobs are only needed while the pipelines are built.
    ShaderLibrary shaders(std::move(shaderDir));
    shaders.preload();

    m_swapchain = std::make_unique<VulkanSwapchain>(m_context, m_windowExtent);

    // The scene renders offscreen into the HDR target (linear RGBA16F at the context's MSAA
    // count, resolved); scene pipelines are built against ITS render pass. The swapchain pass —
    // single-sample, no depth — then only ever runs the tonemapping composite.
    m_hdrTarget = std::make_unique<HdrTarget>(m_context, m_swapchain->extent());
    // The selection-outline mask is screen-sized too; the Scene's silhouette pipeline builds
    // against its pass, so it exists before the Scene.
    m_outlineMask = std::make_unique<OutlineMask>(m_context, m_swapchain->extent());

    m_scene = std::make_unique<Scene>(m_context, m_hdrTarget->renderPass(),
                                      m_outlineMask->renderPass(),
                                      shaders.get("mesh.vert"), shaders.get("mesh.frag"),
                                      shaders.get("skeleton.vert"), shaders.get("skeleton.frag"),
                                      shaders.get("shadow.vert"), shaders.get("shadow.frag"),
                                      shaders.get("background.vert"),
                                      shaders.get("background.frag"),
                                      shaders.get("outlinemask.frag"));

    // The grid samples the scene's shadow map (ground/contact shadow) through the scene-wide
    // set 3, so its pipeline is built against that layout — Scene must exist first.
    m_grid = std::make_unique<Grid>(m_context, m_hdrTarget->renderPass(),
                                    shaders.get("grid.vert"), shaders.get("grid.frag"),
                                    m_scene->iblSetLayout());

    m_postProcess = std::make_unique<PostProcess>(
        m_context, m_swapchain->renderPass(), m_hdrTarget->resolveInfo(),
        m_hdrTarget->specResolveInfo(), m_outlineMask->descriptorInfo(), m_swapchain->extent(),
        shaders.get("fullscreen.vert"), shaders.get("bloombright.frag"),
        shaders.get("bloomblur.frag"), shaders.get("composite.frag"),
        shaders.get("sssblur.frag"));

    createCommandPool();
    createCommandBuffers();
    createSyncObjects();

    const VkExtent2D extent = m_swapchain->extent();
    m_camera.setViewportSize(static_cast<float>(extent.width), static_cast<float>(extent.height));
}

VulkanRenderer::~VulkanRenderer() {
    // Nothing in flight may reference these objects when we destroy them. The sync objects and
    // the command pool are RAII members, released after this body; the surface-dependent objects
    // are released here in dependency order (consumers before the targets/passes they sample or
    // were built against).
    vkDeviceWaitIdle(m_context.device());

    m_postProcess.reset();
    m_grid.reset();
    m_scene.reset();
    m_outlineMask.reset();
    m_hdrTarget.reset();
    m_swapchain.reset();
}

void VulkanRenderer::notifyResize(VkExtent2D newExtent) {
    // Expose events report the current size too — only a real change needs a swapchain rebuild,
    // otherwise every un-obscure of the window would trigger a device-idle + full rebuild.
    if (newExtent.width == m_windowExtent.width && newExtent.height == m_windowExtent.height) {
        return;
    }
    m_windowExtent = newExtent;
    m_framebufferResized = true;
}

void VulkanRenderer::addModel(const ModelData& data) {
    m_scene->addModel(data);
}

void VulkanRenderer::deleteModel(std::size_t index) {
    // The model's buffers and descriptor sets may be referenced by command buffers still in
    // flight; wait for the device to finish before the scene frees them.
    vkDeviceWaitIdle(m_context.device());
    m_scene->removeModel(index);
}

bool VulkanRenderer::frameSelected() {
    glm::vec3 mn;
    glm::vec3 mx;
    if (!m_scene || !m_scene->framingBounds(mn, mx)) {
        return false;
    }
    // The box's bounding sphere: fits from every orbit angle, so the frame never clips a corner
    // as the user orbits on from it.
    const glm::vec3 center = 0.5f * (mn + mx);
    const float radius = std::max(0.5f * glm::length(mx - mn), 0.05f);
    m_camera.frame(center, radius);
    return true;
}

void VulkanRenderer::applyBakedEnvironment(const BakedEnvironment& baked) {
    if (m_scene) {
        m_scene->applyBakedEnvironment(baked);
    }
}

void VulkanRenderer::createCommandPool() {
    VkCommandPoolCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    ci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT; // we reset+rerecord each frame
    ci.queueFamilyIndex = m_context.graphicsFamily();
    VkCommandPool pool = VK_NULL_HANDLE;
    VK_CHECK(vkCreateCommandPool(m_context.device(), &ci, nullptr, &pool));
    m_commandPool.reset(m_context.device(), pool);
}

void VulkanRenderer::createCommandBuffers() {
    m_commandBuffers.resize(kMaxFramesInFlight);
    VkCommandBufferAllocateInfo ai{};
    ai.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    ai.commandPool = m_commandPool.get();
    ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = static_cast<uint32_t>(m_commandBuffers.size());
    VK_CHECK(vkAllocateCommandBuffers(m_context.device(), &ai, m_commandBuffers.data()));
}

void VulkanRenderer::createSyncObjects() {
    VkDevice device = m_context.device();
    m_imageAvailableSemaphores.clear();
    m_inFlightFences.clear();
    m_renderFinishedSemaphores.clear();
    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        m_imageAvailableSemaphores.push_back(makeSemaphore(device));
        m_inFlightFences.push_back(makeSignalledFence(device));
    }
    for (uint32_t i = 0; i < m_swapchain->imageCount(); ++i) {
        m_renderFinishedSemaphores.push_back(makeSemaphore(device));
    }
}

void VulkanRenderer::recreateSwapchain() {
    // Skip while minimised — a zero-sized swapchain is invalid; we'll rebuild once the
    // window has real extent again.
    if (m_windowExtent.width == 0 || m_windowExtent.height == 0) {
        return;
    }
    // The cached Qt extent can lag the surface: if the window was minimised between an update
    // request and this rebuild (acquire/present returned OUT_OF_DATE while m_windowExtent is
    // still the pre-minimise size), the SURFACE reports 0x0 — and creating a swapchain with a
    // zero imageExtent is a spec violation. Check the authoritative source before rebuilding.
    VkSurfaceCapabilitiesKHR caps{};
    if (vkGetPhysicalDeviceSurfaceCapabilitiesKHR(m_context.physicalDevice(), m_context.surface(),
                                                  &caps) == VK_SUCCESS &&
        caps.currentExtent.width != std::numeric_limits<uint32_t>::max() &&
        (caps.currentExtent.width == 0 || caps.currentExtent.height == 0)) {
        return; // surface currently has no area; the next real resize/expose rebuilds
    }
    vkDeviceWaitIdle(m_context.device());

    const uint32_t oldImageCount = m_swapchain->imageCount();
    m_swapchain->recreate(m_windowExtent);

    // The per-image renderFinished semaphores must match the (possibly new) image count.
    if (m_swapchain->imageCount() != oldImageCount) {
        m_renderFinishedSemaphores.clear();
        for (uint32_t i = 0; i < m_swapchain->imageCount(); ++i) {
            m_renderFinishedSemaphores.push_back(makeSemaphore(m_context.device()));
        }
    }

    const VkExtent2D extent = m_swapchain->extent();
    // The offscreen HDR target, the outline mask, and the bloom targets track the swapchain size
    // (the device is idle here).
    m_hdrTarget->resize(extent);
    m_outlineMask->resize(extent);
    m_postProcess->resize(extent, m_hdrTarget->resolveInfo(), m_hdrTarget->specResolveInfo(),
                          m_outlineMask->descriptorInfo());
    m_camera.setViewportSize(static_cast<float>(extent.width), static_cast<float>(extent.height));
    m_framebufferResized = false;
}

bool VulkanRenderer::drawFrame() {
    if (m_windowExtent.width == 0 || m_windowExtent.height == 0) {
        return false; // minimised: nothing to draw (the next expose/resize re-arms drawing)
    }

    VkDevice device = m_context.device();
    const VkFence inFlight = m_inFlightFences[m_currentFrame].get();
    const VkSemaphore imageAvailable = m_imageAvailableSemaphores[m_currentFrame].get();
    VK_CHECK(vkWaitForFences(device, 1, &inFlight, VK_TRUE, UINT64_MAX));

    uint32_t imageIndex = 0;
    VkResult acquire = vkAcquireNextImageKHR(device, m_swapchain->handle(), UINT64_MAX,
                                             imageAvailable, VK_NULL_HANDLE, &imageIndex);
    if (acquire == VK_ERROR_OUT_OF_DATE_KHR) {
        recreateSwapchain();
        return true; // this frame was skipped; the caller must schedule one at the new size
    }
    if (acquire != VK_SUCCESS && acquire != VK_SUBOPTIMAL_KHR) {
        VK_CHECK(acquire);
    }

    VkCommandBuffer cmd = m_commandBuffers[m_currentFrame];
    VK_CHECK(vkResetCommandBuffer(cmd, 0));
    recordCommandBuffer(cmd, imageIndex);

    // Reset the fence only once we're committing to a submit that will re-signal it: an early-out
    // above — or a throw during recording — must leave it signalled, because an unsignalled fence
    // with no pending submit deadlocks the next frame's infinite vkWaitForFences.
    VK_CHECK(vkResetFences(device, 1, &inFlight));

    const VkSemaphore renderFinished = m_renderFinishedSemaphores[imageIndex].get();
    const VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.waitSemaphoreCount = 1;
    submit.pWaitSemaphores = &imageAvailable;
    submit.pWaitDstStageMask = &waitStage;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &cmd;
    submit.signalSemaphoreCount = 1;
    submit.pSignalSemaphores = &renderFinished;
    VK_CHECK(vkQueueSubmit(m_context.graphicsQueue(), 1, &submit, inFlight));

    VkSwapchainKHR swapchain = m_swapchain->handle();
    VkPresentInfoKHR present{};
    present.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    present.waitSemaphoreCount = 1;
    present.pWaitSemaphores = &renderFinished;
    present.swapchainCount = 1;
    present.pSwapchains = &swapchain;
    present.pImageIndices = &imageIndex;

    bool needsRedraw = false;
    VkResult presentResult = vkQueuePresentKHR(m_context.presentQueue(), &present);
    if (presentResult == VK_ERROR_OUT_OF_DATE_KHR || presentResult == VK_SUBOPTIMAL_KHR ||
        m_framebufferResized) {
        recreateSwapchain();
        needsRedraw = true; // the presented frame predates the rebuild — draw once at the new size
    } else if (presentResult != VK_SUCCESS) {
        VK_CHECK(presentResult);
    }

    m_currentFrame = (m_currentFrame + 1) % kMaxFramesInFlight;
    return needsRedraw;
}

/**
 * @brief THE FRAME GRAPH: records every pass of one frame, in order, into @p cmd.
 *
 *   1. Key-light SHADOW map (Scene::recordShadowPass — depth-only, its own pass on the ShadowMap;
 *      skipped with no casters, the light matrix then parked on an always-lit projection).
 *   2. Selection OUTLINE mask (Scene::recordOutlinePass — the selected model's silhouette into the
 *      OutlineMask; skipped with no selection).
 *   3. Offscreen HDR SCENE pass (HdrTarget, at the context's MSAA count, in LINEAR HDR):
 *      the HDRI backdrop -> opaque meshes -> transparent meshes (+ wire/hidden-line variants per
 *      shade mode) -> the floor grid -> the line overlays (skeleton, pins, ortho floor line);
 *      resolved into the sampleable colour + specular images.
 *   4. Screen-space SSS blur (H into a scratch target, V back into the HDR resolve) and then BLOOM
 *      (half-res bright-extract + separable blur) — PBR mode only.
 *   5. Swapchain pass (single-sample, no depth): the fullscreen COMPOSITE — HDR + bloom, ACES
 *      when tonemapping, the selection outline over the top — into the acquired swapchain image.
 *
 * Every pass's inter-pass synchronisation is expressed by its render pass's external
 * dependencies (renderpassbuilder.h), not by barriers here.
 */
void VulkanRenderer::recordCommandBuffer(VkCommandBuffer cmd, uint32_t imageIndex) {
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    VK_CHECK(vkBeginCommandBuffer(cmd, &begin));

    // Key-light shadow pass first (its own depth-only render pass on the shadow map), so the
    // scene pass below can sample the fresh map. Also fits this frame's light matrix, which
    // Scene::record() writes into the camera UBO.
    m_scene->recordShadowPass(cmd, m_currentFrame);

    // Selection-outline mask (its own small pass, skipped when nothing is selected): the
    // composite dilates it into the outline, so it too must precede the swapchain pass.
    const bool outlined = m_scene->recordOutlinePass(cmd, m_camera, m_currentFrame, *m_outlineMask);

    const VkExtent2D extent = m_swapchain->extent();

    VkViewport viewport{};
    viewport.width = static_cast<float>(extent.width);
    viewport.height = static_cast<float>(extent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    VkRect2D scissor{};
    scissor.extent = extent;

    // --- Offscreen HDR scene pass: backdrop + meshes + grid, in LINEAR HDR. --------------------
    // Five clear slots cover every attachment layout the HDR target uses (MSAA:
    // colourMS/depth/resolve/specMS/specResolve; single-sample: colour/depth/spec) — the
    // SPECULAR attachments clear to zero (no specular where nothing draws). The swapchain pass
    // below needs none: its one attachment loads DONT_CARE.
    std::array<VkClearValue, 5> clears{};
    // Viewport background #3E4042 = (62, 64, 66) as LINEAR values (sRGB->linear of each channel).
    // The HDR target is a linear format and the composite's sRGB swapchain store does the final
    // encode, so the displayed pixel comes back out as exactly #3E4042. Alpha clears to 0: the
    // HDR target's alpha is the SSS mask, and empty pixels are not skin.
    clears[0].color = {{0.04816f, 0.05125f, 0.05447f, 0.0f}};
    clears[1].depthStencil = {1.0f, 0};
    clears[2].color = {{0.0f, 0.0f, 0.0f, 0.0f}}; // single-sample: the spec attachment
    clears[3].color = {{0.0f, 0.0f, 0.0f, 0.0f}}; // MSAA: the spec MS attachment
    clears[4].color = {{0.0f, 0.0f, 0.0f, 0.0f}};

    VkRenderPassBeginInfo scenePass{};
    scenePass.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    scenePass.renderPass = m_hdrTarget->renderPass();
    scenePass.framebuffer = m_hdrTarget->framebuffer();
    scenePass.renderArea.extent = extent;
    scenePass.clearValueCount = m_hdrTarget->attachmentCount();
    scenePass.pClearValues = clears.data();
    vkCmdBeginRenderPass(cmd, &scenePass, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    // Imported meshes first (opaque, depth test+write), then the grid — a transparent overlay
    // that depth-tests against scene geometry and blends; the scene's set 3 + light matrix +
    // shadow dials feed its PCSS ground shadow.
    m_scene->record(cmd, m_camera, m_currentFrame);
    const LightingSettings& lighting = m_scene->lightingSettings();
    m_grid->record(cmd, m_camera, m_scene->iblSet(), m_scene->lightViewProj(),
                   glm::vec4(lighting.shadowIntensity, lighting.shadowSoftness,
                             lighting.shadowReach, 0.0f));
    vkCmdEndRenderPass(cmd);

    // --- SSSSS then bloom (PBR mode only — the stylized modes author display-ready values). ----
    const bool pbr = m_scene->isPbr();
    if (pbr) {
        m_postProcess->recordSss(cmd);   // diffuses skin in place (HDR alpha = the SSS mask)
        m_postProcess->recordBloom(cmd); // then blooms the diffused image
    }

    // --- Swapchain pass: the fullscreen composite (tonemap + bloom add + outline). -------------
    // No clear values: the pass's single colour attachment loads DONT_CARE, since the composite's
    // fullscreen triangle overwrites every pixel.
    VkRenderPassBeginInfo presentPass{};
    presentPass.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    presentPass.renderPass = m_swapchain->renderPass();
    presentPass.framebuffer = m_swapchain->framebuffer(imageIndex);
    presentPass.renderArea.extent = extent;
    presentPass.clearValueCount = 0;
    presentPass.pClearValues = nullptr;
    vkCmdBeginRenderPass(cmd, &presentPass, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    vkCmdSetScissor(cmd, 0, 1, &scissor);
    // The outline's width is 2 logical pixels — Blender's default outline weight — scaled to
    // physical pixels by the window's device pixel ratio.
    constexpr float kOutlineWidthLogicalPx = 2.0f;
    m_postProcess->recordComposite(cmd, pbr && m_scene->lightingSettings().tonemap, pbr,
                                   outlined ? kOutlineWidthLogicalPx * m_uiScale : 0.0f);
    vkCmdEndRenderPass(cmd);

    VK_CHECK(vkEndCommandBuffer(cmd));
}

} // namespace pose
