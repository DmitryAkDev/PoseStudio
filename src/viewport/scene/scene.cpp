/**
 * @file scene.cpp
 * @brief Implementation of Scene. See scene.h.
 */

#include "scene.h"

#include "camera.h"
#include "environment.h"
#include "iblmaps.h"
#include "mesh.h"
#include "modeldata.h"
#include "outlinemask.h"
#include "shadowmap.h"
#include "vertex.h"
#include "vulkancommon.h"
#include "vulkancontext.h"
#include "vulkanimage.h"
#include "vulkanpipeline.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp> // lookAt / ortho for the shadow frustum

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace pose {

namespace {

// std140-compatible camera + lighting block. vec3 fields are padded to vec4. Must match the
// `CameraUbo` block declared in mesh.vert / mesh.frag. Layout (std140) must match both shaders.
struct CameraUbo {
    glm::mat4 viewProj;
    glm::mat4 view;          // world -> view (fragment modes derive view-space normals for matcaps)
    glm::mat4 lightViewProj; // key light's ortho view-projection (shadow-map space; see recordShadowPass)
    glm::vec4 cameraPos;     // xyz world-space eye
    glm::vec4 lightDir;    // xyz = normalized direction TO the key light
    glm::vec4 lightColor;  // rgb
    glm::vec4 fillDir;     // fill light (dir TO light)
    glm::vec4 fillColor;   // rgb (intensity baked in)
    glm::vec4 rimDir;      // rim / back light (dir TO light)
    glm::vec4 rimColor;    // rgb (intensity baked in)
    glm::vec4 ambient;     // rgb ambient/fill term
    glm::vec4 params;      // x = shade mode, y = exposure, z = specularIntensity, w = ambientFill
    glm::vec4 sh[9];       // environment diffuse irradiance: 9 SH coefficients (rgb in .xyz)
    glm::vec4 params2;     // x = diffuseIntensity, y = keyIntensity, z = envRotation(rad), w = tonemap(0/1)
    glm::vec4 params3;     // x = subsurface, y = rimIntensity, z = backdropMode, w = backdropBlur
    glm::vec4 params4;     // x = backdropBrightness, y = domeRadius, z = shadowIntensity,
                           // w = the wireframe overlay's linear grey (ShadeMode::wireLevel)
};


// A vertex of the skeleton overlay: a world-space point + its colour. Must match skeleton.vert.
struct LineVertex {
    glm::vec3 pos;
    glm::vec3 color;
};
constexpr uint32_t kMaxSkeletonVerts = 8192; // 2 per bone segment + pin markers; far above any figure's

// Screen-roughly-constant overlay marker radius: a small fraction of the distance to the camera,
// so a joint marker keeps a consistent on-screen size as you dolly in/out.
float markerRadius(const glm::vec3& center, const Camera& camera) {
    return 0.048f * glm::length(center - camera.position());
}

} // namespace

Scene::Scene(VulkanContext& context, VkRenderPass renderPass, VkRenderPass outlineMaskPass,
             const std::vector<char>& vertSpirv, const std::vector<char>& fragSpirv,
             const std::vector<char>& skeletonVertSpirv, const std::vector<char>& skeletonFragSpirv,
             const std::vector<char>& shadowVertSpirv, const std::vector<char>& shadowFragSpirv,
             const std::vector<char>& backgroundVertSpirv,
             const std::vector<char>& backgroundFragSpirv,
             const std::vector<char>& outlineMaskFragSpirv)
    : m_context(context) {
    // The shadow map must exist before createDescriptorResources() — it writes the map into the
    // scene-wide set 3 (binding 2) right after allocating that set.
    m_shadowMap = std::make_unique<ShadowMap>(m_context);
    createDescriptorResources();

    // Bake the default procedural studio environment synchronously so descriptor set 3 is valid before
    // the first frame. A real HDRI is baked off the render thread (VulkanWindow) and swapped in when
    // ready via applyBakedEnvironment().
    applyBakedEnvironment(bakeEnvironment(generateStudioEnvironment()));

    // 1x1 opaque-white fallback texture so untextured meshes can share the same pipeline/shader:
    // sampling white and multiplying by baseColor just yields baseColor.
    const uint8_t white[4] = {255, 255, 255, 255};
    m_fallbackTexture = std::make_unique<VulkanTexture>(m_context, white, 1, 1);
    // 1x1 flat-normal fallback (tangent-space +Z, i.e. no perturbation) for meshes with no detail
    // map. Linear, not sRGB — normal/bump data must be sampled verbatim.
    const uint8_t flatNormal[4] = {128, 128, 255, 255};
    m_fallbackNormal = std::make_unique<VulkanTexture>(m_context, flatNormal, 1, 1, /*srgb=*/false);

    PipelineConfig config;
    config.pushConstantSize = sizeof(MeshPushConstants);
    config.pushConstantStages = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    config.depthTestEnable = true;
    config.depthWriteEnable = true;
    config.blendEnable = false;
    // The HDR pass has a second colour attachment: the SPECULAR target the SSS blur must not
    // smear. Only this opaque pipeline writes it (mesh.frag routes the PBR specular there).
    config.colorAttachmentCount = 2;
    config.writeColorAttachment1 = true;
    // No back-face culling: imported OBJ winding is inconsistent in the wild, so we'd risk an
    // invisible/inside-out model. The fragment shader instead flips the normal per gl_FrontFacing
    // for correct two-sided lighting. Revisit once we normalize winding on import.
    config.cullMode = VK_CULL_MODE_NONE;

    const VkVertexInputBindingDescription binding = Vertex::bindingDescription();
    const auto attrs = Vertex::attributeDescriptions();
    config.vertexBindings.assign(1, binding);
    config.vertexAttributes.assign(attrs.begin(), attrs.end());
    // set 0 = camera, set 1 = material textures, set 2 = per-model skinning joint matrices,
    // set 3 = IBL maps (prefiltered specular cubemap + BRDF LUT).
    config.descriptorSetLayouts = {m_setLayout, m_materialSetLayout, m_jointSetLayout, m_iblSetLayout};

    m_pipeline = std::make_unique<VulkanPipeline>(m_context, renderPass, vertSpirv, fragSpirv, config);

    // Transparent variant for see-through material zones (eye moisture, cornea, later hair/lashes):
    // alpha-blend over what's already there, and keep depth *testing* (so opaque geometry in front
    // still occludes) but disable depth *writes* (so overlapping transparent layers don't reject
    // each other and nothing behind the mostly-clear surface gets occluded). Drawn after the opaque
    // pass. Same shaders/layout — only the blend + depth-write state differs.
    config.blendEnable = true;
    config.depthWriteEnable = false;
    config.colorWriteAlpha = false; // don't clobber the SSS mask under blended shells/cards
    config.writeColorAttachment1 = false; // blended shells keep their specular in-line (mask ≈ 0)
    m_transparentPipeline =
        std::make_unique<VulkanPipeline>(m_context, renderPass, vertSpirv, fragSpirv, config);
    config.colorWriteAlpha = true;

    // Wireframe (the wireframe shade modes): the same shaders and layout rasterized as triangle
    // EDGES (LINE polygon mode), depth-tested and written, pulled a hair toward the camera by a
    // negative depth bias so a surface's own edges win over its coplanar fill in the overlay
    // modes. Alpha and the specular attachment are masked — a wire is neither skin nor a glint.
    // Skipped on a device without fillModeNonSolid; the wire modes then show their surface alone.
    if (m_context.supportsWireframe()) {
        PipelineConfig wireConfig = config; // vertex layout, set layouts, push block as the mesh pass
        wireConfig.blendEnable = false;
        wireConfig.depthTestEnable = true;
        wireConfig.depthWriteEnable = true;
        wireConfig.colorWriteAlpha = false;
        wireConfig.writeColorAttachment1 = false;
        wireConfig.polygonMode = VK_POLYGON_MODE_LINE;
        wireConfig.depthBiasConstant = -1.0f;
        wireConfig.depthBiasSlope = -1.0f;
        m_wirePipeline =
            std::make_unique<VulkanPipeline>(m_context, renderPass, vertSpirv, fragSpirv, wireConfig);
    }

    // Hidden-line surface fill: depth test + write with EVERY colour write masked, so the
    // surface occludes the grid and the wires behind it while the framebuffer keeps the viewport's
    // clear colour — the classic hidden-line drawing once the wire pass draws over it.
    PipelineConfig depthFillConfig = config;
    depthFillConfig.blendEnable = false;
    depthFillConfig.depthTestEnable = true;
    depthFillConfig.depthWriteEnable = true;
    depthFillConfig.colorWriteRgb = false;
    depthFillConfig.colorWriteAlpha = false;
    depthFillConfig.writeColorAttachment1 = false;
    m_hiddenLinePipeline = std::make_unique<VulkanPipeline>(m_context, renderPass, vertSpirv,
                                                            fragSpirv, depthFillConfig);

    // Depth-only shadow pipeline: same skinned vertex layout, rendered into the shadow map's own
    // single-sample pass with a static depth bias (acne) and no colour attachment. Its only
    // descriptor set is the joint-matrix layout (bound at index 0 — see Model::recordShadow).
    PipelineConfig shadowConfig;
    shadowConfig.pushConstantSize = sizeof(ShadowPushConstants);
    shadowConfig.pushConstantStages = VK_SHADER_STAGE_VERTEX_BIT;
    shadowConfig.singleSample = true;
    shadowConfig.hasColorAttachment = false;
    shadowConfig.depthBiasConstant = 1.25f;
    shadowConfig.depthBiasSlope = 1.9f;
    shadowConfig.cullMode = VK_CULL_MODE_NONE; // imported winding is inconsistent (same as main pass)
    shadowConfig.vertexBindings.assign(1, binding);
    shadowConfig.vertexAttributes.assign(attrs.begin(), attrs.end());
    shadowConfig.descriptorSetLayouts = {m_jointSetLayout};
    m_shadowPipeline = std::make_unique<VulkanPipeline>(m_context, m_shadowMap->renderPass(),
                                                        shadowVertSpirv, shadowFragSpirv,
                                                        shadowConfig);

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
    outlineConfig.cullMode = VK_CULL_MODE_NONE;
    outlineConfig.vertexBindings.assign(1, binding);
    outlineConfig.vertexAttributes.assign(attrs.begin(), attrs.end());
    outlineConfig.descriptorSetLayouts = {m_jointSetLayout};
    m_outlinePipeline = std::make_unique<VulkanPipeline>(m_context, outlineMaskPass, shadowVertSpirv,
                                                         outlineMaskFragSpirv, outlineConfig);

    // HDRI backdrop: a fullscreen triangle sampling the environment cube along the eye ray, drawn
    // FIRST in the main pass with depth test/write off so everything else renders over it. Its own
    // two-set layout: 0 = camera UBO, 1 = the scene-wide IBL/shadow set (it samples binding 0).
    PipelineConfig bgConfig;
    bgConfig.depthTestEnable = false;
    bgConfig.depthWriteEnable = false;
    bgConfig.blendEnable = false;
    bgConfig.colorAttachmentCount = 2; // spec target present but masked off (writeColorAttachment1 false)
    bgConfig.descriptorSetLayouts = {m_setLayout, m_iblSetLayout};
    m_backgroundPipeline = std::make_unique<VulkanPipeline>(m_context, renderPass,
                                                            backgroundVertSpirv, backgroundFragSpirv,
                                                            bgConfig);

    // Skeleton overlay: a coloured line list drawn over the figure (depth test OFF, so every joint is
    // visible and pickable through the body). Uses set 0 (camera) only; per-vertex colour.
    PipelineConfig lineConfig;
    lineConfig.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
    lineConfig.depthTestEnable = false;
    lineConfig.depthWriteEnable = false;
    lineConfig.blendEnable = false;
    lineConfig.colorWriteAlpha = false; // overlay lines must not touch the SSS mask
    lineConfig.colorAttachmentCount = 2; // spec target present but masked off
    lineConfig.pushConstantSize = 0;
    VkVertexInputBindingDescription lineBinding{};
    lineBinding.binding = 0;
    lineBinding.stride = sizeof(LineVertex);
    lineBinding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    std::array<VkVertexInputAttributeDescription, 2> lineAttrs{};
    lineAttrs[0].location = 0;
    lineAttrs[0].binding = 0;
    lineAttrs[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    lineAttrs[0].offset = offsetof(LineVertex, pos);
    lineAttrs[1].location = 1;
    lineAttrs[1].binding = 0;
    lineAttrs[1].format = VK_FORMAT_R32G32B32_SFLOAT;
    lineAttrs[1].offset = offsetof(LineVertex, color);
    lineConfig.vertexBindings.assign(1, lineBinding);
    lineConfig.vertexAttributes.assign(lineAttrs.begin(), lineAttrs.end());
    lineConfig.descriptorSetLayouts = {m_setLayout}; // set 0 = camera UBO
    m_skeletonPipeline = std::make_unique<VulkanPipeline>(m_context, renderPass, skeletonVertSpirv,
                                                          skeletonFragSpirv, lineConfig);

    // One overlay vertex buffer per frame-in-flight (see the member comment in scene.h).
    m_skeletonVertexBuffers.reserve(kMaxFramesInFlight);
    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        m_skeletonVertexBuffers.emplace_back(m_context, kMaxSkeletonVerts * sizeof(LineVertex),
                                             VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO,
                                             VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                                 VMA_ALLOCATION_CREATE_MAPPED_BIT);
    }
}

Scene::~Scene() {
    // Nothing in flight may reference the pipeline/descriptors when we tear them down.
    vkDeviceWaitIdle(m_context.device());

    m_models.clear();         // frees mesh buffers, textures, and per-model descriptor pools
    m_pipeline.reset();
    m_transparentPipeline.reset();
    m_skeletonPipeline.reset();
    m_backgroundPipeline.reset();
    m_outlinePipeline.reset();
    m_wirePipeline.reset();
    m_hiddenLinePipeline.reset();
    m_shadowPipeline.reset();
    m_shadowMap.reset();
    m_fallbackTexture.reset();
    m_fallbackNormal.reset();
    m_iblMaps.reset();         // frees the specular cubemap + BRDF LUT images
    m_cameraBuffers.clear();   // VulkanBuffers free themselves
    m_skeletonVertexBuffers.clear();

    VkDevice device = m_context.device();
    if (m_iblPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device, m_iblPool, nullptr); // frees the IBL set too
    }
    if (m_iblSetLayout != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device, m_iblSetLayout, nullptr);
    }
    if (m_descriptorPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(device, m_descriptorPool, nullptr); // frees the camera sets too
    }
    if (m_setLayout != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device, m_setLayout, nullptr);
    }
    if (m_materialSetLayout != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device, m_materialSetLayout, nullptr);
    }
    if (m_jointSetLayout != VK_NULL_HANDLE) {
        vkDestroyDescriptorSetLayout(device, m_jointSetLayout, nullptr);
    }
}

void Scene::createDescriptorResources() {
    VkDevice device = m_context.device();

    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = 1;
    layoutInfo.pBindings = &binding;
    VK_CHECK(vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &m_setLayout));

    // Set 1: six combined image samplers read in the fragment stage — binding 0 = diffuse,
    // binding 1 = detail (normal/bump) map, binding 2 = roughness map, binding 3 = spec-mask map,
    // binding 4 = translucency map, binding 5 = micro-detail (pore) normal map (2/3 are per-texel
    // multipliers on the material scalars, 4 the transmitted tint/strength, 5 a tiled linear
    // normal; absent maps bind the white / flat-normal fallbacks).
    std::array<VkDescriptorSetLayoutBinding, 6> samplerBindings{};
    for (uint32_t i = 0; i < samplerBindings.size(); ++i) {
        samplerBindings[i].binding = i;
        samplerBindings[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        samplerBindings[i].descriptorCount = 1;
        samplerBindings[i].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    }

    VkDescriptorSetLayoutCreateInfo materialLayoutInfo{};
    materialLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    materialLayoutInfo.bindingCount = static_cast<uint32_t>(samplerBindings.size());
    materialLayoutInfo.pBindings = samplerBindings.data();
    VK_CHECK(vkCreateDescriptorSetLayout(device, &materialLayoutInfo, nullptr, &m_materialSetLayout));

    // Set 2: the per-model skinning joint-matrix storage buffer, read in the vertex stage.
    VkDescriptorSetLayoutBinding jointBinding{};
    jointBinding.binding = 0;
    jointBinding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    jointBinding.descriptorCount = 1;
    jointBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

    VkDescriptorSetLayoutCreateInfo jointLayoutInfo{};
    jointLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    jointLayoutInfo.bindingCount = 1;
    jointLayoutInfo.pBindings = &jointBinding;
    VK_CHECK(vkCreateDescriptorSetLayout(device, &jointLayoutInfo, nullptr, &m_jointSetLayout));

    // Set 3: the scene-wide maps — binding 0 = prefiltered specular cubemap, binding 1 = BRDF LUT
    // (both refilled by applyBakedEnvironment()), binding 2 = the key light's shadow map
    // (comparison sampler), binding 3 = the SAME shadow map through the non-comparison sampler
    // (raw depths for the ground shadow's PCSS blocker search). 2 and 3 are written once below —
    // the image is stable, its contents re-render per frame. All fragment-stage; one static set
    // for the whole scene.
    std::array<VkDescriptorSetLayoutBinding, 4> iblBindings{};
    for (uint32_t i = 0; i < iblBindings.size(); ++i) {
        iblBindings[i].binding = i;
        iblBindings[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        iblBindings[i].descriptorCount = 1;
        iblBindings[i].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    }
    VkDescriptorSetLayoutCreateInfo iblLayoutInfo{};
    iblLayoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    iblLayoutInfo.bindingCount = static_cast<uint32_t>(iblBindings.size());
    iblLayoutInfo.pBindings = iblBindings.data();
    VK_CHECK(vkCreateDescriptorSetLayout(device, &iblLayoutInfo, nullptr, &m_iblSetLayout));

    std::array<VkDescriptorPoolSize, 1> iblPoolSizes{};
    iblPoolSizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    iblPoolSizes[0].descriptorCount = 4;
    VkDescriptorPoolCreateInfo iblPoolInfo{};
    iblPoolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    iblPoolInfo.poolSizeCount = static_cast<uint32_t>(iblPoolSizes.size());
    iblPoolInfo.pPoolSizes = iblPoolSizes.data();
    iblPoolInfo.maxSets = 1;
    VK_CHECK(vkCreateDescriptorPool(device, &iblPoolInfo, nullptr, &m_iblPool));
    VkDescriptorSetAllocateInfo iblAlloc{};
    iblAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    iblAlloc.descriptorPool = m_iblPool;
    iblAlloc.descriptorSetCount = 1;
    iblAlloc.pSetLayouts = &m_iblSetLayout;
    VK_CHECK(vkAllocateDescriptorSets(device, &iblAlloc, &m_iblSet));

    // Bindings 2 + 3 = the shadow map (comparison + raw-depth samplers), written once
    // (applyBakedEnvironment rewrites only 0/1).
    const VkDescriptorImageInfo shadowInfo = m_shadowMap->descriptorInfo();
    const VkDescriptorImageInfo shadowRawInfo = m_shadowMap->rawDescriptorInfo();
    std::array<VkWriteDescriptorSet, 2> shadowWrites{};
    for (auto& write : shadowWrites) {
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = m_iblSet;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.descriptorCount = 1;
    }
    shadowWrites[0].dstBinding = 2;
    shadowWrites[0].pImageInfo = &shadowInfo;
    shadowWrites[1].dstBinding = 3;
    shadowWrites[1].pImageInfo = &shadowRawInfo;
    vkUpdateDescriptorSets(device, static_cast<uint32_t>(shadowWrites.size()),
                           shadowWrites.data(), 0, nullptr);

    VkDescriptorPoolSize poolSize{};
    poolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    poolSize.descriptorCount = kMaxFramesInFlight;

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = 1;
    poolInfo.pPoolSizes = &poolSize;
    poolInfo.maxSets = kMaxFramesInFlight;
    VK_CHECK(vkCreateDescriptorPool(device, &poolInfo, nullptr, &m_descriptorPool));

    const std::vector<VkDescriptorSetLayout> layouts(kMaxFramesInFlight, m_setLayout);
    VkDescriptorSetAllocateInfo setAlloc{};
    setAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    setAlloc.descriptorPool = m_descriptorPool;
    setAlloc.descriptorSetCount = kMaxFramesInFlight;
    setAlloc.pSetLayouts = layouts.data();
    m_cameraSets.resize(kMaxFramesInFlight);
    VK_CHECK(vkAllocateDescriptorSets(device, &setAlloc, m_cameraSets.data()));

    m_cameraBuffers.reserve(kMaxFramesInFlight);
    for (int i = 0; i < kMaxFramesInFlight; ++i) {
        m_cameraBuffers.emplace_back(createMappedUniformBuffer(m_context, sizeof(CameraUbo)));

        VkDescriptorBufferInfo bufferInfo{};
        bufferInfo.buffer = m_cameraBuffers[i].handle();
        bufferInfo.offset = 0;
        bufferInfo.range = sizeof(CameraUbo);

        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = m_cameraSets[i];
        write.dstBinding = 0;
        write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        write.descriptorCount = 1;
        write.pBufferInfo = &bufferInfo;
        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
    }
}

void Scene::applyBakedEnvironment(const BakedEnvironment& baked) {
    // The mesh pipeline samples the specular cubemap + LUT this is about to replace; ensure no in-flight
    // frame is still reading the old IblMaps before it's destroyed. (The expensive CPU bake — SH +
    // prefiltered specular — already happened in bakeEnvironment(), off the render thread for switches.)
    vkDeviceWaitIdle(m_context.device());
    m_environmentSH = baked.sh;
    // The BRDF LUT is environment-independent (roughness + NdotV only), so integrate it once and reuse
    // — re-running the ~8M-sample integration on every HDRI swap would be pure waste.
    if (m_brdfLut.data.empty()) {
        m_brdfLut = integrateBrdfLut();
    }
    m_iblMaps = std::make_unique<IblMaps>(m_context, baked.specular, m_brdfLut);

    const VkDescriptorImageInfo specInfo = m_iblMaps->specularInfo();
    const VkDescriptorImageInfo brdfInfo = m_iblMaps->brdfInfo();
    std::array<VkWriteDescriptorSet, 2> writes{};
    writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet = m_iblSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[0].descriptorCount = 1;
    writes[0].pImageInfo = &specInfo;
    writes[1] = writes[0];
    writes[1].dstBinding = 1;
    writes[1].pImageInfo = &brdfInfo;
    vkUpdateDescriptorSets(m_context.device(), static_cast<uint32_t>(writes.size()), writes.data(), 0,
                           nullptr);
}

void Scene::addModel(const ModelData& data) {
    m_models.push_back(std::make_unique<Model>(m_context, data, m_materialSetLayout, m_jointSetLayout,
                                               *m_fallbackTexture, *m_fallbackNormal));
    setSelectedModel(static_cast<int>(m_models.size()) - 1); // the new arrival is the selection
}

int Scene::selectedModelIndex() const {
    return (m_selectedModel >= 0 && static_cast<std::size_t>(m_selectedModel) < m_models.size())
               ? m_selectedModel
               : -1;
}

void Scene::setSelectedModel(int index) {
    if (index < 0 || static_cast<std::size_t>(index) >= m_models.size()) {
        index = -1;
    }
    const int previous = selectedModelIndex();
    if (index == previous) {
        return;
    }
    // The outgoing selection's joint selection goes away — a joint
    // selection on a model that isn't the selection would contradict the outline.
    if (previous >= 0 && m_models[static_cast<std::size_t>(previous)]->hasSkeleton()) {
        m_models[static_cast<std::size_t>(previous)]->setSelectedBone(-1);
    }
    m_selectedModel = index;
    if (index >= 0 && m_models[static_cast<std::size_t>(index)]->hasSkeleton()) {
        m_activeFigure = index; // the selected figure is the posing target
    }
}

int Scene::pickModel(const Ray& ray) const {
    int best = -1;
    float bestT = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < m_models.size(); ++i) {
        float t = 0.0f;
        if (m_models[i]->intersectRay(ray, t) && t < bestT) {
            bestT = t;
            best = static_cast<int>(i);
        }
    }
    return best;
}

bool Scene::framingBounds(glm::vec3& outMin, glm::vec3& outMax) const {
    const int selected = selectedModelIndex();
    if (selected >= 0) {
        return m_models[static_cast<std::size_t>(selected)]->worldBounds(outMin, outMax);
    }
    bool any = false;
    outMin = glm::vec3(std::numeric_limits<float>::max());
    outMax = glm::vec3(std::numeric_limits<float>::lowest());
    for (const std::unique_ptr<Model>& model : m_models) {
        glm::vec3 a;
        glm::vec3 b;
        if (model->worldBounds(a, b)) {
            outMin = glm::min(outMin, a);
            outMax = glm::max(outMax, b);
            any = true;
        }
    }
    return any;
}

void Scene::removeModel(std::size_t index) {
    if (index < m_models.size()) {
        m_models.erase(m_models.begin() + static_cast<std::ptrdiff_t>(index));
        // Keep the active figure pointing at the same model (indices above shift down); the
        // deleted figure itself falls back to the first remaining one. The selection likewise
        // follows its model, and a deleted selection leaves nothing selected.
        if (m_activeFigure == static_cast<int>(index)) {
            m_activeFigure = -1;
        } else if (m_activeFigure > static_cast<int>(index)) {
            --m_activeFigure;
        }
        if (m_selectedModel == static_cast<int>(index)) {
            m_selectedModel = -1;
        } else if (m_selectedModel > static_cast<int>(index)) {
            --m_selectedModel;
        }
    }
}

glm::vec3 Scene::keyLightDir() const {
    const float az = glm::radians(m_lighting.keyAzimuthDeg);
    const float el = glm::radians(m_lighting.keyElevationDeg);
    glm::vec3 dir(std::cos(el) * std::sin(az), std::sin(el), std::cos(el) * std::cos(az));
    // In the image-based (PBR) mode the key stands in for the environment's dominant light — the
    // HDRI bake auto-aims the dials at it — so it must follow the panel's Rotation dial the way
    // the visible environment does. The shader samples the environment at rotateY(worldDir, +rot)
    // (mesh.frag), so the world direction matching a fixed environment-space direction is
    // rotateY(dir, -rot) with the same rotation convention, replicated here. The analytic modes
    // keep the dial as a plain world-space direction (their rig ignores the environment).
    if (isPbr()) {
        const float a = -glm::radians(m_lighting.environmentRotationDeg);
        const float c = std::cos(a);
        const float s = std::sin(a);
        dir = glm::vec3(c * dir.x + s * dir.z, dir.y, -s * dir.x + c * dir.z);
    }
    return dir;
}

void Scene::recordShadowPass(VkCommandBuffer cmd, uint32_t frameIndex) {
    // Shadows off: skip the whole pass (its cost included) and park the light matrix on the same
    // degenerate projection the no-casters case uses — every receiver (figure + floor) reads
    // fully lit through it.
    if (!m_lighting.shadowsEnabled) {
        m_lightViewProj = glm::mat4(0.0f);
        m_lightViewProj[3][3] = 1.0f;
        return;
    }

    // Fit the light's ortho frustum around every caster AND its shadow's landing spot on the floor
    // (casting each AABB corner along the light onto y=0) — the ground shadow is only correct where
    // the RECEIVER is inside the frustum, so the floor patch must be covered too.
    glm::vec3 mn(std::numeric_limits<float>::max());
    glm::vec3 mx(std::numeric_limits<float>::lowest());
    bool any = false;
    const glm::vec3 dir = keyLightDir(); // TO the light; a shadow ray travels along -dir
    for (const std::unique_ptr<Model>& model : m_models) {
        glm::vec3 a;
        glm::vec3 b;
        if (!model->worldBounds(a, b)) {
            continue;
        }
        any = true;
        mn = glm::min(mn, a);
        mx = glm::max(mx, b);
        if (dir.y > 0.05f) {
            for (int i = 0; i < 8; ++i) {
                const glm::vec3 c((i & 1) ? b.x : a.x, (i & 2) ? b.y : a.y, (i & 4) ? b.z : a.z);
                const glm::vec3 onFloor = c - dir * (c.y / dir.y); // where c's shadow lands on y=0
                mn = glm::min(mn, onFloor);
                mx = glm::max(mx, onFloor);
            }
        }
    }
    if (!any) {
        // No casters: skip the pass (the map was cleared fully-lit at creation) and park the light
        // matrix on a projection that maps every receiver to depth 0 — the LESS_OR_EQUAL compare
        // then always passes, i.e. everything reads unshadowed.
        m_lightViewProj = glm::mat4(0.0f);
        m_lightViewProj[3][3] = 1.0f;
        return;
    }

    // Ortho frustum around the fitted bounding sphere (padded ~10%: bind-pose bounds understate a
    // posed figure's reach a little).
    const glm::vec3 center = 0.5f * (mn + mx);
    const float radius = 0.55f * glm::length(mx - mn) + 0.05f;
    const glm::vec3 eye = center + dir * (radius + 0.25f);
    const glm::vec3 up =
        (std::abs(dir.y) > 0.95f) ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::vec3(0.0f, 1.0f, 0.0f);
    const glm::mat4 view = glm::lookAt(eye, center, up);
    // GLM_FORCE_DEPTH_ZERO_TO_ONE puts z in [0,1]. No Vulkan Y-negate here: the map is rendered
    // and sampled through this same matrix, so the flip would cancel anyway.
    const glm::mat4 proj = glm::ortho(-radius, radius, -radius, radius, 0.01f, 2.0f * radius + 0.5f);
    m_lightViewProj = proj * view;

    VkClearValue clear{};
    clear.depthStencil = {1.0f, 0};
    VkRenderPassBeginInfo rp{};
    rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rp.renderPass = m_shadowMap->renderPass();
    rp.framebuffer = m_shadowMap->framebuffer();
    rp.renderArea.extent = {m_shadowMap->size(), m_shadowMap->size()};
    rp.clearValueCount = 1;
    rp.pClearValues = &clear;
    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport viewport{};
    viewport.width = static_cast<float>(m_shadowMap->size());
    viewport.height = static_cast<float>(m_shadowMap->size());
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    VkRect2D scissor{};
    scissor.extent = {m_shadowMap->size(), m_shadowMap->size()};
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_shadowPipeline->handle());
    for (const std::unique_ptr<Model>& model : m_models) {
        model->recordShadow(cmd, m_shadowPipeline->layout(), m_lightViewProj, frameIndex);
    }
    vkCmdEndRenderPass(cmd);
}

bool Scene::recordOutlinePass(VkCommandBuffer cmd, const Camera& camera, uint32_t frameIndex,
                              const OutlineMask& mask) {
    const int selected = selectedModelIndex();
    if (selected < 0) {
        return false; // nothing selected: skip the pass; the composite draws no outline
    }
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

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_outlinePipeline->handle());
    m_models[static_cast<std::size_t>(selected)]->recordSilhouette(
        cmd, m_outlinePipeline->layout(), camera.viewProjection(), frameIndex);
    vkCmdEndRenderPass(cmd);
    return true;
}

void Scene::record(VkCommandBuffer cmd, const Camera& camera, uint32_t frameIndex) {
    // Update this frame's camera + lighting UBO. The default (PBR) mode is image-based-lit — its diffuse
    // comes from the environment's SH irradiance (filled in below) and its specular from the prefiltered-
    // environment cubemap (set 3). The analytic lights below feed only the *non-IBL* shade modes: only
    // Rendered (mode 0) uses the full key/fill/rim rig; the other analytic modes use just the key +
    // ambient, and PBR uses only the key (scaled by the panel's keyIntensity). The rig is world-space
    // (fixed relative to the subject, so lighting stays consistent as the camera orbits). Dirs point TO
    // each light: Key = front-right & above (form + shadows); Fill = opposite side, lower + dimmer (opens
    // the shadow side); Back/rim = behind + above (rims the shoulders so the figure pops off the grid).
    CameraUbo ubo{};
    ubo.viewProj = camera.viewProjection();
    ubo.view = camera.view();
    ubo.lightViewProj = m_lightViewProj; // fitted by recordShadowPass earlier this frame
    ubo.cameraPos = glm::vec4(camera.position(), 1.0f);
    // Key direction from the Environment panel's azimuth/elevation dials (world-fixed relative to
    // the subject; defaults reproduce the old hardcoded front-right ~30° direction).
    ubo.lightDir = glm::vec4(keyLightDir(), 0.0f);
    ubo.lightColor = glm::vec4(1.0f, 0.96f, 0.9f, 0.0f);
    ubo.fillDir = glm::vec4(glm::normalize(glm::vec3(-0.6f, 0.28f, 0.5f)), 0.0f);    // fill: opposite, lower
    ubo.fillColor = glm::vec4(0.26f, 0.31f, 0.4f, 0.0f);                              // dim + cool (~1/4 key)
    ubo.rimDir = glm::vec4(glm::normalize(glm::vec3(-0.15f, 0.55f, -0.9f)), 0.0f);   // back: behind + above
    ubo.rimColor = glm::vec4(1.2f, 1.28f, 1.45f, 0.0f);                               // cool, localised to edges
    ubo.ambient = glm::vec4(0.10f, 0.11f, 0.13f, 0.0f);
    const ShadeMode& spec = shadeModeSpec();
    // params.x is mesh.frag's mode for the SURFACE (the picker row's fragMode — the picker index
    // itself never reaches the shader); a row without a shaded surface pushes 0, unread.
    ubo.params = glm::vec4(static_cast<float>(std::max(spec.fragMode, 0)), m_lighting.exposure,
                           m_lighting.specularIntensity, m_lighting.ambientFill);
    // params2.w is free: the tonemap flag rides composite.frag's push constant now (tonemapping
    // moved to the composite pass so bloom sees real radiance) — no scene shader reads it.
    ubo.params2 = glm::vec4(m_lighting.diffuseIntensity, m_lighting.keyIntensity,
                            glm::radians(m_lighting.environmentRotationDeg), 0.0f);
    ubo.params3 = glm::vec4(m_lighting.subsurface, m_lighting.rimIntensity,
                            static_cast<float>(m_lighting.backdropMode), m_lighting.backdropBlur);
    ubo.params4 = glm::vec4(m_lighting.backdropBrightness, m_lighting.domeRadius,
                            m_lighting.shadowIntensity, spec.wireLevel);
    // Environment diffuse irradiance (SH). The PBR mode reconstructs per-normal ambient from these
    // instead of the flat `ambient` constant, so shadow sides pick up the environment's colour.
    for (int i = 0; i < 9; ++i) {
        ubo.sh[i] = glm::vec4(m_environmentSH.c[i], 0.0f);
    }
    std::memcpy(m_cameraBuffers[frameIndex].mappedData(), &ubo, sizeof(ubo));

    // HDRI backdrop first (PBR mode only — the stylized modes keep the flat viewport clear): the
    // environment that lights the figure, visible behind it. Depth test/write are off in its
    // pipeline, so everything after simply draws over it. Runs even with no models — an empty
    // PBR viewport still shows the environment. Backdrop mode 0 ("Off", an Environment-panel
    // dial) skips it, leaving the flat viewport grey.
    if (isPbr() && m_lighting.backdropMode != 0) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_backgroundPipeline->handle());
        const VkDescriptorSet bgSets[2] = {m_cameraSets[frameIndex], m_iblSet};
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                m_backgroundPipeline->layout(), 0, 2, bgSets, 0, nullptr);
        vkCmdDraw(cmd, 3, 1, 0, 0);
    }

    if (m_models.empty()) {
        return; // nothing else to draw; the grid still renders on its own
    }

    // Every mesh pipeline shares one layout: the camera set (0) and the scene-global IBL/shadow set
    // (3) are bound once here and stay bound through the surface and wire passes below; set 1
    // (material) is bound per mesh and set 2 (joints) per model.
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipeline->layout(), 0, 1,
                            &m_cameraSets[frameIndex], 0, nullptr);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipeline->layout(), 3, 1, &m_iblSet,
                            0, nullptr);

    // --- The surface: the lit passes, a hidden-line depth fill, or nothing (see ShadeMode). ---
    if (spec.fill == FillKind::Shaded) {
        // Opaque pass first.
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipeline->handle());
        for (const std::unique_ptr<Model>& model : m_models) {
            model->record(cmd, m_pipeline->layout(), /*transparentPass=*/false, camera.position(),
                          frameIndex);
        }
        // Transparent pass: alpha-blended, depth-write off, drawn after all opaque geometry so it
        // blends over what's behind it (e.g. the eye's moisture/cornea over the iris); each model
        // sorts its transparent meshes back-to-front from the camera.
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_transparentPipeline->handle());
        for (const std::unique_ptr<Model>& model : m_models) {
            model->record(cmd, m_transparentPipeline->layout(), /*transparentPass=*/true,
                          camera.position(), frameIndex);
        }
    } else if (spec.fill == FillKind::HiddenLine) {
        // Depth only (colour writes masked): the surface hides what's behind it and shows the
        // viewport's clear colour; the wire pass then draws only the edges facing the camera.
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_hiddenLinePipeline->handle());
        for (const std::unique_ptr<Model>& model : m_models) {
            model->recordDepthFill(cmd, m_hiddenLinePipeline->layout(), frameIndex);
        }
    }

    // --- Wireframe: every mesh's triangle edges, over the surface or on their own. ---
    if (spec.wireframe && m_wirePipeline) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_wirePipeline->handle());
        for (const std::unique_ptr<Model>& model : m_models) {
            model->recordWire(cmd, m_wirePipeline->layout(), frameIndex);
        }
    }

    // --- Line overlays, all through one host-mapped line buffer + one draw at the end. ---
    VulkanBuffer& lineBuffer = m_skeletonVertexBuffers[frameIndex];
    auto* verts = static_cast<LineVertex*>(lineBuffer.mappedData());
    uint32_t count = 0;

    // Orthographic side views (the Front/Back/Left/Right hotkeys) see the floor plane exactly
    // edge-on, where the grid shader's ray/plane intersection has nothing to hit — so draw the
    // floor as the one line it is from there: the y = 0 trace of the view plane through the
    // target, along the camera's right vector, in the grid's own line grey. Pure elevation
    // drawings get their ground line, as in any DCC's fixed side camera.
    if (camera.orthographic() && std::abs(camera.pitch()) < 1e-3f && count + 2 <= kMaxSkeletonVerts) {
        const glm::mat4 view = camera.view();
        const glm::vec3 right(view[0][0], view[1][0], view[2][0]); // world-space screen right
        const glm::vec3 base(camera.target().x, 0.0f, camera.target().z);
        constexpr float kFloorLineHalfLength = 200.0f;
        const glm::vec3 floorLineGrey(0.30f); // the grid's major-line brightness (grid.frag)
        verts[count++] = {base - right * kFloorLineHalfLength, floorLineGrey};
        verts[count++] = {base + right * kFloorLineHalfLength, floorLineGrey};
    }

    // Posing overlay: (only when enabled) the skeleton line list, plus the pin markers. The
    // skeleton is hidden by default — joints are grabbed directly on the figure — but they stay
    // pickable regardless, because picking (selectBoneAt) is independent of what's drawn here.
    if (Model* fig = figureModel(); fig && fig->boneCount() > 0) {
        const int selected = fig->selectedBone();

        // Skeleton as line segments (joint -> parent); the selected joint's segments are highlighted.
        if (m_showSkeleton) {
            const glm::vec3 boneColor(0.25f, 0.85f, 1.0f); // cyan
            const glm::vec3 selColor(1.0f, 0.8f, 0.1f);    // yellow highlight
            for (std::size_t i = 0; i < fig->boneCount() && count + 2 <= kMaxSkeletonVerts; ++i) {
                const int parent = fig->boneParent(i);
                if (parent < 0) {
                    continue;
                }
                const bool hot = (static_cast<int>(i) == selected || parent == selected);
                const glm::vec3 c = hot ? selColor : boneColor;
                verts[count++] = {fig->boneWorldPosition(static_cast<std::size_t>(parent)), c};
                verts[count++] = {fig->boneWorldPosition(i), c};
            }
        }

        // Pin markers: a small wireframe octahedron on every USER-pinned joint (orange, always
        // shown — the pin persists across drags), and a smaller cyan one on each ground-contact
        // pin while an IK drag is live, so it's visible which feet the solve is holding planted.
        // Screen-constant sizing (markerRadius). 12 edges = 24 line vertices each.
        const auto emitPinMarker = [&](const glm::vec3& c, float r, const glm::vec3& col) {
            if (count + 24 > kMaxSkeletonVerts) {
                return;
            }
            const glm::vec3 ax[3] = {glm::vec3(r, 0.0f, 0.0f), glm::vec3(0.0f, r, 0.0f),
                                     glm::vec3(0.0f, 0.0f, r)};
            for (int i = 0; i < 3; ++i) {
                const int j = (i + 1) % 3;
                for (int si = -1; si <= 1; si += 2) {
                    const glm::vec3 a = c + ax[i] * static_cast<float>(si);
                    for (int sj = -1; sj <= 1; sj += 2) {
                        verts[count++] = {a, col};
                        verts[count++] = {c + ax[j] * static_cast<float>(sj), col};
                    }
                }
            }
        };
        const glm::vec3 userPinColor(1.0f, 0.55f, 0.12f);
        const glm::vec3 contactPinColor(0.25f, 0.9f, 0.95f);
        for (const int node : fig->activeContactPins()) {
            if (node >= 0 && static_cast<std::size_t>(node) < fig->boneCount()) {
                const glm::vec3 c = fig->boneWorldPosition(static_cast<std::size_t>(node));
                emitPinMarker(c, 0.3f * markerRadius(c, camera), contactPinColor);
            }
        }
        for (const std::unique_ptr<Model>& anyFig : m_models) {
            if (!anyFig->hasSkeleton()) {
                continue;
            }
            for (std::size_t i = 0; i < anyFig->boneCount(); ++i) {
                if (anyFig->isBonePinned(i)) {
                    const glm::vec3 c = anyFig->boneWorldPosition(i);
                    emitPinMarker(c, 0.45f * markerRadius(c, camera), userPinColor);
                }
            }
        }

    }

    if (count > 0) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_skeletonPipeline->handle());
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_skeletonPipeline->layout(),
                                0, 1, &m_cameraSets[frameIndex], 0, nullptr);
        const VkBuffer vb = lineBuffer.handle();
        const VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &vb, &offset);
        vkCmdDraw(cmd, count, 1, 0, 0);
    }
}

Model* Scene::figureModel() const {
    const int idx = activeFigureIndex();
    return idx >= 0 ? m_models[static_cast<std::size_t>(idx)].get() : nullptr;
}

int Scene::activeFigureIndex() const {
    if (m_activeFigure >= 0 && static_cast<std::size_t>(m_activeFigure) < m_models.size() &&
        m_models[static_cast<std::size_t>(m_activeFigure)]->hasSkeleton()) {
        return m_activeFigure;
    }
    for (std::size_t i = 0; i < m_models.size(); ++i) {
        if (m_models[i]->hasSkeleton()) {
            return static_cast<int>(i); // the first figure until one is clicked
        }
    }
    return -1;
}

void Scene::setActiveFigure(int index) {
    if (index >= 0 && static_cast<std::size_t>(index) < m_models.size() &&
        m_models[static_cast<std::size_t>(index)]->hasSkeleton()) {
        m_activeFigure = index;
        setSelectedModel(index); // the posing target is the selection (and the outline)
    }
}

bool Scene::hasPosableFigure() const { return figureModel() != nullptr; }

bool Scene::hasSelectedBone() const {
    const Model* fig = figureModel();
    return fig != nullptr && fig->selectedBone() >= 0;
}

int Scene::selectBoneByName(const std::string& name) {
    Model* fig = figureModel();
    return fig ? fig->selectBoneByName(name) : -1;
}

int Scene::selectBoneAt(float px, float py, float vpW, float vpH, const Camera& camera) {
    // Every figure's joints compete: the nearest of ANY figure wins, and its figure becomes the
    // active one (the posing target). Without this a second figure in the scene could never be
    // posed — every call went to the first skeleton.
    //
    // Picking is by BONE, not just by joint origin: a bone's body is the segment from its joint
    // to each child joint, and a click anywhere along it selects that bone — so clicking the
    // middle of a thigh grabs the thigh, where joint-origin picking alone (the thigh's joints
    // sit at the hip and the knee, farther than any sane radius from a mid-thigh click) fell
    // through to a plain model click. A click right ON a joint (kJointSnapPx) still snaps to
    // that joint, so the knee picks the shin bone whose origin it is rather than the thigh
    // segment ending there. A root's own segment is skipped: a figure node sits at the origin
    // with the hip a metre above, and that virtual bone runs straight between the legs.
    const glm::mat4 viewProj = camera.viewProjection();
    const glm::vec2 click(px, py);
    const auto distToSegment = [](const glm::vec2& p, const glm::vec2& a, const glm::vec2& b) {
        const glm::vec2 ab = b - a;
        const float len2 = glm::dot(ab, ab);
        const float t = (len2 > 1e-6f) ? glm::clamp(glm::dot(p - a, ab) / len2, 0.0f, 1.0f) : 0.0f;
        return glm::length(p - (a + ab * t));
    };
    int   jointModel = -1, joint = -1; float jointDist = 1e9f; // nearest joint ORIGIN
    int   segModel = -1,   seg = -1;   float segDist = 1e9f;   // nearest bone SEGMENT (its owner)
    std::vector<glm::vec2> screen;
    std::vector<char>      visible;
    for (std::size_t m = 0; m < m_models.size(); ++m) {
        const Model* fig = m_models[m].get();
        if (!fig->hasSkeleton()) {
            continue;
        }
        const std::size_t n = fig->boneCount();
        screen.assign(n, glm::vec2(0.0f));
        visible.assign(n, 0);
        for (std::size_t i = 0; i < n; ++i) {
            const glm::vec4 clip = viewProj * glm::vec4(fig->boneWorldPosition(i), 1.0f);
            if (clip.w <= 1e-4f) {
                continue; // behind the camera
            }
            const glm::vec3 ndc = glm::vec3(clip) / clip.w;
            // viewProj already carries Vulkan's Y flip, so this matches the mouse convention.
            screen[i] = glm::vec2((ndc.x * 0.5f + 0.5f) * vpW, (ndc.y * 0.5f + 0.5f) * vpH);
            visible[i] = 1;
            const float dist = glm::length(screen[i] - click);
            if (dist < jointDist) {
                jointDist = dist;
                joint = static_cast<int>(i);
                jointModel = static_cast<int>(m);
            }
        }
        for (std::size_t i = 0; i < n; ++i) {
            const int parent = fig->boneParent(i);
            if (parent < 0 || fig->boneParent(static_cast<std::size_t>(parent)) < 0 || !visible[i] ||
                !visible[static_cast<std::size_t>(parent)]) {
                continue; // no segment, a root's virtual segment, or off-camera
            }
            const float dist = distToSegment(click, screen[static_cast<std::size_t>(parent)], screen[i]);
            if (dist < segDist) {
                segDist = dist;
                seg = parent; // the segment is the PARENT bone's body
                segModel = static_cast<int>(m);
            }
        }
    }
    // Priority: a joint under the cursor, then the nearest bone body within the click
    // tolerance, then a joint within it (leaf bones — toes, fingertips — have no segment).
    // On a miss return -1 (so the caller orbits / click-selects) but keep the current selection
    // — the figure can be orbited while a joint is selected.
    constexpr float kJointSnapPx = 14.0f;  // right on a joint origin
    constexpr float kPickRadiusPx = 32.0f; // click tolerance around a bone body or joint
    int chosenModel = -1;
    int chosen = -1;
    if (joint >= 0 && jointDist <= kJointSnapPx) {
        chosenModel = jointModel;
        chosen = joint;
    } else if (seg >= 0 && segDist <= kPickRadiusPx) {
        chosenModel = segModel;
        chosen = seg;
    } else if (joint >= 0 && jointDist <= kPickRadiusPx) {
        chosenModel = jointModel;
        chosen = joint;
    }
    if (chosen >= 0) {
        if (chosenModel != activeFigureIndex()) {
            // Switching figures: the previous one's selection goes away.
            if (Model* previous = figureModel()) {
                previous->setSelectedBone(-1);
            }
        }
        setActiveFigure(chosenModel);
        m_models[static_cast<std::size_t>(chosenModel)]->setSelectedBone(chosen);
        return chosen;
    }
    return -1;
}

void Scene::nudgeSelectedBone(const glm::vec3& deltaEulerDegrees) {
    if (Model* fig = figureModel()) {
        fig->nudgeSelectedBone(deltaEulerDegrees);
    }
}

void Scene::finalizePose() {
    if (Model* fig = figureModel()) {
        fig->refreshCorrectives();
    }
}

bool Scene::beginBoneIkDrag() {
    Model* fig = figureModel();
    return fig != nullptr && fig->beginIkDrag();
}

bool Scene::dragBoneIkTo(const glm::vec3& targetWorld) {
    Model* fig = figureModel();
    return fig && fig->dragIkTo(targetWorld);
}

bool Scene::settleBoneIkTick() {
    Model* fig = figureModel();
    return fig && fig->settleIkTick();
}

void Scene::endBoneIkDrag() {
    if (Model* fig = figureModel()) {
        fig->endIkDrag();
    }
}

bool Scene::selectedBoneWorldPosition(glm::vec3& out) const {
    const Model* fig = figureModel();
    if (!fig || fig->selectedBone() < 0 ||
        fig->selectedBone() >= static_cast<int>(fig->boneCount())) {
        return false;
    }
    out = fig->boneWorldPosition(static_cast<std::size_t>(fig->selectedBone()));
    return true;
}

bool Scene::groundFigure() {
    if (Model* fig = figureModel()) {
        return fig->dropToGround();
    }
    return false;
}

bool Scene::figureGroundGap(float& lowestY) const {
    const Model* fig = figureModel();
    return fig != nullptr && fig->groundGap(lowestY);
}

void Scene::translateFigureY(float dy) {
    if (Model* fig = figureModel()) {
        fig->translateY(dy);
    }
}

bool Scene::togglePinSelectedBone() {
    Model* fig = figureModel();
    return fig && fig->togglePinSelectedBone();
}

bool Scene::selectedBonePinned() const {
    const Model* fig = figureModel();
    return fig && fig->selectedBonePinned();
}

bool Scene::hasPinnedBones() const {
    const Model* fig = figureModel();
    return fig && fig->hasPinnedBones();
}

void Scene::unpinAllBones() {
    if (Model* fig = figureModel()) {
        fig->unpinAllBones();
    }
}

std::vector<std::pair<std::string, glm::vec3>> Scene::capturePose() const {
    const Model* fig = figureModel();
    return fig ? fig->capturePose() : std::vector<std::pair<std::string, glm::vec3>>{};
}

void Scene::applyPose(const std::vector<std::pair<std::string, glm::vec3>>& pose) {
    if (Model* fig = figureModel()) {
        fig->applyPose(pose);
    }
}

bool Scene::savePose(const std::string& path) const {
    const Model* fig = figureModel();
    if (!fig) {
        return false;
    }
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    // One line per posed joint: "boneName rx ry rz" (Euler degrees).
    for (const auto& [name, euler] : fig->capturePose()) {
        out << name << ' ' << euler.x << ' ' << euler.y << ' ' << euler.z << '\n';
    }
    return true;
}

bool Scene::loadPose(const std::string& path) {
    Model* fig = figureModel();
    if (!fig) {
        return false;
    }
    std::ifstream in(path);
    if (!in) {
        return false;
    }
    std::vector<std::pair<std::string, glm::vec3>> pose;
    std::string name;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    while (in >> name >> x >> y >> z) {
        pose.emplace_back(name, glm::vec3(x, y, z));
    }
    fig->applyPose(pose);
    return true;
}

} // namespace pose
