/**
 * @file mesh.cpp
 * @brief Mesh upload, material set, and draw. See mesh.h.
 */

#include "mesh.h"

#include "descriptorutil.h"
#include "modeldata.h"
#include "textureuploadcache.h"
#include "vertex.h"
#include "vulkancommands.h"
#include "vulkancontext.h"
#include "vulkanimage.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace pose {

Mesh::Mesh(VulkanContext& context, const MeshData& data, VkDescriptorSetLayout materialSetLayout,
           VkDescriptorPool materialPool, const VulkanTexture& fallbackDiffuse,
           const VulkanTexture& fallbackNormal, TextureUploadCache& uploads, ImmediateBatch& batch,
           const std::vector<uint32_t>* correctiveRanges)
    : m_indexCount(static_cast<uint32_t>(data.indices.size())), m_opacity(data.opacity),
      m_hasOpacityMask(data.hasOpacityMask) {
    // Bind-pose centroid (the transparent pass's back-to-front sort key) and the joint carrying
    // most of the mesh's skin weight (which carries that centroid through the pose). An unskinned
    // mesh's vertices all weight joint 0 fully, so it lands on the identity joint.
    if (!data.vertices.empty()) {
        constexpr uint32_t kMaxJointIndex = 4096; // sanity bound on a corrupt index
        glm::vec3 sum(0.0f);
        std::vector<float> jointWeight; // summed skin weight per joint index
        for (const Vertex& v : data.vertices) {
            sum += v.pos;
            for (int k = 0; k < 4; ++k) {
                const uint32_t joint = v.joints[k];
                const float weight = v.weights[k];
                if (weight <= 0.0f || joint >= kMaxJointIndex) {
                    continue;
                }
                if (joint >= jointWeight.size()) {
                    jointWeight.resize(static_cast<std::size_t>(joint) + 1, 0.0f);
                }
                jointWeight[joint] += weight;
            }
        }
        m_centroid = sum / static_cast<float>(data.vertices.size());
        if (!jointWeight.empty()) {
            m_dominantJoint = static_cast<uint32_t>(
                std::max_element(jointWeight.begin(), jointWeight.end()) - jointWeight.begin());
        }
    }

    const VkDeviceSize vertexBytes = sizeof(Vertex) * data.vertices.size();
    const VkDeviceSize indexBytes = sizeof(uint32_t) * data.indices.size();
    // Record all uploads into the shared batch instead of submitting per buffer/texture. A mesh
    // some pose corrective touches uploads a COPY of its vertices with the Model's per-vertex
    // corrective ranges stamped in (Vertex::correctiveRange) — the importer's data stays
    // pristine (the ground samples and bounds read it after this), and the batch copies the
    // temporary into its staging buffer before returning. (A one-time whole-array copy per
    // touched mesh at import; not worth avoiding.)
    if (correctiveRanges != nullptr && correctiveRanges->size() == data.vertices.size()) {
        std::vector<Vertex> stamped = data.vertices;
        for (std::size_t i = 0; i < stamped.size(); ++i) {
            stamped[i].correctiveRange = (*correctiveRanges)[i];
        }
        m_vertexBuffer = createDeviceLocalBuffer(context, stamped.data(), vertexBytes,
                                                 VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, batch);
    } else {
        m_vertexBuffer = createDeviceLocalBuffer(context, data.vertices.data(), vertexBytes,
                                                 VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, batch);
    }
    m_indexBuffer = createDeviceLocalBuffer(context, data.indices.data(), indexBytes,
                                            VK_BUFFER_USAGE_INDEX_BUFFER_BIT, batch);

    // Upload (or reuse — see TextureUploadCache) the diffuse map if the Qt layer decoded one for
    // this mesh; else use the white fallback.
    m_texture = uploadShared(context, uploads, data.diffuseImage, /*srgb=*/true, batch);
    // The detail (normal/bump) map — a LINEAR texture — and its mode. With no map we bind the
    // flat-normal fallback and force mode 0 (no perturbation).
    m_normalTexture = uploadShared(context, uploads, data.normalImage, /*srgb=*/false, batch);
    const int   normalMode = m_normalTexture ? data.normalMode : 0;       // 0 none / 1 normal / 2 bump
    const float normalStrength = m_normalTexture ? data.normalStrength : 1.0f; // authored strength
    // Specular parameter maps (linear ×-multiplier data; see modeldata.h). Absent => the shared
    // white fallback, so the shader's per-texel multiplies are no-ops for scalar-only materials.
    m_roughnessTexture = uploadShared(context, uploads, data.roughnessImage, /*srgb=*/false, batch);
    m_specMaskTexture = uploadShared(context, uploads, data.specMaskImage, /*srgb=*/false, batch);
    // Translucency map (sRGB — it's the transmitted-tint colour; grayscale weight maps read the
    // same either way through their luminance).
    m_translucencyTexture =
        uploadShared(context, uploads, data.translucencyImage, /*srgb=*/true, batch);
    // Micro-detail (pore) normal map — linear, tiled (the sampler wraps).
    m_microNormalTexture =
        uploadShared(context, uploads, data.detailNormalImage, /*srgb=*/false, batch);

    // The push block's per-mesh constants, baked once (record() patches the model matrix and the
    // draw-kind/highlight field per draw).
    m_pushTemplate.baseColor = glm::vec4(data.baseColor, data.opacity);
    // z = metalness ("Metallic Weight" — scalar only, no map yet); w = the per-material specular
    // (F0) weight the material parser derived from the active specular lobe's reflectivity
    // (1 = standard 4% dielectric; skin ~0.5 → ~2%; OBJ meshes default to 1).
    // material.y packs the detail mode + its authored strength (floor = mode, fract×8 = strength;
    // the push block is at the 128-byte limit, so the pair shares one float).
    const float modePacked =
        static_cast<float>(normalMode) +
        std::clamp(normalStrength, 0.0f, 7.9f) * 0.125f * (normalMode > 0 ? 1.0f : 0.0f);
    m_pushTemplate.material = glm::vec4(data.roughness, modePacked, data.metalness,
                                        data.specularWeight);
    m_pushTemplate.material2 = glm::vec4(data.lobe1Roughness, data.lobe2Roughness, data.lobeRatio,
                                         data.translucencyWeight);
    // material3.w packs the micro-detail layer (the block sits at the 128-byte push limit):
    // integer part = UV tiling, fraction = weight; 0 = no detail layer. material3.z is the
    // per-draw field.
    const float detailPacked =
        (m_microNormalTexture && data.detailTiles >= 1.0f && data.detailWeight > 0.0f)
            ? std::floor(data.detailTiles) + std::min(data.detailWeight, 0.99f)
            : 0.0f;
    m_pushTemplate.material3 = glm::vec4(data.topCoatWeight, data.topCoatRoughness, 0.0f,
                                         detailPacked);

    const VulkanTexture& diffuse = m_texture ? *m_texture : fallbackDiffuse;
    const VulkanTexture& detail = m_normalTexture ? *m_normalTexture : fallbackNormal;
    // The white fallback is numerically identical sampled as sRGB or linear (1.0 either way), so
    // the diffuse fallback doubles for the parameter maps.
    const VulkanTexture& roughMap = m_roughnessTexture ? *m_roughnessTexture : fallbackDiffuse;
    const VulkanTexture& specMask = m_specMaskTexture ? *m_specMaskTexture : fallbackDiffuse;
    const VulkanTexture& transMap = m_translucencyTexture ? *m_translucencyTexture : fallbackDiffuse;
    const VulkanTexture& microNormal = m_microNormalTexture ? *m_microNormalTexture : fallbackNormal;

    // Allocate this mesh's set-1 descriptor from the owning Model's pool: binding 0 = diffuse,
    // binding 1 = detail map, binding 2 = roughness map, binding 3 = spec-mask map,
    // binding 4 = translucency map, binding 5 = micro-detail (pore) normal map.
    m_materialSet = allocateDescriptorSet(context.device(), materialPool, materialSetLayout);
    const std::array<VkDescriptorImageInfo, 6> imageInfos = {
        sampledImageInfo(diffuse.imageView(), diffuse.sampler()),
        sampledImageInfo(detail.imageView(), detail.sampler()),
        sampledImageInfo(roughMap.imageView(), roughMap.sampler()),
        sampledImageInfo(specMask.imageView(), specMask.sampler()),
        sampledImageInfo(transMap.imageView(), transMap.sampler()),
        sampledImageInfo(microNormal.imageView(), microNormal.sampler()),
    };
    writeCombinedImageSamplers(context.device(), m_materialSet, 0, imageInfos.data(),
                               static_cast<uint32_t>(imageInfos.size()));
}

void Mesh::record(VkCommandBuffer cmd, VkPipelineLayout layout, const glm::mat4& model,
                  MeshDrawKind kind, int highlightJoint, int highlightTwin) const {
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 1, 1, &m_materialSet, 0,
                            nullptr); // set 1 = this mesh's textures

    MeshPushConstants push = m_pushTemplate;
    push.model = model;
    // material3.z packs the draw kind with the selected joint + its highlight twin (see the
    // declaration): kind + 8·(joint+1) + 8192·(twin+1), every field a small exact integer.
    const int packedKind = static_cast<int>(kind) +
                           8 * (std::clamp(highlightJoint, -1, 1022) + 1) +
                           8192 * (std::clamp(highlightTwin, -1, 1022) + 1);
    push.material3.z = static_cast<float>(packedKind);
    vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                       sizeof(push), &push);

    const VkBuffer vertexBuffer = m_vertexBuffer.handle();
    const VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuffer, &offset);
    vkCmdBindIndexBuffer(cmd, m_indexBuffer.handle(), 0, VK_INDEX_TYPE_UINT32);
    vkCmdDrawIndexed(cmd, m_indexCount, 1, 0, 0, 0);
}

void Mesh::recordDepth(VkCommandBuffer cmd) const {
    const VkBuffer vertexBuffer = m_vertexBuffer.handle();
    const VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vertexBuffer, &offset);
    vkCmdBindIndexBuffer(cmd, m_indexBuffer.handle(), 0, VK_INDEX_TYPE_UINT32);
    vkCmdDrawIndexed(cmd, m_indexCount, 1, 0, 0, 0);
}

} // namespace pose
