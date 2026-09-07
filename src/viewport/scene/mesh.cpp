/**
 * @file mesh.cpp
 * @brief Implementation of Mesh and Model. See mesh.h.
 */

#include "mesh.h"

#include "camera.h" // Ray
#include "modeldata.h"
#include "vertex.h"
#include "vulkancommands.h"
#include "vulkancommon.h"
#include "vulkancontext.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <string>

#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pose {

namespace {

// Evaluates a corrective driver spline at @p x: a Catmull-Rom Hermite through the (ascending-in-x)
// knots, clamped flat outside the knot range and — within each segment — clamped to that segment's
// endpoint values so the smoothing can never overshoot into a wrong-signed correction.
float evalSpline(const std::vector<CorrectiveKnot>& knots, float x) {
    if (knots.empty()) {
        return 0.0f;
    }
    if (knots.size() == 1 || x <= knots.front().x) {
        return knots.front().y;
    }
    if (x >= knots.back().x) {
        return knots.back().y;
    }
    std::size_t i = 0;
    while (i + 1 < knots.size() && x > knots[i + 1].x) {
        ++i;
    }
    const CorrectiveKnot& p1 = knots[i];
    const CorrectiveKnot& p2 = knots[i + 1];
    const float h = p2.x - p1.x;
    if (h <= 1e-6f) {
        return p1.y;
    }
    const float u = (x - p1.x) / h;
    const float y0 = (i > 0) ? knots[i - 1].y : p1.y;                    // one-sided at the ends
    const float y3 = (i + 2 < knots.size()) ? knots[i + 2].y : p2.y;
    const float m1 = 0.5f * (p2.y - y0);
    const float m2 = 0.5f * (y3 - p1.y);
    const float u2 = u * u;
    const float u3 = u2 * u;
    const float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
    const float h10 = u3 - 2.0f * u2 + u;
    const float h01 = -2.0f * u3 + 3.0f * u2;
    const float h11 = u3 - u2;
    const float y = h00 * p1.y + h10 * m1 + h01 * p2.y + h11 * m2;
    return std::clamp(y, std::min(p1.y, p2.y), std::max(p1.y, p2.y));
}

} // namespace

/// Per-model dedup of texture uploads: one VulkanTexture per unique DecodedImage (+ colour space —
/// the same image could in principle feed both an sRGB and a LINEAR slot, which need distinct
/// VkImages). Lives only for the duration of the Model constructor; the meshes keep the textures
/// alive through their shared_ptrs afterwards.
struct TextureUploadCache {
    std::unordered_map<const DecodedImage*, std::shared_ptr<VulkanTexture>> srgb;
    std::unordered_map<const DecodedImage*, std::shared_ptr<VulkanTexture>> linear;
};

namespace {

// Returns the (shared) texture for @p image, uploading it into @p batch only on first use. Null
// image => null (the caller binds the appropriate 1x1 fallback).
std::shared_ptr<VulkanTexture> uploadShared(VulkanContext& context, TextureUploadCache& cache,
                                            const DecodedImagePtr& image, bool srgbFormat,
                                            ImmediateBatch& batch) {
    if (!image || image->width == 0 || image->height == 0 || image->pixels.empty()) {
        return nullptr;
    }
    auto& map = srgbFormat ? cache.srgb : cache.linear;
    const auto it = map.find(image.get());
    if (it != map.end()) {
        return it->second;
    }
    auto texture = std::make_shared<VulkanTexture>(context, image->pixels.data(), image->width,
                                                   image->height, batch, srgbFormat);
    map.emplace(image.get(), texture);
    return texture;
}

} // namespace

Mesh::Mesh(VulkanContext& context, const MeshData& data, VkDescriptorSetLayout materialSetLayout,
           VkDescriptorPool materialPool, const VulkanTexture& fallbackDiffuse,
           const VulkanTexture& fallbackNormal, TextureUploadCache& uploads, ImmediateBatch& batch)
    : m_indexCount(static_cast<uint32_t>(data.indices.size())), m_baseColor(data.baseColor),
      m_roughness(data.roughness), m_specularWeight(data.specularWeight),
      m_metalness(data.metalness), m_lobe1Roughness(data.lobe1Roughness),
      m_lobe2Roughness(data.lobe2Roughness), m_lobeRatio(data.lobeRatio),
      m_topCoatWeight(data.topCoatWeight), m_topCoatRoughness(data.topCoatRoughness),
      m_translucencyWeight(data.translucencyWeight), m_detailWeight(data.detailWeight),
      m_detailTiles(data.detailTiles), m_opacity(data.opacity),
      m_hasOpacityMask(data.hasOpacityMask) {
    // Bind-pose centroid: the transparent pass's back-to-front sort key.
    if (!data.vertices.empty()) {
        glm::vec3 sum(0.0f);
        for (const Vertex& v : data.vertices) {
            sum += v.pos;
        }
        m_centroid = sum / static_cast<float>(data.vertices.size());
    }

    const VkDeviceSize vertexBytes = sizeof(Vertex) * data.vertices.size();
    const VkDeviceSize indexBytes = sizeof(uint32_t) * data.indices.size();
    // Record all uploads into the shared batch instead of submitting per buffer/texture.
    m_vertexBuffer = createDeviceLocalBuffer(context, data.vertices.data(), vertexBytes,
                                             VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, batch);
    m_indexBuffer = createDeviceLocalBuffer(context, data.indices.data(), indexBytes,
                                            VK_BUFFER_USAGE_INDEX_BUFFER_BIT, batch);

    // Upload (or reuse — see TextureUploadCache) the diffuse map if the Qt layer decoded one for
    // this mesh; else use the white fallback.
    m_texture = uploadShared(context, uploads, data.diffuseImage, /*srgb=*/true, batch);
    // The detail (normal/bump) map — a LINEAR texture — and its mode. With no map we bind the
    // flat-normal fallback and force mode 0 (no perturbation).
    m_normalTexture = uploadShared(context, uploads, data.normalImage, /*srgb=*/false, batch);
    if (m_normalTexture) {
        m_normalMode = data.normalMode;
        m_normalStrength = data.normalStrength;
    }
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
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = materialPool;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &materialSetLayout;
    VK_CHECK(vkAllocateDescriptorSets(context.device(), &allocInfo, &m_materialSet));

    std::array<VkDescriptorImageInfo, 6> imageInfos{};
    imageInfos[0].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imageInfos[0].imageView = diffuse.imageView();
    imageInfos[0].sampler = diffuse.sampler();
    imageInfos[1].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imageInfos[1].imageView = detail.imageView();
    imageInfos[1].sampler = detail.sampler();
    imageInfos[2].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imageInfos[2].imageView = roughMap.imageView();
    imageInfos[2].sampler = roughMap.sampler();
    imageInfos[3].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imageInfos[3].imageView = specMask.imageView();
    imageInfos[3].sampler = specMask.sampler();
    imageInfos[4].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imageInfos[4].imageView = transMap.imageView();
    imageInfos[4].sampler = transMap.sampler();
    imageInfos[5].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imageInfos[5].imageView = microNormal.imageView();
    imageInfos[5].sampler = microNormal.sampler();

    std::array<VkWriteDescriptorSet, 6> writes{};
    for (uint32_t i = 0; i < writes.size(); ++i) {
        writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i].dstSet = m_materialSet;
        writes[i].dstBinding = i;
        writes[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[i].descriptorCount = 1;
        writes[i].pImageInfo = &imageInfos[i];
    }
    vkUpdateDescriptorSets(context.device(), static_cast<uint32_t>(writes.size()), writes.data(), 0,
                           nullptr);
}

void Mesh::record(VkCommandBuffer cmd, VkPipelineLayout layout, const glm::mat4& model,
                  MeshDrawKind kind, int highlightJoint, int highlightTwin) const {
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 1, 1, &m_materialSet, 0,
                            nullptr); // set 1 = this mesh's diffuse texture

    MeshPushConstants push{};
    push.model = model;
    push.baseColor = glm::vec4(m_baseColor, m_opacity);
    // z = metalness ("Metallic Weight" — scalar only, no map yet); w = the per-material specular
    // (F0) weight the material parser derived from the active specular lobe's reflectivity
    // (1 = standard 4% dielectric; skin ~0.5 → ~2%; OBJ meshes default to 1).
    // material.y packs the detail mode + its authored strength (floor = mode, fract×8 = strength;
    // the push block is at the 128-byte limit, so the pair shares one float).
    const float modePacked =
        static_cast<float>(m_normalMode) +
        std::clamp(m_normalStrength, 0.0f, 7.9f) * 0.125f * (m_normalMode > 0 ? 1.0f : 0.0f);
    push.material = glm::vec4(m_roughness, modePacked, m_metalness, m_specularWeight);
    push.material2 = glm::vec4(m_lobe1Roughness, m_lobe2Roughness, m_lobeRatio,
                               m_translucencyWeight);
    // material3.w packs the micro-detail layer (the block sits at the 128-byte push limit):
    // integer part = UV tiling, fraction = weight; 0 = no detail layer.
    const float detailPacked =
        (m_microNormalTexture && m_detailTiles >= 1.0f && m_detailWeight > 0.0f)
            ? std::floor(m_detailTiles) + std::min(m_detailWeight, 0.99f)
            : 0.0f;
    // material3.z packs the draw kind with the selected joint + its highlight twin (see the
    // declaration): kind + 8·(joint+1) + 8192·(twin+1), every field a small exact integer.
    const int packedKind = static_cast<int>(kind) +
                           8 * (std::clamp(highlightJoint, -1, 1022) + 1) +
                           8192 * (std::clamp(highlightTwin, -1, 1022) + 1);
    push.material3 = glm::vec4(m_topCoatWeight, m_topCoatRoughness,
                               static_cast<float>(packedKind), detailPacked);
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

void Mesh::reuploadVertices(VulkanContext& context, const std::vector<Vertex>& vertices,
                            ImmediateBatch& batch) {
    // Recreate the device-local vertex buffer with the re-morphed data (same count). The previous
    // buffer is freed on assignment; the caller has made the GPU idle so it isn't in use.
    const VkDeviceSize vertexBytes = sizeof(Vertex) * vertices.size();
    m_vertexBuffer = createDeviceLocalBuffer(context, vertices.data(), vertexBytes,
                                             VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, batch);
}

Model::Model(VulkanContext& context, const ModelData& data, VkDescriptorSetLayout materialSetLayout,
             VkDescriptorSetLayout jointSetLayout, const VulkanTexture& fallbackDiffuse,
             const VulkanTexture& fallbackNormal)
    : m_context(context) {
    uint32_t meshCount = 0;
    for (const MeshData& meshData : data.meshes) {
        if (!meshData.indices.empty()) {
            ++meshCount;
        }
    }
    if (meshCount == 0) {
        return; // nothing to draw; no pool/sets needed
    }

    // Compute a local-space AABB over every (non-empty) mesh's vertices for mouse picking.
    glm::vec3 lo(std::numeric_limits<float>::max());
    glm::vec3 hi(std::numeric_limits<float>::lowest());
    for (const MeshData& meshData : data.meshes) {
        if (meshData.indices.empty()) {
            continue;
        }
        for (const Vertex& vertex : meshData.vertices) {
            lo = glm::min(lo, vertex.pos);
            hi = glm::max(hi, vertex.pos);
        }
    }
    if (lo.x <= hi.x) { // at least one vertex seen
        m_boundsMin = lo;
        m_boundsMax = hi;
        m_hasBounds = true;
    }

    // Descriptors from this model's pool: one set-1 per mesh (six samplers each: diffuse, detail,
    // roughness map, spec mask, translucency map, micro-detail normal) + one set-2 joint set for
    // the whole model.
    std::array<VkDescriptorPoolSize, 2> poolSizes{};
    poolSizes[0].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    poolSizes[0].descriptorCount = meshCount * 6;
    poolSizes[1].type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    poolSizes[1].descriptorCount = kMaxFramesInFlight; // one joint set per frame in flight

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
    poolInfo.pPoolSizes = poolSizes.data();
    poolInfo.maxSets = meshCount + kMaxFramesInFlight;
    VK_CHECK(vkCreateDescriptorPool(m_context.device(), &poolInfo, nullptr, &m_materialPool));

    // One upload batch for the whole model: every mesh's vertex/index buffers and texture record
    // into it, and it's submitted exactly once below — collapsing what used to be ~3 blocking
    // submits per mesh (the dominant cost of importing a model with many material groups) into a
    // single GPU round-trip. Trade-off: all staging buffers are held until that submit completes,
    // so peak staging memory is the sum across meshes rather than one-at-a-time.
    const bool hasCorrectives = !data.correctives.empty();
    ImmediateBatch batch(m_context);
    // Meshes sharing a DecodedImage (zones sampling the same atlas file) share one VulkanTexture
    // through this cache — one staging upload + mip chain per unique image instead of per mesh.
    TextureUploadCache uploads;
    m_meshes.reserve(meshCount);
    std::vector<std::vector<uint32_t>> perMeshBaseIndex; // parallel to m_meshes (for corrective mapping)
    if (hasCorrectives) {
        m_baseVertices.reserve(meshCount);
        perMeshBaseIndex.reserve(meshCount);
    }
    const bool skinned = !data.bones.empty();
    if (skinned) {
        // Reserve the ground-sample store ONCE, summed across meshes. An exact-fit reserve inside
        // the per-mesh loop would reallocate-and-copy the whole vector every iteration (exact-fit
        // defeats geometric growth), turning a figure's ~10 MB of samples into O(zones × total)
        // redundant copying.
        std::size_t totalVerts = 0;
        for (const MeshData& meshData : data.meshes) {
            if (!meshData.indices.empty()) {
                totalVerts += meshData.vertices.size();
            }
        }
        m_groundSamples.reserve(totalVerts);
    }
    for (const MeshData& meshData : data.meshes) {
        if (meshData.indices.empty()) {
            continue;
        }
        m_meshes.emplace_back(m_context, meshData, materialSetLayout, m_materialPool, fallbackDiffuse,
                              fallbackNormal, uploads, batch);
        if (hasCorrectives) {
            m_baseVertices.push_back(meshData.vertices);    // uncorrected base, for re-morphing
            perMeshBaseIndex.push_back(meshData.baseVertex); // source base index per render vertex
        }
        if (skinned) {
            // Ground samples: the skinning inputs dropToGround() needs to find the posed lowest
            // point (the GPU buffers are device-local and unreadable).
            for (const Vertex& v : meshData.vertices) {
                m_groundSamples.push_back({v.pos, v.joints, v.weights});
            }
        }
    }
    batch.submitAndWait();

    if (hasCorrectives) {
        buildRuntimeCorrectives(data, perMeshBaseIndex);
    }

    // The armature — the runtime skeleton + pose (see armature.h) — built from the model's bones
    // (empty for a static model: one identity joint). Bind transforms are translation-only, so
    // each bone's local rest offset is the difference of the global rest positions.
    std::vector<ArmatureBone> armatureBones(data.bones.size());
    for (std::size_t i = 0; i < data.bones.size(); ++i) {
        const ModelBone& src = data.bones[i];
        ArmatureBone& dst = armatureBones[i];
        dst.name = src.name;
        dst.parent = src.parent;
        const glm::vec3 parentGlobal =
            (src.parent >= 0 && src.parent < static_cast<int>(data.bones.size()))
                ? glm::vec3(data.bones[static_cast<std::size_t>(src.parent)].bindGlobal[3])
                : glm::vec3(0.0f);
        dst.localBindTranslation = glm::vec3(src.bindGlobal[3]) - parentGlobal;
        dst.orientation = src.orientation;
        dst.rotationOrder = src.rotationOrder;
        dst.rotationMin = src.rotationMin;
        dst.rotationMax = src.rotationMax;
        dst.rotationLimited = src.rotationLimited;
    }
    m_armature.build(armatureBones);
    // Debug hook: POSESTUDIO_DUMP_SKELETON=<path> dumps the imported skeleton (one bone per
    // line) so the IK harness (tools/ikharness/) can run its drag-loop tests against REAL figure
    // rigs instead of synthetic approximations. No-op unless the environment variable is set.
    if (const char* dumpPath = std::getenv("POSESTUDIO_DUMP_SKELETON");
        dumpPath != nullptr && m_armature.hasSkeleton()) {
        m_armature.dump(dumpPath);
    }
    m_jointCount = m_armature.jointCount();

    // Host-mapped storage buffers of skinning DUAL QUATERNIONS (2 vec4 per joint — see
    // Armature::skinDualQuats) — ONE PER FRAME IN FLIGHT (a single shared buffer raced the GPU
    // and tore skinned frames). Each frame slot gets its own buffer + set-2 descriptor;
    // uploadJointsIfDirty() fills the current slot's buffer at record time, when that slot's
    // fence guarantees the GPU is done with it.
    std::array<VkDescriptorSetLayout, kMaxFramesInFlight> jointLayouts;
    jointLayouts.fill(jointSetLayout);
    VkDescriptorSetAllocateInfo jointAlloc{};
    jointAlloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    jointAlloc.descriptorPool = m_materialPool;
    jointAlloc.descriptorSetCount = kMaxFramesInFlight;
    jointAlloc.pSetLayouts = jointLayouts.data();
    VK_CHECK(vkAllocateDescriptorSets(m_context.device(), &jointAlloc, m_jointSets.data()));

    for (int f = 0; f < kMaxFramesInFlight; ++f) {
        m_jointBuffers[static_cast<std::size_t>(f)] =
            VulkanBuffer(m_context, m_jointCount * 2 * sizeof(glm::vec4),
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VMA_MEMORY_USAGE_AUTO,
                         VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                             VMA_ALLOCATION_CREATE_MAPPED_BIT);

        VkDescriptorBufferInfo jointInfo{};
        jointInfo.buffer = m_jointBuffers[static_cast<std::size_t>(f)].handle();
        jointInfo.offset = 0;
        jointInfo.range = m_jointCount * 2 * sizeof(glm::vec4);

        VkWriteDescriptorSet jointWrite{};
        jointWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        jointWrite.dstSet = m_jointSets[static_cast<std::size_t>(f)];
        jointWrite.dstBinding = 0;
        jointWrite.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
        jointWrite.descriptorCount = 1;
        jointWrite.pBufferInfo = &jointInfo;
        vkUpdateDescriptorSets(m_context.device(), 1, &jointWrite, 0, nullptr);
    }
}

void Model::uploadJointsIfDirty(uint32_t frameIndex) {
    const std::vector<glm::vec4>& quats = m_armature.skinDualQuats();
    if (quats.empty() || frameIndex >= static_cast<uint32_t>(kMaxFramesInFlight) ||
        m_jointUploaded[frameIndex] == m_armature.skinVersion()) {
        return;
    }
    auto* dst = static_cast<glm::vec4*>(m_jointBuffers[frameIndex].mappedData());
    if (dst == nullptr) {
        return;
    }
    const std::size_t bytes =
        std::min(quats.size(), std::size_t{2} * m_jointCount) * sizeof(glm::vec4);
    std::memcpy(dst, quats.data(), bytes);
    m_jointUploaded[frameIndex] = m_armature.skinVersion();
}

void Model::setBoneRotation(const std::string& boneName, const glm::vec3& eulerDegrees) {
    if (m_armature.setBoneRotation(boneName, eulerDegrees)) {
        refreshCorrectives();
    }
}

void Model::applyPose(const std::vector<std::pair<std::string, glm::vec3>>& pose) {
    m_armature.applyPose(pose);
    refreshCorrectives(); // re-morph for the restored pose
}

void Model::buildRuntimeCorrectives(const ModelData& data,
                                    const std::vector<std::vector<uint32_t>>& perMeshBaseIndex) {
    // Invert each mesh's baseVertex array: base vertex -> the render (mesh, local) vertices it fed. A
    // base vertex on a UV/zone seam feeds several render vertices (possibly across meshes), so a
    // corrective's single delta must reach all of them.
    std::unordered_map<uint32_t, std::vector<std::pair<uint32_t, uint32_t>>> baseToRender;
    for (uint32_t mi = 0; mi < perMeshBaseIndex.size(); ++mi) {
        const std::vector<uint32_t>& bidx = perMeshBaseIndex[mi];
        for (uint32_t lv = 0; lv < bidx.size(); ++lv) {
            baseToRender[bidx[lv]].emplace_back(mi, lv);
        }
    }

    std::unordered_set<uint32_t> affected;
    m_correctives.reserve(data.correctives.size());
    for (const PoseCorrective& pc : data.correctives) {
        RuntimeCorrective rc;
        rc.id = pc.id;
        rc.sumFormulas = pc.sumFormulas;
        rc.gateScale = pc.gateScale;
        rc.clamped = pc.clamped;
        rc.clampMin = pc.clampMin;
        rc.clampMax = pc.clampMax;
        std::unordered_map<uint32_t, std::size_t> slotOfMesh; // mesh index -> index into rc.meshDeltas
        for (const auto& [baseIdx, delta] : pc.deltas) {
            const auto it = baseToRender.find(baseIdx);
            if (it == baseToRender.end()) {
                continue; // delta targets a vertex not present in any rendered mesh
            }
            for (const auto& [mi, lv] : it->second) {
                std::size_t slot;
                const auto s = slotOfMesh.find(mi);
                if (s == slotOfMesh.end()) {
                    slot = rc.meshDeltas.size();
                    slotOfMesh.emplace(mi, slot);
                    RuntimeCorrective::MeshDelta md;
                    md.mesh = mi;
                    rc.meshDeltas.push_back(std::move(md));
                } else {
                    slot = s->second;
                }
                rc.meshDeltas[slot].localVertex.push_back(lv);
                rc.meshDeltas[slot].delta.push_back(delta);
            }
        }
        if (rc.meshDeltas.empty()) {
            continue; // none of this corrective's deltas landed on rendered geometry
        }
        for (const RuntimeCorrective::MeshDelta& md : rc.meshDeltas) {
            affected.insert(md.mesh);
        }
        m_correctives.push_back(std::move(rc));
    }

    m_correctiveWeight.assign(m_correctives.size(), 0.0f);
    m_correctiveMeshes.assign(affected.begin(), affected.end());
    std::sort(m_correctiveMeshes.begin(), m_correctiveMeshes.end());

    // Base vertices are only needed for meshes a corrective actually touches; free the rest (and all
    // of them if no corrective survived).
    if (m_correctives.empty()) {
        m_baseVertices.clear();
    } else {
        for (uint32_t mi = 0; mi < m_baseVertices.size(); ++mi) {
            if (affected.find(mi) == affected.end()) {
                std::vector<Vertex>().swap(m_baseVertices[mi]);
            }
        }
    }
}

float Model::evalCorrectiveWeight(std::size_t correctiveIndex) const {
    const RuntimeCorrective& rc = m_correctives[correctiveIndex];
    float sum = 0.0f;
    std::vector<float> st;
    st.reserve(8);
    for (const CorrectiveFormula& f : rc.sumFormulas) {
        st.clear();
        for (const CorrectiveOp& op : f.ops) {
            switch (op.kind) {
                case CorrectiveOp::Kind::PushRotation: {
                    float angle = 0.0f;
                    const int bone = m_armature.boneIndex(op.bone);
                    if (bone >= 0) {
                        angle = m_armature.boneEuler(static_cast<std::size_t>(bone))[op.axis];
                    }
                    st.push_back(angle);
                    break;
                }
                case CorrectiveOp::Kind::PushConst:
                    st.push_back(op.value);
                    break;
                case CorrectiveOp::Kind::Spline: {
                    const float d = st.empty() ? 0.0f : st.back();
                    if (!st.empty()) {
                        st.pop_back();
                    }
                    st.push_back(evalSpline(op.knots, d));
                    break;
                }
                case CorrectiveOp::Kind::Mult:
                    if (st.size() >= 2) {
                        const float b = st.back();
                        st.pop_back();
                        st.back() *= b;
                    }
                    break;
                case CorrectiveOp::Kind::Add:
                    if (st.size() >= 2) {
                        const float b = st.back();
                        st.pop_back();
                        st.back() += b;
                    }
                    break;
            }
        }
        sum += st.empty() ? 0.0f : st.back();
    }
    const float w = sum * rc.gateScale;
    // The channel clamp is part of the authored driver (see PoseCorrective::clamped): linear ramp
    // drivers rely on it to switch off outside their intended range — a knee-EXTENSION flexion
    // (rotation/x × -1/11) must stay 0 through the 155° of flexion, not run to -14.
    return rc.clamped ? glm::clamp(w, rc.clampMin, rc.clampMax) : w;
}

void Model::refreshCorrectives() {
    if (m_correctives.empty()) {
        return;
    }
    // Re-evaluate weights; bail if none moved enough to matter (the common case mid-drag / for a joint
    // that has no corrective).
    bool changed = false;
    for (std::size_t i = 0; i < m_correctives.size(); ++i) {
        const float w = evalCorrectiveWeight(i);
        if (std::fabs(w - m_correctiveWeight[i]) > 1e-4f) {
            changed = true;
        }
        m_correctiveWeight[i] = w;
    }
    if (!changed) {
        return;
    }
    // Diagnostic hook: POSESTUDIO_DUMP_CORRECTIVES=1 prints every corrective whose weight is
    // non-zero after a pose settles — which JCMs fire, and how hard. No-op unless set.
    if (std::getenv("POSESTUDIO_DUMP_CORRECTIVES") != nullptr) {
        std::fprintf(stderr, "[correctives] active after pose change:\n");
        for (std::size_t i = 0; i < m_correctives.size(); ++i) {
            if (std::fabs(m_correctiveWeight[i]) >= 1e-4f) {
                std::fprintf(stderr, "  %-48s w=%.3f\n", m_correctives[i].id.c_str(),
                             m_correctiveWeight[i]);
            }
        }
        std::fflush(stderr);
    }

    // Re-morph each affected mesh from its base and re-upload. The old buffers may be referenced by an
    // in-flight frame, so make the GPU idle first (this runs between frames on a settled pose, not per
    // drag-move, so the stall is infrequent).
    vkDeviceWaitIdle(m_context.device());
    ImmediateBatch batch(m_context);
    for (const uint32_t mi : m_correctiveMeshes) {
        std::vector<Vertex> verts = m_baseVertices[mi]; // start from the uncorrected base
        for (std::size_t c = 0; c < m_correctives.size(); ++c) {
            const float w = m_correctiveWeight[c];
            if (std::fabs(w) < 1e-4f) {
                continue;
            }
            for (const RuntimeCorrective::MeshDelta& md : m_correctives[c].meshDeltas) {
                if (md.mesh != mi) {
                    continue;
                }
                for (std::size_t k = 0; k < md.localVertex.size(); ++k) {
                    verts[md.localVertex[k]].pos += w * md.delta[k];
                }
            }
        }
        m_meshes[mi].reuploadVertices(m_context, verts, batch);
    }
    batch.submitAndWait();
}

Model::~Model() {
    // Destroying the pool frees all of this model's set-1 descriptors; the meshes (textures +
    // buffers) are freed afterwards as members. The device is idle by the time a Model dies
    // (Scene waits before tearing models down), so the freed sets are never referenced again.
    if (m_materialPool != VK_NULL_HANDLE) {
        vkDestroyDescriptorPool(m_context.device(), m_materialPool, nullptr);
    }
}

void Model::record(VkCommandBuffer cmd, VkPipelineLayout layout, bool transparentPass,
                   const glm::vec3& cameraPos, uint32_t frameIndex) {
    // A model whose meshes were all index-empty never created its pool/joint sets (the
    // constructor early-returns) — binding the VK_NULL_HANDLE set would be invalid Vulkan usage.
    if (m_meshes.empty()) {
        return;
    }
    // Set 2 = this frame slot's skin matrices (shared by all this model's meshes); upload the
    // current pose into the slot first if it hasn't seen it (no-op when the shadow pass already
    // did this frame).
    uploadJointsIfDirty(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 2, 1,
                            &m_jointSets[frameIndex], 0, nullptr);
    const glm::mat4& transform = m_armature.transform();
    const int selected = m_armature.selectedBone();
    const int twin = m_armature.selectedHighlightTwin();
    if (!transparentPass) {
        for (const Mesh& mesh : m_meshes) {
            if (!mesh.isTransparent()) {
                mesh.record(cmd, layout, transform, MeshDrawKind::Opaque, selected, twin);
            }
        }
        return;
    }
    // Transparent pass: back-to-front by (bind-pose) centroid distance, so layered shells —
    // lashes over cornea over sclera — blend in the right order from any viewing angle.
    std::vector<std::pair<float, const Mesh*>> order;
    for (const Mesh& mesh : m_meshes) {
        if (mesh.isTransparent()) {
            const glm::vec3 c = glm::vec3(transform * glm::vec4(mesh.centroid(), 1.0f));
            const glm::vec3 d = c - cameraPos;
            order.emplace_back(glm::dot(d, d), &mesh);
        }
    }
    std::sort(order.begin(), order.end(),
              [](const std::pair<float, const Mesh*>& a, const std::pair<float, const Mesh*>& b) {
                  return a.first > b.first; // farthest first
              });
    for (const auto& [distSq, mesh] : order) {
        mesh->record(cmd, layout, transform, MeshDrawKind::Transparent, selected, twin);
    }
}

void Model::recordWire(VkCommandBuffer cmd, VkPipelineLayout layout, uint32_t frameIndex) {
    if (m_meshes.empty()) {
        return;
    }
    uploadJointsIfDirty(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 2, 1,
                            &m_jointSets[frameIndex], 0, nullptr);
    const int twin = m_armature.selectedHighlightTwin();
    for (const Mesh& mesh : m_meshes) { // every mesh: cards and shells are geometry too
        mesh.record(cmd, layout, m_armature.transform(), MeshDrawKind::Wire,
                    m_armature.selectedBone(), twin);
    }
}

void Model::recordDepthFill(VkCommandBuffer cmd, VkPipelineLayout layout, uint32_t frameIndex) {
    if (m_meshes.empty()) {
        return;
    }
    uploadJointsIfDirty(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 2, 1,
                            &m_jointSets[frameIndex], 0, nullptr);
    for (const Mesh& mesh : m_meshes) {
        if (!mesh.isTransparent()) {
            mesh.record(cmd, layout, m_armature.transform(), MeshDrawKind::DepthOnly);
        }
    }
}

void Model::recordShadow(VkCommandBuffer cmd, VkPipelineLayout layout,
                         const glm::mat4& lightViewProj, uint32_t frameIndex) {
    if (m_meshes.empty()) {
        return;
    }
    // The shadow pipeline's only set (index 0) is the joint-matrix layout — bind the same set
    // object the main pass binds at index 2 (set compatibility is by layout, not index), so a
    // posed figure casts its posed shadow. This pass runs first in the frame, so it typically
    // performs the frame slot's joint upload.
    uploadJointsIfDirty(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1,
                            &m_jointSets[frameIndex], 0, nullptr);
    ShadowPushConstants push{};
    push.viewProj = lightViewProj;
    push.model = m_armature.transform();
    vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(push), &push);
    for (const Mesh& mesh : m_meshes) {
        if (!mesh.isTransparent()) { // no alpha in this pass — cards/shells would cast solid shadows
            mesh.recordDepth(cmd);
        }
    }
}

void Model::recordSilhouette(VkCommandBuffer cmd, VkPipelineLayout layout,
                             const glm::mat4& viewProj, uint32_t frameIndex) {
    if (m_meshes.empty()) {
        return;
    }
    // Same shader + set layout as the shadow pass (shadow.vert; joint set at index 0), projected
    // by the camera instead of the light — the outline traces the POSED silhouette.
    uploadJointsIfDirty(frameIndex);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1,
                            &m_jointSets[frameIndex], 0, nullptr);
    ShadowPushConstants push{};
    push.viewProj = viewProj;
    push.model = m_armature.transform();
    vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(push), &push);
    for (const Mesh& mesh : m_meshes) {
        if (!mesh.hasOpacityMask()) { // a cutout card's real shape lives in a map this pass can't read
            mesh.recordDepth(cmd);
        }
    }
}

bool Model::worldBounds(glm::vec3& outMin, glm::vec3& outMax) const {
    if (!m_hasBounds) {
        return false;
    }
    // Transform all 8 local AABB corners (the transform may rotate; min/max of the corners is the
    // tight world AABB of the local box).
    outMin = glm::vec3(std::numeric_limits<float>::max());
    outMax = glm::vec3(std::numeric_limits<float>::lowest());
    for (int i = 0; i < 8; ++i) {
        const glm::vec3 corner((i & 1) ? m_boundsMax.x : m_boundsMin.x,
                               (i & 2) ? m_boundsMax.y : m_boundsMin.y,
                               (i & 4) ? m_boundsMax.z : m_boundsMin.z);
        const glm::vec3 world = glm::vec3(m_armature.transform() * glm::vec4(corner, 1.0f));
        outMin = glm::min(outMin, world);
        outMax = glm::max(outMax, world);
    }
    // A POSED figure can leave its bind-pose box (a forward lean carries the head well outside
    // it), and the shadow frustum is fitted from these bounds — geometry crossing the fitted
    // map's edge showed as transient dark bands/streaks while posing. Union in the posed bone
    // positions, padded by a flesh margin (the skin extends past the joints — the skull above
    // the head joint, toes past the toe joints). Static models are exact via the box alone.
    if (m_armature.hasSkeleton()) {
        constexpr float kFleshMargin = 0.25f;
        glm::vec3 boneMin(std::numeric_limits<float>::max());
        glm::vec3 boneMax(std::numeric_limits<float>::lowest());
        for (std::size_t i = 0; i < m_armature.boneCount(); ++i) {
            const glm::vec3& p = m_armature.boneWorldPosition(i);
            boneMin = glm::min(boneMin, p);
            boneMax = glm::max(boneMax, p);
        }
        outMin = glm::min(outMin, boneMin - glm::vec3(kFleshMargin));
        outMax = glm::max(outMax, boneMax + glm::vec3(kFleshMargin));
    }
    return true;
}

bool Model::intersectRay(const Ray& ray, float& tOut) const {
    if (!m_hasBounds) {
        return false;
    }
    if (m_armature.hasSkeleton()) {
        // A POSED figure is picked against the world-space box of its live joints plus a flesh
        // margin, not its bind box: full-body IK walks and crouches move the whole figure
        // through the hip's pose translation, so after a walk the mesh sat far outside the
        // bind bounds — a right-click on the character found its joints (those are picked by
        // live position) but not the model, and the context menu lost its Delete entry.
        constexpr float kFleshMargin = 0.16f; // metres: the torso's depth around the spine
        glm::vec3 lo(std::numeric_limits<float>::max());
        glm::vec3 hi(std::numeric_limits<float>::lowest());
        for (std::size_t i = 0; i < m_armature.boneCount(); ++i) {
            const glm::vec3& p = m_armature.boneWorldPosition(i);
            lo = glm::min(lo, p);
            hi = glm::max(hi, p);
        }
        lo -= glm::vec3(kFleshMargin);
        hi += glm::vec3(kFleshMargin);
        float tMin = 0.0f;
        float tMax = std::numeric_limits<float>::max();
        for (int axis = 0; axis < 3; ++axis) {
            if (std::abs(ray.direction[axis]) < 1e-8f) {
                if (ray.origin[axis] < lo[axis] || ray.origin[axis] > hi[axis]) {
                    return false;
                }
                continue;
            }
            const float invD = 1.0f / ray.direction[axis];
            float t1 = (lo[axis] - ray.origin[axis]) * invD;
            float t2 = (hi[axis] - ray.origin[axis]) * invD;
            if (t1 > t2) {
                std::swap(t1, t2);
            }
            tMin = std::max(tMin, t1);
            tMax = std::min(tMax, t2);
            if (tMin > tMax) {
                return false;
            }
        }
        tOut = tMin;
        return true;
    }
    // Transform the ray into local space so we can slab-test the AABB directly (equivalent to an
    // oriented-box test in world space, but cheaper). The transform is affine, so the ray
    // parameter t is preserved — the t we find is the same world-space distance for every model.
    const glm::mat4 inv = glm::inverse(m_armature.transform());
    const glm::vec3 o = glm::vec3(inv * glm::vec4(ray.origin, 1.0f));
    const glm::vec3 d = glm::vec3(inv * glm::vec4(ray.direction, 0.0f));

    float tMin = 0.0f; // ignore hits behind the ray origin
    float tMax = std::numeric_limits<float>::max();
    for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(d[axis]) < 1e-8f) {
            // Ray parallel to this slab: miss unless the origin is already within it.
            if (o[axis] < m_boundsMin[axis] || o[axis] > m_boundsMax[axis]) {
                return false;
            }
            continue;
        }
        const float invD = 1.0f / d[axis];
        float t1 = (m_boundsMin[axis] - o[axis]) * invD;
        float t2 = (m_boundsMax[axis] - o[axis]) * invD;
        if (t1 > t2) {
            std::swap(t1, t2);
        }
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        if (tMin > tMax) {
            return false;
        }
    }
    tOut = tMin;
    return true;
}

bool Model::dropToGround() {
    float minY = 0.0f;
    if (!groundGap(minY) || std::abs(minY) < 1e-4f) {
        return false; // nothing sampled, or already resting on the floor
    }
    translateY(-minY);
    return true;
}

bool Model::groundGap(float& lowestY) const {
    float minY = std::numeric_limits<float>::max();
    if (m_armature.hasSkeleton() && !m_groundSamples.empty()) {
        // Posed lowest point: CPU-skin every ground sample with the CURRENT pose's skin matrices
        // (poseGlobal · inverseBind — the armature's poseGlobal always holds the last pose
        // update) and track the world-space minimum. A one-shot ~few-ms walk; corrective deltas
        // are ignored (they move contact regions by millimetres at most). Linear matrix blending
        // is fine here even though the GPU skins with dual quaternions: contact regions (feet,
        // knees) are near-single-joint weighted, where LBS and DQS agree exactly.
        const std::size_t boneCount = m_armature.boneCount();
        std::vector<glm::mat4> skin(boneCount);
        for (std::size_t i = 0; i < boneCount; ++i) {
            skin[i] = m_armature.poseGlobal(i) * m_armature.inverseBind(i);
        }
        for (const GroundSample& s : m_groundSamples) {
            const glm::vec4 p(s.pos, 1.0f);
            glm::vec3 posed(0.0f);
            for (int j = 0; j < 4; ++j) {
                const float w = s.weights[j];
                if (w > 0.0f && s.joints[j] < skin.size()) {
                    posed += w * glm::vec3(skin[s.joints[j]] * p);
                }
            }
            minY = std::min(minY, (m_armature.transform() * glm::vec4(posed, 1.0f)).y);
        }
    } else if (m_hasBounds) {
        // Static model: the bind AABB through the transform is exact (it can't pose).
        for (int i = 0; i < 8; ++i) {
            const glm::vec3 c((i & 1) ? m_boundsMax.x : m_boundsMin.x,
                              (i & 2) ? m_boundsMax.y : m_boundsMin.y,
                              (i & 4) ? m_boundsMax.z : m_boundsMin.z);
            minY = std::min(minY, (m_armature.transform() * glm::vec4(c, 1.0f)).y);
        }
    } else {
        return false; // no geometry to ground
    }
    if (!(minY < std::numeric_limits<float>::max())) {
        return false; // nothing sampled
    }
    lowestY = minY;
    return true;
}

} // namespace pose
