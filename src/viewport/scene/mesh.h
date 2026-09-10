/**
 * @file mesh.h
 * @brief GPU-resident geometry of one material group (Mesh), and the push-constant blocks the
 *        mesh shaders read.
 *
 * A Mesh owns device-local vertex + index buffers, its material parameters (base color, roughness,
 * dual-lobe specular, top coat, translucency, opacity — baked once into a push-constant template),
 * and its six texture maps — diffuse, detail normal/bump, roughness, spec mask, translucency, and
 * micro-detail (pore) normal, each falling back to a shared 1x1 texture when absent — bound through
 * its own descriptor set (set 1, six samplers). A Model (model.h) groups the meshes of one
 * imported file. Qt-free.
 */

#ifndef MESH_H
#define MESH_H

#include "vulkanbuffer.h"

#include <glm/glm.hpp>
#include <vulkan/vulkan.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace pose {

class ImmediateBatch;
class VulkanContext;
class VulkanTexture;
struct MeshData;
struct TextureUploadCache; // per-model dedup of texture uploads (textureuploadcache.h)

/// Per-draw push-constant block for the mesh shaders. Must stay byte-identical to the
/// `push_constant` block in mesh.frag (mat4 + 4×vec4 = 128 bytes — exactly the guaranteed
/// push-constant minimum, so no further vec4 fits; mesh.vert declares the shorter prefix).
struct MeshPushConstants {
    glm::mat4 model;
    glm::vec4 baseColor; // rgb tint; a = opacity (1 = opaque)
    glm::vec4 material;  // x = roughness (ratio-blended), y = normalMode (0/1/2) packed with the
                         // authored detail-map strength as mode + strength/8 (floor = mode,
                         // fract×8 = strength — the block is at the 128-byte push limit),
                         // z = metalness, w = specular (F0) weight (1 = 4% dielectric)
    glm::vec4 material2; // x = lobe1 rough, y = lobe2 rough, z = lobe ratio (1 = single-lobe),
                         // w = translucency weight
    glm::vec4 material3; // x = top-coat weight (0 = none), y = top-coat roughness,
                         // z = the draw kind (MeshDrawKind: opaque = PBR writes the SSS mask into
                         //     alpha, transparent = alpha stays the blend opacity, wire, depth
                         //     fill) packed with the selected joint + its highlight twin as
                         //     kind + 8·(joint+1) + 8192·(twin+1) — see Mesh::record,
                         // w = micro-detail packed as floor(tiles) + fract(weight) — the block is
                         //     at the 128-byte push limit, so the pair shares one float (0 = none)
};

/// What a Mesh::record draw is for — pushed as mesh.frag's material3.z. Opaque/Transparent select
/// the alpha semantics (SSS mask vs blend opacity) in the lit passes; Wire is the wireframe
/// overlay (every edge in a flat grey from the UBO's wire level, through the LINE-polygon-mode
/// pipeline); DepthOnly is the hidden-line surface fill (its pipeline masks every colour write,
/// so the shader returns at once — the draw exists to lay down depth).
enum class MeshDrawKind { Opaque = 0, Transparent = 1, Wire = 2, DepthOnly = 3 };

/// Push-constant block for the position-only projected passes — the depth-only shadow pass and
/// the selection-outline mask pass, which share shadow.vert. Must stay byte-identical to that
/// shader's `push_constant` block (two mat4s = 128 bytes, the guaranteed push-constant minimum).
struct ShadowPushConstants {
    glm::mat4 viewProj; // the fitted light matrix (shadow) or the camera's (outline mask)
    glm::mat4 model;
};

/// One material group of a model: device-local vertex/index buffers, material params (base color,
/// roughness, opacity, dual-lobe specular, top coat, translucency), and its six texture maps bound
/// through its own set-1 descriptor (six samplers, each with a shared 1x1 fallback when absent).
class Mesh {
public:
    /// @param materialSetLayout  The shared set-1 layout (Scene owns it).
    /// @param materialPool       The owning Model's pool to allocate this mesh's set from.
    /// @param fallbackDiffuse    Shared 1x1 white texture, bound when the mesh has no diffuse map.
    /// @param fallbackNormal     Shared 1x1 flat-normal texture, bound when the mesh has no detail map.
    /// @param uploads            The owning Model's texture-upload cache: meshes sharing a
    ///                           DecodedImage share one VulkanTexture (figure zones commonly sample
    ///                           the same atlas maps — uploading per mesh multiplied GPU memory and
    ///                           import time by the sharing factor).
    /// @param batch              Upload batch the mesh's buffers/textures record into (the Model
    ///                           flushes it once for all its meshes — one submit per import).
    /// @param correctiveRanges   Per-vertex packed pose-corrective ranges (Vertex::correctiveRange,
    ///                           one per data.vertices entry) the Model resolved, or nullptr when
    ///                           no corrective touches this mesh. They're stamped into the vertex
    ///                           data as it uploads — the importer's copy stays pristine.
    Mesh(VulkanContext& context, const MeshData& data, VkDescriptorSetLayout materialSetLayout,
         VkDescriptorPool materialPool, const VulkanTexture& fallbackDiffuse,
         const VulkanTexture& fallbackNormal, TextureUploadCache& uploads, ImmediateBatch& batch,
         const std::vector<uint32_t>* correctiveRanges = nullptr);

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    Mesh(Mesh&&) noexcept = default;
    Mesh& operator=(Mesh&&) noexcept = default;

    /// Binds set 1 (this mesh's textures), pushes (model, baseColor, material), binds buffers, and
    /// draws. The caller has already bound the matching pipeline and the per-frame camera set
    /// (set 0). @p kind tells the shader what the draw is (alpha semantics / wire / depth fill);
    /// @p highlightJoint / @p highlightTwin (-1 = none) are the SELECTED joint and its twist
    /// partner — the vertex stage sums the vertex's skin weights on them into the highlight
    /// amount the fragment tints in the selection colour (the "you grabbed this part" cue).
    /// All three ride in material3.z, packed as kind + 8·(joint+1) + 8192·(twin+1) (3 + 10 + 10
    /// bits, exact in a float): the push block sits at the 128-byte limit, so the three small
    /// integers share one float.
    void record(VkCommandBuffer cmd, VkPipelineLayout layout, const glm::mat4& model,
                MeshDrawKind kind, int highlightJoint = -1, int highlightTwin = -1) const;

    /// Depth-only draw for the shadow pass: just buffers + drawIndexed — no material set, no push
    /// (the Model pushes the light matrices once for all its meshes).
    void recordDepth(VkCommandBuffer cmd) const;

    /// True if this mesh is (partly) see-through and must be drawn in the transparent pass. An
    /// opacity mask baked into the diffuse alpha makes a mesh transparent even at scalar opacity 1
    /// (a lash/brow card's shape exists only in the mask).
    bool isTransparent() const { return m_opacity < 0.999f || m_hasOpacityMask; }
    /// True if the mesh's shape is (partly) defined by an opacity mask in its diffuse alpha — a
    /// lash/brow card, an older figure's eye-shell fade. Material-less passes can't honour the
    /// mask, so they skip such meshes (their solid card geometry is not the shape the user sees).
    bool hasOpacityMask() const { return m_hasOpacityMask; }

    /// Bind-pose centroid (local space) — the transparent pass's back-to-front sort key, carried
    /// through the dominant joint's pose by Model::posedCentroid.
    const glm::vec3& centroid() const { return m_centroid; }
    /// The joint carrying most of this mesh's summed skin weight (0 for an unskinned mesh) —
    /// whose skin transform moves the centroid with the pose for the transparency sort.
    uint32_t dominantJoint() const { return m_dominantJoint; }

private:
    VulkanBuffer                   m_vertexBuffer;
    VulkanBuffer                   m_indexBuffer;
    uint32_t                       m_indexCount = 0;
    glm::vec3                      m_centroid{0.0f};   // bind-pose local centroid (transparency sort)
    uint32_t                       m_dominantJoint = 0; // see dominantJoint()
    float                          m_opacity = 1.0f;
    bool                           m_hasOpacityMask = false;
    // The per-mesh constants of the push block (baseColor + opacity, the material scalars, the
    // packed detail mode/strength and micro-detail layer), baked once at construction; record()
    // copies it and patches only the model matrix and the draw-kind/highlight field.
    MeshPushConstants              m_pushTemplate{};
    // Textures are shared_ptr: meshes sampling the same DecodedImage share one upload (see the
    // TextureUploadCache note on the constructor).
    std::shared_ptr<VulkanTexture> m_texture;                     // diffuse; null => white fallback
    std::shared_ptr<VulkanTexture> m_normalTexture;               // detail; null => flat-normal fallback
    std::shared_ptr<VulkanTexture> m_roughnessTexture;            // linear ×-multiplier map; null => white
    std::shared_ptr<VulkanTexture> m_specMaskTexture;             // linear spec mask; null => white
    std::shared_ptr<VulkanTexture> m_translucencyTexture;         // sRGB translucency map; null => white
    std::shared_ptr<VulkanTexture> m_microNormalTexture;          // linear tiled pore normals; null => flat
    VkDescriptorSet                m_materialSet = VK_NULL_HANDLE; // set 1; owned by the Model's pool
};

} // namespace pose

#endif // MESH_H
