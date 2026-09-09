/**
 * @file mesh.h
 * @brief GPU-resident geometry: a Mesh (one material group) and a Model (one imported file).
 *
 * A Mesh owns device-local vertex + index buffers, its material parameters (base color, roughness,
 * dual-lobe specular, top coat, translucency, opacity), and its six texture maps — diffuse, detail
 * normal/bump, roughness, spec mask, translucency, and micro-detail (pore) normal, each falling
 * back to a shared 1x1 texture when absent — bound through its own descriptor set (set 1, six
 * samplers). A Model groups the meshes of one imported file under a shared transform, owns the
 * descriptor pool those sets are allocated from, and carries an Armature (armature.h) — the
 * runtime skeleton, pose, pins, and full-body IK of a skinned figure — whose skin data it
 * uploads per frame. Qt-free.
 */

#ifndef MESH_H
#define MESH_H

#include "armature.h"  // the skeleton + pose + IK a Model poses through
#include "modeldata.h" // CorrectiveFormula (runtime corrective evaluation)
#include "vulkanbuffer.h"
#include "vulkancommon.h" // kMaxFramesInFlight (per-frame joint buffers)
#include "vulkanimage.h"

#include <glm/glm.hpp>
#include <vulkan/vulkan.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pose {

class VulkanContext;
class ImmediateBatch;
class IkRig; // full-body-IK orchestrator (scene/ik/ikrig.h); built lazily on first IK drag
struct Ray;
struct TextureUploadCache; // per-model dedup of texture uploads (defined in mesh.cpp)

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
                         // z = transparent-pass flag (0 = opaque: PBR writes the SSS mask into
                         // alpha; 1 = transparent: alpha stays the blend opacity),
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

    /// Binds set 1 (this mesh's texture), pushes (model, baseColor, material), binds buffers, and
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

    /// Bind-pose centroid (local space) — the transparent pass's back-to-front sort key.
    const glm::vec3& centroid() const { return m_centroid; }

private:
    VulkanBuffer                   m_vertexBuffer;
    VulkanBuffer                   m_indexBuffer;
    uint32_t                       m_indexCount = 0;
    glm::vec3                      m_centroid{0.0f}; // bind-pose local centroid (transparency sort)
    glm::vec3                      m_baseColor{0.8f};
    float                          m_roughness = 0.7f;
    float                          m_specularWeight = 1.0f; // F0 multiplier (1 = 4% dielectric)
    float                          m_metalness = 0.0f;
    float                          m_lobe1Roughness = 0.7f; // dual-lobe spec (ratio 1 = single lobe)
    float                          m_lobe2Roughness = 0.7f;
    float                          m_lobeRatio = 1.0f;
    float                          m_topCoatWeight = 0.0f;
    float                          m_topCoatRoughness = 0.65f;
    float                          m_translucencyWeight = 0.5f;
    float                          m_detailWeight = 0.0f; // micro-detail normal strength (0 = none)
    float                          m_detailTiles = 0.0f;  // micro-detail UV tiling
    float                          m_opacity = 1.0f;
    bool                           m_hasOpacityMask = false;
    int                            m_normalMode = 0;              // 0 none / 1 normal map / 2 bump map
    float                          m_normalStrength = 1.0f;       // authored detail-map strength
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

/// One imported model (OBJ mesh or rigged figure): its meshes (each a material group) plus an
/// Armature — the shared model transform and, for a figure, the runtime skeleton, pose, pins,
/// and full-body IK (see armature.h). Owns the descriptor pool the meshes' set-1 descriptors are
/// allocated from and the per-frame joint buffers the armature's skin data is uploaded into.
class Model {
public:
    Model(VulkanContext& context, const ModelData& data, VkDescriptorSetLayout materialSetLayout,
          VkDescriptorSetLayout jointSetLayout, const VulkanTexture& fallbackDiffuse,
          const VulkanTexture& fallbackNormal);
    ~Model();

    Model(const Model&) = delete;
    Model& operator=(const Model&) = delete;

    /// Records the model's sub-meshes at its current transform. @p transparentPass selects which
    /// meshes are drawn: false = only opaque meshes, true = only transparent ones (sorted
    /// back-to-front by centroid distance to @p cameraPos, so layered shells — lashes over cornea
    /// over sclera — blend correctly from any angle). The caller draws the whole scene's opaque
    /// pass first, then binds the blend pipeline for the transparent pass. @p frameIndex selects
    /// this frame-in-flight's joint buffer (see m_jointBuffers) and uploads pending skin
    /// matrices into it.
    void record(VkCommandBuffer cmd, VkPipelineLayout layout, bool transparentPass,
                const glm::vec3& cameraPos, uint32_t frameIndex);

    /// Records EVERY mesh as wireframe edges (the caller has bound the wire pipeline): the
    /// wireframe shade modes' edge overlay, or their whole drawing when there is no surface.
    void recordWire(VkCommandBuffer cmd, VkPipelineLayout layout, uint32_t frameIndex);

    /// Records the OPAQUE meshes as a depth-only surface fill (the caller has bound the
    /// hidden-line pipeline, whose colour writes are masked): the surface occludes what lies
    /// behind it — the grid, the far side's wires — while showing the viewport's clear colour.
    /// Transparent meshes are left out so an eye's iris wires stay visible through its shells.
    void recordDepthFill(VkCommandBuffer cmd, VkPipelineLayout layout, uint32_t frameIndex);

    /// Records the model's opaque meshes into the depth-only shadow pass (the caller has bound the
    /// shadow pipeline and begun its render pass). Transparent meshes (clear eye shells, cutout
    /// lash/brow cards) are skipped — the pass has no alpha, so they'd cast solid-card shadows.
    void recordShadow(VkCommandBuffer cmd, VkPipelineLayout layout,
                      const glm::mat4& lightViewProj, uint32_t frameIndex);

    /// Records the model's silhouette into the selection-outline mask pass (the caller has
    /// bound the mask pipeline and begun its render pass): every mesh, skinned and projected by
    /// the camera's @p viewProj, except opacity-masked cards (lashes/brows), whose solid card
    /// geometry would put rectangular bumps on the outline — the shape the user sees exists only
    /// in a texture this material-less pass doesn't sample. Translucent shells (a cornea) ARE
    /// drawn: a selected translucent object still has a silhouette.
    void recordSilhouette(VkCommandBuffer cmd, VkPipelineLayout layout, const glm::mat4& viewProj,
                          uint32_t frameIndex);

    /// The model's world-space AABB (local bounds through the model transform). Returns false when
    /// the model has no geometry bounds. Used to fit the key light's shadow frustum.
    bool worldBounds(glm::vec3& outMin, glm::vec3& outMax) const;

    /// The armature: the skeleton, the pose, the user pins, and the full-body IK — see
    /// armature.h. Every posing call below is a thin forwarder into it (the ones that change the
    /// pose also settle this model's correctives, which the armature can't know about); new
    /// posing behaviour belongs in the armature, never here.
    Armature&       armature() { return m_armature; }
    const Armature& armature() const { return m_armature; }
    /// The model matrix (owned by the armature — IK, picking, and grounding all need it).
    const glm::mat4& transform() const { return m_armature.transform(); }

    /// Poses the joint @p boneName by an Euler rotation (degrees) applied in its local frame, then
    /// recomputes the skin matrices and settles the correctives. No-op if the model has no such
    /// bone. This is the primitive a posing UI drives; @p eulerDegrees of 0 restores the rest pose.
    void setBoneRotation(const std::string& boneName, const glm::vec3& eulerDegrees);

    /// Whether this model has a skeleton (i.e. is a posable figure rather than a static mesh).
    bool hasSkeleton() const { return m_armature.hasSkeleton(); }

    // --- Posing-UI support (world-space skeleton for overlay/picking + interactive rotation) ---
    std::size_t      boneCount() const { return m_armature.boneCount(); }
    /// Current world-space position of each joint (updated whenever the pose changes).
    const glm::vec3& boneWorldPosition(std::size_t i) const { return m_armature.boneWorldPosition(i); }
    int              boneParent(std::size_t i) const { return m_armature.boneParent(i); }
    const std::string& boneName(std::size_t i) const { return m_armature.boneName(i); }

    int  selectedBone() const { return m_armature.selectedBone(); }
    void setSelectedBone(int index) { m_armature.setSelectedBone(index); }
    /// Selects the bone named @p name (diagnostics / the IK benchmark); its index, or -1.
    int selectBoneByName(const std::string& name) { return m_armature.selectBoneByName(name); }
    /// Adds @p deltaEulerDegrees to the selected bone's accumulated rotation and re-poses it.
    void nudgeSelectedBone(const glm::vec3& deltaEulerDegrees) {
        m_armature.nudgeSelectedBone(deltaEulerDegrees);
    }

    // --- Full-body IK (scene/ik/, driven through the armature): drag a joint, the body follows ---
    bool beginIkDrag() { return m_armature.beginIkDrag(); }
    bool dragIkTo(const glm::vec3& targetWorld) { return m_armature.dragIkTo(targetWorld); }
    bool settleIkTick() { return m_armature.settleIkTick(); }
    /// Ends the FBIK drag (the solved pose stays; the caller settles correctives).
    void endIkDrag() { m_armature.endIkDrag(); }
    // --- User joint pins (see Armature) ---
    bool togglePinSelectedBone() { return m_armature.togglePinSelectedBone(); }
    bool isBonePinned(std::size_t index) const { return m_armature.isBonePinned(index); }
    bool selectedBonePinned() const { return m_armature.selectedBonePinned(); }
    bool hasPinnedBones() const { return m_armature.hasPinnedBones(); }
    void unpinAllBones() { m_armature.unpinAllBones(); }
    // --- Pose utilities (see Armature): reset / mirror. The correctives follow at the next
    // recorded frame like any pose change; the caller's finalizePose is the settled-pose hook.
    bool resetSelectedBone(bool subtree) {
        return m_armature.resetBone(m_armature.selectedBone(), subtree);
    }
    void resetPose() { m_armature.resetPose(); }
    void mirrorPose() { m_armature.mirrorPose(); }
    bool mirrorSelectedLimb() {
        return m_armature.mirrorSubtreeToOpposite(m_armature.selectedBone());
    }
    /// The rig's CONTACT pins (ground-detected, not user pins) while an IK drag is active — for
    /// the overlay's "which feet are planted" markers. Empty outside a drag.
    std::vector<int> activeContactPins() const { return m_armature.activeContactPins(); }
    /// The accumulated pose rotation of bone @p i (Euler degrees, its rotation order).
    const glm::vec3& boneEuler(std::size_t i) const { return m_armature.boneEuler(i); }

    /// Captures the current pose as (bone name, Euler degrees) for each non-rest joint (for saving).
    std::vector<std::pair<std::string, glm::vec3>> capturePose() const {
        return m_armature.capturePose();
    }
    /// Resets to bind pose, then applies @p pose (bone name -> Euler degrees; unknown bones
    /// ignored) and re-morphs the correctives for the restored pose.
    void applyPose(const std::vector<std::pair<std::string, glm::vec3>>& pose);

    /// Whether this model carries pose correctives (joint-driven corrective morphs).
    bool hasCorrectives() const { return !m_correctives.empty(); }
    /// Re-evaluates the pose correctives' weights against the current pose now (the GPU blend
    /// picks them up at the next recorded frame) and, with POSESTUDIO_DUMP_CORRECTIVES set,
    /// prints the active ones. No-op without correctives. The correctives no longer NEED this to
    /// show: every recorded frame re-evaluates them from the live pose (uploadJointsIfDirty), so
    /// they track a drag continuously — this is the explicit "pose settled" hook
    /// (Scene::finalizePose, setBoneRotation, applyPose): cheap, and where the dump lives.
    void refreshCorrectives();

    /// Tests @p ray (world space) against the model's axis-aligned bounding box (transformed by the
    /// model matrix). On a hit, writes the entry distance (a world-space ray parameter, so values
    /// are comparable across models) to @p tOut and returns true. Box-level precision — enough to
    /// pick which imported object the cursor is over.
    bool intersectRay(const Ray& ray, float& tOut) const;

    /// Translates the model vertically so its lowest point rests on the ground plane (y = 0). For a
    /// skinned figure that is the CURRENT POSE's lowest point (CPU-skinned from the compact ground
    /// samples kept at construction — a knee bend lifts the feet, so the bind AABB would ground
    /// wrong); a static model uses its bind AABB through the transform (exact — it can't pose).
    /// Returns true if the transform actually changed (false when already grounded / no geometry).
    bool dropToGround();
    /// The world height of the CURRENT pose's lowest point (the same scan dropToGround uses):
    /// positive = hovering that far above the floor, negative = sunk into it. False when there is
    /// no geometry to measure. The viewport animates the ground button's drop from this.
    bool groundGap(float& lowestY) const;
    /// Translates the model by @p dy along world Y and refreshes the transform-dependent bone
    /// positions (the animated ground drop applies its per-frame fall increments through this).
    void translateY(float dy) { m_armature.translateY(dy); }

private:
    /// Evaluates one corrective's blend weight from the current pose (Σ sumFormulas × gateScale).
    float evalCorrectiveWeight(std::size_t correctiveIndex) const;
    /// Re-evaluates every corrective's weight from the current pose; returns whether any moved
    /// (past a 1e-4 threshold) and, if so, bumps m_correctiveVersion so the frame slots re-upload.
    bool evaluateCorrectiveWeights();
    /// Writes the current corrective weights into frame slot @p frameIndex's buffer if that slot
    /// hasn't seen m_correctiveVersion — re-evaluating them first when the pose moved since the
    /// last evaluation (Armature::skinVersion is the pose's change counter).
    void uploadCorrectiveWeightsIfDirty(uint32_t frameIndex);

    /// One (corrective, displacement) entry of the GPU delta buffer — the vertex shaders'
    /// CorrectiveDelta (std430: uint + 3 floats, 16 bytes). Entries are grouped per render
    /// vertex; Vertex::correctiveRange addresses a vertex's run.
    struct CorrectiveEntry {
        uint32_t  corrective; ///< Index into m_correctives / the weights buffer.
        glm::vec3 delta;      ///< Bind-space displacement at weight 1 (model units).
    };
    static_assert(sizeof(CorrectiveEntry) == 16, "must match the shaders' CorrectiveDelta");

    /// Resolves the model's correctives for the GPU: keeps each one's driver formulas (bone
    /// names pre-resolved to indices) and maps every base-vertex delta through each mesh's
    /// baseVertex array onto the render vertices (a seam vertex feeds several), producing the
    /// flat, per-vertex-grouped @p entries and, per non-empty mesh, the packed range each of its
    /// vertices carries (@p perMeshRanges — parallel to the meshes the constructor builds next;
    /// an untouched mesh's vector stays empty). Runs once at construction, BEFORE the meshes
    /// upload, because the ranges ride in the vertex data.
    void buildRuntimeCorrectives(const ModelData& data,
                                 std::vector<std::vector<uint32_t>>& perMeshRanges,
                                 std::vector<CorrectiveEntry>& entries);

    // One pose corrective resolved for this model: its driver formulas in joint-angle terms (the
    // deltas themselves live in the GPU delta buffer). opBone[f][k] is the pre-resolved bone
    // index of sumFormulas[f].ops[k] (a PushRotation); -1 for other ops / unknown bones.
    struct RuntimeCorrective {
        std::string                    id; // corrective morph id (diagnostics)
        std::vector<CorrectiveFormula> sumFormulas;
        std::vector<std::vector<int>>  opBone;
        float                          gateScale = 1.0f;
        bool                           clamped = false; // clamp the driven weight to [min,max]
        float                          clampMin = 0.0f;
        float                          clampMax = 1.0f;
    };

    VulkanContext&       m_context;
    VkDescriptorPool     m_materialPool = VK_NULL_HANDLE;
    std::vector<Mesh>    m_meshes;
    glm::vec3            m_boundsMin{0.0f}; // local-space AABB over all mesh vertices
    glm::vec3            m_boundsMax{0.0f};
    bool                 m_hasBounds = false;

    // The skeleton + pose + pins + IK (and the model transform) — see armature.h.
    Armature             m_armature;

    // Skinning upload: the armature's dual quaternions (see Armature::skinDualQuats) reach the
    // GPU through a storage buffer (set 2) bound per draw — ONE PER FRAME IN FLIGHT: interactive
    // posing rewrites the data at 60 Hz, and a single shared buffer was overwritten while a
    // still-executing frame READ it — vertices skinned by half-updated matrices streaked into
    // momentary exploded-geometry artifacts (and the same torn frame corrupted the shadow
    // pass). At record time this frame slot's fence has been waited on, so writing ITS buffer
    // can never race the GPU.
    std::array<VulkanBuffer, kMaxFramesInFlight>    m_jointBuffers;
    std::array<VkDescriptorSet, kMaxFramesInFlight> m_jointSets{}; // set 2; from m_materialPool
    std::array<std::uint64_t, kMaxFramesInFlight>   m_jointUploaded{}; // last skin version per slot
    uint32_t                                        m_jointCount = 1;

    /// Uploads the armature's skin data into frame slot @p frameIndex's joint buffer if that slot
    /// hasn't seen the current Armature::skinVersion yet, and likewise the pose-corrective
    /// weights (uploadCorrectiveWeightsIfDirty). Called by every record path — whichever runs
    /// first in a frame does the copies, the rest no-op.
    void uploadJointsIfDirty(uint32_t frameIndex);

    // Pose correctives, blended ON THE GPU (set 2 bindings 1 + 2 — see mesh.vert): the resolved
    // correctives (driver formulas), each one's current weight, the static device-local buffer of
    // per-vertex (corrective, delta) entries, and — ONE PER FRAME IN FLIGHT, like the joints — the
    // host-mapped weight buffers the vertex shaders read. At record time the weights are
    // re-evaluated whenever the pose moved (Armature::skinVersion) and copied into the current
    // frame slot when it is behind m_correctiveVersion: a weight change costs a few hundred bytes
    // per frame, never a geometry re-upload. (The CPU re-morph this replaced had to idle the
    // device and rebuild whole vertex buffers, so correctives could only run once a drag ENDED —
    // the figure visibly deformed mid-drag and snapped right on release.) A model without
    // correctives keeps 16-byte zero placeholders bound so its set 2 is complete.
    std::vector<RuntimeCorrective>                  m_correctives;
    std::vector<float>                              m_correctiveWeight; // current weight per corrective
    VulkanBuffer                                    m_correctiveDeltaBuffer;
    std::array<VulkanBuffer, kMaxFramesInFlight>    m_correctiveWeightBuffers;
    std::array<std::uint64_t, kMaxFramesInFlight>   m_correctiveUploaded{}; // weight version per slot
    std::uint64_t                                   m_correctiveVersion = 1;
    std::uint64_t                                   m_correctiveEvalSkinVersion = ~std::uint64_t{0};

    // Compact CPU copy of every render vertex's skinning inputs (position + joints + weights),
    // kept only for skinned models, so dropToGround() can find the CURRENT pose's lowest point
    // without reading back device-local buffers. ~44 bytes/vertex (~10 MB for a figure).
    struct GroundSample {
        glm::vec3  pos;
        glm::uvec4 joints;
        glm::vec4  weights;
    };
    std::vector<GroundSample> m_groundSamples;
};

} // namespace pose

#endif // MESH_H
