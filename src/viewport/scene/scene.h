/**
 * @file scene.h
 * @brief Owns the lit mesh pipeline, the per-frame camera/lighting UBO + descriptors, and the
 *        list of imported models. The mesh analogue of Grid.
 *
 * Kept self-contained so VulkanRenderer doesn't accumulate per-feature pipeline/descriptor code:
 * the renderer just constructs a Scene, forwards imports to addModel(), and calls record() inside
 * the render pass (before the transparent grid). Qt-free (Vulkan + std + GLM).
 */

#ifndef SCENE_H
#define SCENE_H

#include "environment.h"
#include "lightingsettings.h"
#include "shademode.h"
#include "vulkanbuffer.h"

#include <glm/glm.hpp>
#include <vulkan/vulkan.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace pose {

class VulkanContext;
class VulkanPipeline;
class VulkanTexture;
class Camera;
class Model;
class IblMaps;
class OutlineMask;
class ShadowMap;
struct ModelData;
struct Ray;

/**
 * @class Scene
 * @brief The collection of imported meshes plus the machinery to draw them lit.
 */
class Scene {
public:
    /// @param outlineMaskPass The selection-outline mask's render pass (OutlineMask; the renderer
    ///                        owns the screen-sized target) — the silhouette pipeline builds
    ///                        against it, reusing shadow.vert with @p outlineMaskFragSpirv.
    Scene(VulkanContext& context, VkRenderPass renderPass, VkRenderPass outlineMaskPass,
          const std::vector<char>& vertSpirv, const std::vector<char>& fragSpirv,
          const std::vector<char>& skeletonVertSpirv, const std::vector<char>& skeletonFragSpirv,
          const std::vector<char>& shadowVertSpirv, const std::vector<char>& shadowFragSpirv,
          const std::vector<char>& backgroundVertSpirv,
          const std::vector<char>& backgroundFragSpirv,
          const std::vector<char>& outlineMaskFragSpirv);
    ~Scene();

    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;

    /// Uploads CPU geometry to the GPU and adds it to the scene, and SELECTS it (a freshly
    /// imported object is the one the user is about to work with, and the outline shows the
    /// import landed). Call from the GUI thread between frames (the upload blocks briefly on its
    /// own one-time submit; it touches no in-flight state).
    void addModel(const ModelData& data);

    // --- Object selection (the outlined model) ---
    /// The selected model's index, or -1 when nothing is selected. The selection is what the
    /// viewport outlines; clicking a figure's joint selects that figure (setActiveFigure), a
    /// plain click on a model's box selects it, a click on empty space clears it.
    int selectedModelIndex() const;
    /// Selects model @p index (-1 or out of range = clear). Selecting a figure also makes it the
    /// active (posing-target) figure; the previously selected figure's joint selection is
    /// cleared, so nothing posing-related lingers on an unselected model.
    void setSelectedModel(int index);

    /// Number of models currently in the scene.
    std::size_t modelCount() const { return m_models.size(); }

    /// Uploads a CPU-baked environment (SH irradiance + prefiltered specular; see bakeEnvironment) and
    /// points the IBL descriptor set at it. Waits for the GPU to idle first, since it swaps the maps the
    /// mesh pipeline samples. The heavy CPU bake is done by the caller (off the render thread for
    /// interactive HDRI switches); this does only the fast GPU work.
    void applyBakedEnvironment(const BakedEnvironment& baked);

    /// Live lighting/exposure controls (the Environment panel drives these). All shader-side, so this
    /// is just a cached value read into the next frame's UBO — no re-bake.
    void                   setLightingSettings(const LightingSettings& settings) { m_lighting = settings; }
    const LightingSettings& lightingSettings() const { return m_lighting; }

    /// Returns the index of the nearest model whose bounding box @p ray hits, or -1 if none.
    int pickModel(const Ray& ray) const;

    /// The world AABB to frame for "frame selected": the selected model's, else the union of
    /// every model's (a posed figure's bounds follow its joints — see Model::worldBounds).
    /// Returns false with nothing to frame.
    bool framingBounds(glm::vec3& outMin, glm::vec3& outMax) const;

    /// Removes the model at @p index (no-op if out of range). The caller MUST have ensured the GPU
    /// is idle first (the model's buffers/descriptors may be referenced by in-flight frames) —
    /// VulkanRenderer::deleteModel() does this.
    void removeModel(std::size_t index);

    /// Records the key light's depth-only shadow pass (its own render pass on the shadow map).
    /// Call BEFORE the main render pass each frame: it also (re)fits the light's ortho frustum
    /// around the scene + its floor projections, which record() then feeds to the shaders via the
    /// UBO. Skipped (leaving the map fully lit) while the scene has no casters.
    void recordShadowPass(VkCommandBuffer cmd, uint32_t frameIndex);

    /// Records the selection-outline mask pass — the selected model's silhouette, skinned and
    /// camera-projected, into @p mask (its own render pass, begun and ended here). Call before
    /// the main render pass, like recordShadowPass. Returns false, recording nothing, when no
    /// model is selected (the composite then draws no outline).
    bool recordOutlinePass(VkCommandBuffer cmd, const Camera& camera, uint32_t frameIndex,
                           const OutlineMask& mask);

    /// Updates this frame's camera UBO, binds the pipeline + camera set, and records every model.
    /// The caller has begun the render pass and set the dynamic viewport/scissor.
    void record(VkCommandBuffer cmd, const Camera& camera, uint32_t frameIndex);

    // The grid's ground shadow samples the scene's shadow map through the scene-wide set 3 (the
    // IBL set — binding 2 is the shadow map). Grid's pipeline is built against this layout and
    // binds this set at record time, with the matching light matrix from lightViewProj().
    VkDescriptorSetLayout iblSetLayout() const { return m_iblSetLayout; }
    VkDescriptorSet       iblSet() const { return m_iblSet; }
    const glm::mat4&      lightViewProj() const { return m_lightViewProj; }

    // --- Posing UI ---
    /// Whether the scene holds a posable figure (a model with a skeleton).
    bool hasPosableFigure() const;
    /// The ACTIVE figure — the one every posing call (selection, IK, FK, pins, pose
    /// snapshot, ground) addresses: the figure whose joint was last clicked (selectBoneAt
    /// searches every figure's joints), else the first figure in the scene. -1 without one.
    int  activeFigureIndex() const;
    /// Makes model @p index the active figure (no-op unless it is a figure) — and the SELECTED
    /// model, so the outline follows the posing target. Undo/redo restores the figure a pose
    /// snapshot was taken from through this before applying it.
    void setActiveFigure(int index);
    /// Show/hide the skeleton overlay (drawn over the figure so the joints — always grabbable,
    /// drawn or not — can be seen).
    void setShowSkeleton(bool on) { m_showSkeleton = on; }
    bool showSkeleton() const { return m_showSkeleton; }

    // --- Shading ---
    /// Selects the viewport shade mode: an index into the picker's table (scene/shademode.h),
    /// whose row says which mesh.frag mode shades the surface (written into the per-frame camera
    /// UBO — no pipeline swap for that) and whether record() draws the surface at all, a
    /// hidden-line depth fill, and/or a wireframe overlay.
    void setShadeMode(int mode) { m_shadeMode = mode; }
    int  shadeMode() const { return m_shadeMode; }
    /// The current mode's table row, and whether it is in the PBR family (PBR Shaded and its
    /// specular-only view) — the modes that render linear HDR through the image-based path, so
    /// the HDR post chain (SSS, bloom, ACES) and the HDRI backdrop run only for them.
    const ShadeMode& shadeModeSpec() const { return shadeModeAt(m_shadeMode); }
    bool             isPbr() const { return fragModeIsHdr(shadeModeSpec().fragMode); }
    /// Selects the figure joint nearest the pixel (@p px, @p py) in a @p vpW × @p vpH viewport (only
    /// if within a small radius). Returns the selected bone index, or -1 if none is selected.
    int selectBoneAt(float px, float py, float vpW, float vpH, const Camera& camera);
    /// True if a joint is currently selected on the figure.
    bool hasSelectedBone() const;
    /// Selects the posable figure's bone named @p name (diagnostics / the IK benchmark).
    int selectBoneByName(const std::string& name);
    /// Rotates the selected joint by @p deltaEulerDegrees (accumulated), re-posing the figure.
    void nudgeSelectedBone(const glm::vec3& deltaEulerDegrees);
    /// Settles the figure after an interactive pose edit: applies pose correctives, which are
    /// deferred during a drag (they re-upload geometry) and applied here on release.
    void finalizePose();

    // --- Full-body IK (Ctrl+drag a joint; see scene/ik/) ---
    /// Begins an FBIK drag of the figure's selected joint: ground contacts are detected and become
    /// the anchor/pins, and the balance support polygon is captured. Returns false without a
    /// figure or selection.
    bool beginBoneIkDrag();
    /// One FBIK drag update: solves the whole body so the selected joint reaches toward
    /// @p targetWorld (feet stay planted, CoM auto-balanced), then re-poses the figure. Returns
    /// true if the pose actually changed (false: deadband / settle-freeze — no redraw needed).
    bool dragBoneIkTo(const glm::vec3& targetWorld);
    /// One animated release-settle step (see Model::settleIkTick): call at the drag tick rate
    /// after release until it returns false, then endBoneIkDrag().
    bool settleBoneIkTick();
    /// Ends the FBIK drag (the pose stays; call finalizePose() to settle correctives, as with any
    /// pose drag).
    void endBoneIkDrag();
    /// World-space position of the figure's selected joint (the IK drag plane's anchor point).
    /// Returns false when no joint is selected.
    bool selectedBoneWorldPosition(glm::vec3& out) const;
    // --- User joint pins (forwarded to the figure; see Model::togglePinSelectedBone) ---
    bool togglePinSelectedBone();
    bool selectedBonePinned() const;
    bool hasPinnedBones() const;
    void unpinAllBones();
    /// Drops the posable figure onto the ground plane: translates it so the CURRENT pose's lowest
    /// point rests at y = 0 (the viewport's "move to ground" button). Returns true if it moved.
    bool groundFigure();
    /// The posable figure's current lowest world height (see Model::groundGap); false without one.
    bool figureGroundGap(float& lowestY) const;
    /// Translates the posable figure along world Y (the animated ground drop's per-frame step).
    void translateFigureY(float dy);

    // --- Pose snapshot (for undo/redo) ---
    std::vector<std::pair<std::string, glm::vec3>> capturePose() const;
    void applyPose(const std::vector<std::pair<std::string, glm::vec3>>& pose);

    /// Saves / loads the figure's pose (per-joint rotations) to/from a plain-text file. Returns false
    /// if there's no figure or the file can't be opened.
    bool savePose(const std::string& path) const;
    bool loadPose(const std::string& path);

private:
    void createDescriptorResources();
    Model*    figureModel() const; // first model with a skeleton, or nullptr
    glm::vec3 keyLightDir() const; // normalized world dir TO the key light (azimuth/elevation dials)

    VulkanContext&                  m_context;
    std::unique_ptr<VulkanPipeline> m_pipeline;            // opaque pass (depth write on)
    std::unique_ptr<VulkanPipeline> m_transparentPipeline; // alpha-blended pass (depth write off)
    std::unique_ptr<VulkanPipeline> m_skeletonPipeline;    // line overlay for the posing skeleton
    std::unique_ptr<VulkanPipeline> m_backgroundPipeline;  // HDRI backdrop (PBR mode; drawn first)
    std::unique_ptr<VulkanPipeline> m_outlinePipeline;     // selected model -> the outline mask (its own pass)
    std::unique_ptr<VulkanPipeline> m_wirePipeline;        // wireframe modes: triangle EDGES (null: device can't)
    std::unique_ptr<VulkanPipeline> m_hiddenLinePipeline;  // hidden-line modes: depth-only surface fill
    // Host-mapped line vertices (pos+color), one buffer per frame-in-flight: record() rewrites the
    // overlay every frame, so a single shared buffer would be CPU-written while the previous
    // frame's GPU read of it is still in flight.
    std::vector<VulkanBuffer>       m_skeletonVertexBuffers;
    bool                            m_showSkeleton = false;
    int                             m_shadeMode = kDefaultShadeMode; // picker-table index (shademode.h)

    // Lighting environment: the baked diffuse-irradiance SH (feeds the per-frame camera UBO so the PBR
    // mode is image-based-lit) and the environment-independent split-sum BRDF LUT (integrated once,
    // reused on every re-bake). The prefiltered specular cubemap lives in m_iblMaps. A CPU-baked
    // BakedEnvironment (SH + specular) is applied via applyBakedEnvironment(); no source image is kept.
    EnvironmentSH    m_environmentSH;
    BrdfLut          m_brdfLut;
    LightingSettings m_lighting; // live exposure/diffuse/specular/fill/key/rotation/tonemap dials (Environment panel)

    // Per-frame camera/lighting UBO (set 0, binding 0): one buffer + one set per frame-in-flight.
    VkDescriptorSetLayout        m_setLayout = VK_NULL_HANDLE;
    VkDescriptorPool             m_descriptorPool = VK_NULL_HANDLE;
    std::vector<VulkanBuffer>    m_cameraBuffers;
    std::vector<VkDescriptorSet> m_cameraSets;

    // Per-material textures (set 1), six samplers: binding 0 = diffuse (sRGB), 1 = detail
    // normal/bump (linear), 2 = roughness map (linear), 3 = spec-mask map (linear),
    // 4 = translucency map (sRGB), 5 = micro-detail (pore) normal (linear). The layout is shared;
    // each Model owns the pool/sets for its meshes. Meshes without a given map point at a shared
    // 1x1 fallback (white / flat normal) so the shader always samples and one pipeline serves
    // textured and untextured meshes alike.
    VkDescriptorSetLayout          m_materialSetLayout = VK_NULL_HANDLE;
    std::unique_ptr<VulkanTexture> m_fallbackTexture; // 1x1 opaque white
    std::unique_ptr<VulkanTexture> m_fallbackNormal;  // 1x1 flat normal (128,128,255), linear

    // Per-model skinning joint matrices (set 2, binding 0): a storage buffer of skin matrices read
    // in the vertex stage. The layout is shared; each Model owns its own buffer + set (see mesh.h).
    VkDescriptorSetLayout m_jointSetLayout = VK_NULL_HANDLE;

    // Image-based lighting (set 3): the prefiltered specular cubemap + BRDF LUT (owned by m_iblMaps),
    // bound scene-wide. The diffuse half is the SH in m_environmentSH (fed via the camera UBO). One
    // static set, its images (re)filled by applyBakedEnvironment() when the environment changes.
    VkDescriptorSetLayout    m_iblSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool         m_iblPool = VK_NULL_HANDLE;
    VkDescriptorSet          m_iblSet = VK_NULL_HANDLE;
    std::unique_ptr<IblMaps> m_iblMaps;

    // Key-light shadows: the depth-only map + its skinned depth pipeline, re-rendered each frame
    // before the main pass (recordShadowPass). The map is bound scene-wide as set 3 binding 2; the
    // fitted light matrix rides in the camera UBO (and to the grid via lightViewProj()).
    std::unique_ptr<ShadowMap>      m_shadowMap;
    std::unique_ptr<VulkanPipeline> m_shadowPipeline;
    glm::mat4                       m_lightViewProj{1.0f};

    std::vector<std::unique_ptr<Model>> m_models;
    int m_activeFigure = -1; ///< See activeFigureIndex(): the posing target among the figures.
    int m_selectedModel = -1; ///< See selectedModelIndex(): the outlined model (-1 = none).
};

} // namespace pose

#endif // SCENE_H
