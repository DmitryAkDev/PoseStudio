/**
 * @file armature.h
 * @brief The runtime skeleton of a posable figure: its bones, the per-joint pose (Euler rotation
 *        + translation), forward kinematics into skinning dual quaternions, the user's joint
 *        pins, and the full-body-IK integration (armatureik.cpp).
 *
 * An Armature is everything about a figure's pose that is NOT geometry: Model owns one next to
 * its meshes and GPU buffers, uploads the dual quaternions it computes, and forwards every posing
 * call to it. The split is load-bearing: the armature is pure std + GLM — no Vulkan, no Qt — so
 * the IK harness (tools/ikharness/) drives the REAL drag loop (solve → extract rotations → FK →
 * next solve) on a real figure's dumped skeleton, tick for tick as the viewport does, instead of
 * a mirror copy that drifts. Anything that changes how a pose is composed, clamped, or extracted
 * belongs here, never in Model. A static model (an OBJ) still carries an Armature: empty of
 * bones, it holds the model transform and one identity joint for the shared skinned pipeline.
 */

#ifndef ARMATURE_H
#define ARMATURE_H

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pose {

class IkRig;

/// One bone of an armature, as the importer (or a skeleton dump) describes it. Rest transforms
/// are TRANSLATION-ONLY (the figure format's convention: a bone's rest frame is axis-aligned with
/// model space; its orientation lives separately in `orientation` and cancels at rest).
struct ArmatureBone {
    std::string name;
    int         parent = -1;                 ///< Index into the bone list; -1 = the skeleton root.
    glm::vec3   localBindTranslation{0.0f};  ///< Rest position relative to the parent joint (model units).
    glm::vec3   orientation{0.0f};           ///< Rest orientation, Euler degrees (XYZ) — the frame pose rotations act in.
    std::string rotationOrder = "XYZ";       ///< Order the pose Euler angles compose in.
    // Per-axis pose-rotation limits (degrees): the joint's anatomical range of motion. Only axes
    // flagged in `rotationLimited` are enforced; the others rotate freely.
    glm::vec3   rotationMin{0.0f};
    glm::vec3   rotationMax{0.0f};
    glm::bvec3  rotationLimited{false, false, false};
};

/**
 * @class Armature
 * @brief Bones + pose + FK + pins + full-body IK for one figure. Vulkan-free, Qt-free.
 *
 * Pose model: each bone's `poseLocal` = localBind · orient · R(euler, rotationOrder) · orient⁻¹,
 * plus a pose translation in the parent frame (only ever non-zero where FBIK moved the root).
 * poseGlobal chains parent-first; the skin transform is poseGlobal · inverseBind, converted to a
 * unit dual quaternion for the shaders. The world transform of the whole figure (the model
 * matrix) also lives here — IK, picking, and grounding all need it alongside the pose.
 */
class Armature {
public:
    Armature();
    ~Armature();
    Armature(Armature&&) noexcept;
    Armature& operator=(Armature&&) noexcept;
    Armature(const Armature&) = delete;
    Armature& operator=(const Armature&) = delete;

    /// Builds the runtime skeleton from @p bones (parents must precede children — figure
    /// skeletons are listed in hierarchy order) at bind pose. An empty list makes a static
    /// armature: no bones, one identity joint.
    void build(const std::vector<ArmatureBone>& bones);

    /// Writes the skeleton (one bone per line — see loadDump for the format) so the IK harness
    /// can run its drag-loop tests against a REAL figure's rig. Returns false if @p path can't
    /// be written. Debug hook (POSESTUDIO_DUMP_SKELETON=<path> in Model's constructor).
    bool dump(const std::string& path) const;

    /// Reads a skeleton written by dump(): `name parent order tx ty tz ox oy oz minx miny minz
    /// maxx maxy maxz lx ly lz` per line (local bind translation, orientation Euler degrees, the
    /// per-axis limits and their enable flags). Returns false (leaving @p out empty) on a read or
    /// parse failure.
    static bool loadDump(const std::string& path, std::vector<ArmatureBone>& out);

    // --- Skeleton ---
    bool               hasSkeleton() const { return !m_bones.empty(); }
    std::size_t        boneCount() const { return m_bones.size(); }
    const std::string& boneName(std::size_t i) const { return m_boneNames[i]; }
    int                boneParent(std::size_t i) const { return m_bones[i].parent; }
    /// Index of the bone named @p name, or -1.
    int boneIndex(const std::string& name) const {
        const auto it = m_boneIndex.find(name);
        return it == m_boneIndex.end() ? -1 : it->second;
    }
    /// The number of skinning joints the shaders see: the bone count, or 1 (identity) for a
    /// static armature.
    std::uint32_t jointCount() const {
        return static_cast<std::uint32_t>(m_bones.empty() ? 1 : m_bones.size());
    }

    // --- World transform (the model matrix) ---
    const glm::mat4& transform() const { return m_transform; }
    /// Replaces the model matrix and refreshes the transform-dependent bone world positions.
    void setTransform(const glm::mat4& transform);
    /// Translates the figure by @p dy along world Y (the animated ground drop applies its
    /// per-frame fall increments through this) and refreshes the bone world positions.
    void translateY(float dy);

    // --- Pose state ---
    /// Current world-space position of each joint (updated whenever the pose changes).
    const glm::vec3& boneWorldPosition(std::size_t i) const { return m_boneWorldPos[i]; }
    /// The accumulated pose rotation of bone @p i (Euler degrees, its rotation order).
    const glm::vec3& boneEuler(std::size_t i) const { return m_boneEuler[i]; }
    /// The pose translation of bone @p i (parent frame; non-zero only where FBIK moved the root).
    const glm::vec3& boneTranslation(std::size_t i) const { return m_boneTranslation[i]; }
    /// Bone @p i's posed model-space transform (poseGlobal) as of the last pose update.
    const glm::mat4& poseGlobal(std::size_t i) const { return m_poseGlobal[i]; }
    /// Bone @p i's inverse bind transform (the skin transform is poseGlobal · inverseBind).
    const glm::mat4& inverseBind(std::size_t i) const { return m_bones[i].inverseBind; }
    /// The skinning DUAL QUATERNIONS — per joint two vec4s: real = rotation as (x,y,z,w), dual =
    /// 0.5·(0,t)·real carrying the translation — recomputed by every pose update. The shaders
    /// blend THESE, not matrices: the figure format authors its weights (and every pose
    /// corrective) against dual-quaternion skinning, and linear matrix blending collapses deep
    /// bends (a 155° knee folded into a shapeless blob that the JCMs made worse).
    const std::vector<glm::vec4>& skinDualQuats() const { return m_skinDualQuats; }
    /// Bumped by every pose update; the GPU layer uploads per frame in flight when it changes.
    std::uint64_t skinVersion() const { return m_skinVersion; }

    /// Recomputes every bone's poseGlobal from its poseLocal, the world positions, and the skin
    /// dual quaternions. Every pose path ends here; callers that edit several bones' Euler
    /// through the primitives below call it once afterwards.
    void computeSkinMatrices();

    // --- Selection (the posing UI's current joint) ---
    int  selectedBone() const { return m_selectedBone; }
    void setSelectedBone(int index) { m_selectedBone = index; }
    /// Selects the bone named @p name (diagnostics / the IK benchmark); its index, or -1.
    int selectBoneByName(const std::string& name) {
        const int index = boneIndex(name);
        if (index >= 0) {
            m_selectedBone = index;
        }
        return index;
    }
    /// The selected joint's highlight twin (see m_highlightTwin), or -1 without a selection or twin.
    int selectedHighlightTwin() const {
        return (m_selectedBone >= 0 && m_selectedBone < static_cast<int>(m_highlightTwin.size()))
                   ? m_highlightTwin[static_cast<std::size_t>(m_selectedBone)]
                   : -1;
    }

    // --- Forward-kinematic posing ---
    /// Poses the joint @p boneName by an Euler rotation (degrees) applied in its oriented frame
    /// (clamped to its anatomical limits), then recomputes the skin data. Returns false (no-op)
    /// if there is no such bone. @p eulerDegrees of 0 restores the bone's rest pose.
    bool setBoneRotation(const std::string& boneName, const glm::vec3& eulerDegrees);
    /// Adds @p deltaEulerDegrees to the selected bone's accumulated rotation and re-poses it.
    void nudgeSelectedBone(const glm::vec3& deltaEulerDegrees);

    /// Captures the current pose as (bone name, Euler degrees) for each non-rest joint, plus
    /// "@trans:<bone>" rows for pose translations and "@pin:<bone>" rows for user pins (see
    /// the pin notes below) — the snapshot the undo stack and the .pose file carry.
    std::vector<std::pair<std::string, glm::vec3>> capturePose() const;
    /// Resets to bind pose, then applies @p pose (bone name -> Euler degrees, @trans:/@pin:
    /// rows honoured, unknown names ignored) and recomputes the skin data.
    void applyPose(const std::vector<std::pair<std::string, glm::vec3>>& pose);

    // --- Pose utilities: reset and mirror (the Edit menu / joint context menu). Pins are left
    // untouched by all of them — they are constraints, not shape (Unpin All exists for that).
    /// The bone on the OTHER side of the body: names pair by prefix — `l`/`r` followed by an
    /// uppercase letter or underscore (`lShin`/`rShin`, `l_thigh`/`r_thigh`), or `Left`/`Right`
    /// — and a centre bone (or an unpaired name) maps to itself.
    int mirrorBone(std::size_t index) const { return m_mirrorBone[index]; }
    /// Returns bone @p index to rest (rotation and pose translation zero); with @p subtree,
    /// every descendant too. False without such a bone.
    bool resetBone(int index, bool subtree);
    /// Returns every bone to the bind pose (rotations + translations).
    void resetPose();
    /// Mirrors the WHOLE pose across the sagittal plane: left and right swap, centre bones flip.
    /// Per Euler channel the mirror is (x, -y, -z) and the pose translation negates x — exact
    /// for a reflection because the figure's left/right orientation frames are themselves
    /// mirrored ((a, b, c) <-> (a, -b, -c)), so the reflected world rotation lands in the other
    /// side's frame with exactly those channel values (verified geometrically in the harness).
    void mirrorPose();
    /// Copies bone @p index's pose AND its subtree's to the opposite side, mirrored (the
    /// matching bones there take the mirrored values; centre bones inside the subtree flip in
    /// place, so mirroring from the chest mirrors both arms and the head). False without such a
    /// bone.
    bool mirrorSubtreeToOpposite(int index);

    // --- Full-body IK (scene/ik/): drag a joint, the whole body follows anatomically ---
    /// Begins an FBIK drag of the SELECTED joint: detects which joints are planted on the ground,
    /// pins them (feet for a standing figure — never the hip), and builds the balance support
    /// polygon. Builds the IK rig from the skeleton on first use. Returns false without a
    /// skeleton or selection.
    bool beginIkDrag();
    /// One FBIK drag update: solves the body so the selected joint reaches toward @p targetWorld
    /// (constrained multi-chain FABRIK + CoM auto-balance), converts the solved joint positions
    /// back to per-channel Euler rotations (clamped to the figure's anatomical limits — the
    /// authoritative constraint pass) plus a root pose-translation, and re-skins. Returns true if
    /// the pose changed. Correctives are the Model's business (deferred to the drag's end).
    bool dragIkTo(const glm::vec3& targetWorld);
    /// One ANIMATED release-settle step: with the drag goal gone, relaxes the body one capped
    /// round toward its pins (an unreachable goal can hold a foot hovering off its plant; on
    /// release the feet should visibly land, not pop). Call at the drag tick rate after the mouse
    /// is released, until it returns false (pins planted / no further progress — each round is
    /// reverted if it fails to improve the worst pin error, so the settle can never regress).
    bool settleIkTick();
    /// Ends the FBIK drag (the solved pose stays).
    void endIkDrag();
    /// True between beginIkDrag() and endIkDrag().
    bool ikDragActive() const;
    /// The rig (null until the first IK drag built it) — diagnostics and the harness.
    const IkRig* ikRig() const { return m_ikRig.get(); }

    // --- User joint pins (see IkRig::beginDrag's userPins): a pinned joint is held exactly where
    // it is through every later IK drag of OTHER joints, until unpinned. Dragging a pinned joint
    // itself moves the pin. Pins are part of the POSE SNAPSHOT (capturePose/applyPose carry them
    // as "@pin:<bone>" rows), so a pin toggle is undoable, undoing a drag restores the pins of
    // that moment, and a .pose file reproduces its pins on load (a file without pin rows — an
    // older one — loads with none).
    /// Toggles the pin on the selected joint; returns the new pinned state (false with no selection).
    bool togglePinSelectedBone();
    bool isBonePinned(std::size_t index) const {
        return index < m_bonePinned.size() && m_bonePinned[index] != 0;
    }
    bool selectedBonePinned() const {
        return m_selectedBone >= 0 && isBonePinned(static_cast<std::size_t>(m_selectedBone));
    }
    bool hasPinnedBones() const;
    void unpinAllBones();
    /// The rig's CONTACT pins (ground-detected, not user pins) while an IK drag is active — for
    /// the overlay's "which feet are planted" markers. Empty outside a drag.
    std::vector<int> activeContactPins() const;

private:
    // One runtime skeleton joint. localBind is its rest transform relative to its parent;
    // poseLocal folds in the current animated rotation (== localBind at bind pose).
    // `orient`/`invOrient` frame that rotation in the joint's true orientation so bends are
    // anatomically correct.
    struct Bone {
        int         parent = -1;
        glm::mat4   inverseBind{1.0f};
        glm::mat4   localBind{1.0f};
        glm::mat4   poseLocal{1.0f};
        glm::mat4   orient{1.0f};
        glm::mat4   invOrient{1.0f};
        glm::vec3   orientationDeg{0.0f}; ///< The authored orientation (dump round-trip).
        std::string rotationOrder = "XYZ";
        // Per-axis pose-rotation limits (degrees) enforced by clampBoneEuler(); only axes flagged
        // in rotLimited are constrained (the figure's anatomical range of motion).
        glm::vec3   rotMin{0.0f};
        glm::vec3   rotMax{0.0f};
        glm::bvec3  rotLimited{false, false, false};
    };

    /// Re-poses bone @p index from its accumulated Euler (m_boneEuler) in its oriented frame, then
    /// recomputes the skin data. Shared by setBoneRotation() and nudgeSelectedBone().
    void applyBoneEuler(int index);

    /// Recomposes bone @p index's poseLocal from its current Euler + pose translation:
    /// localBind · orient · R(euler, order) · orient⁻¹, translation added in the parent frame.
    /// The single composition point every pose path (FK, pose load, IK extraction) shares.
    void recomposePoseLocal(std::size_t index);

    /// Appends bone @p index and every descendant to @p out (anatomical hierarchy).
    void collectSubtree(int index, std::vector<int>& out) const;
    /// Re-poses every bone from its (limit-clamped) Euler + translation, then the skin data once.
    void reposeAll();

    /// Clamps m_boneEuler[index] in place to the bone's per-axis rotation limits (a no-op on axes
    /// the figure leaves unconstrained). The single enforcement point every posing path funnels
    /// through.
    void clampBoneEuler(int index);

    /// Converts an FBIK solve's joint positions back into the engine's pose: walks the anatomical
    /// hierarchy top-down, absorbs a moved skeleton root into its pose translation, best-fits each
    /// active bone's world rotation to its solved child directions (aim at the longest child +
    /// average twist about it), decomposes into the bone's Euler channels, clamps to the authored
    /// limits, and updates poseGlobal incrementally. Inactive subtrees keep their local pose and
    /// ride along.
    void applyIkSolution(const std::vector<glm::vec3>& solved, const std::vector<char>& active,
                         bool rotationPrior);

    /// EXACT enforcement of the joint pins, run after every governed pose update of an IK tick
    /// (drag and release settle alike). The solver holds a pin in POSITION space, but the
    /// applied pose is what the user sees, and the extraction (aim fit, per-joint angular caps,
    /// limit clamps, the rotational prior) plus the governor's under-relaxation — a blend in
    /// JOINT space, which does not preserve an end effector's position — land a pinned joint
    /// millimetres off every tick: visible micro-motion on a joint declared immovable, and
    /// planted feet that slide by millimetres under a hand drag. This refines each pin's own
    /// limb chain in joint space — damped least squares on the chain's unlocked Euler channels
    /// against a numeric Jacobian, limits respected, iterated to 0.1mm — and re-imposes the
    /// pin's drag-start world orientation exactly (the flat-sole hold in applyIkSolution is
    /// capped and then relaxed by the governor, so it too left a residual). USER pins are held
    /// unconditionally, every tick; the drag's CONTACT pins and the released joint the settle
    /// holds only when @p settling — the release settle is where the pose comes to rest — and
    /// only as residual cleanup, band-gated and per-tick capped so the landing stays animated
    /// (see kContactRefineBand; holding contacts exact DURING the drag masked the FK slip that
    /// is the balance stepper's strain signal, and its pelvis stage fought the solver root).
    /// The correction is local to each pin's limb and never touches the trunk or another limb.
    /// With @p dragTarget (a drag tick: the solve target, model space) the GRABBED joint itself
    /// is refined onto it the same way — the cursor as a pin: the limb closes whatever part of
    /// the gap it can reach from the current trunk NOW, deterministically, so the hand tracks
    /// the cursor without lag while the body's redundant motion stays on the damped dynamics
    /// underneath (and pixel noise passes through 1:1 instead of being amplified ~14x by the
    /// whole-body solve — the fit is locally linear). The drag correction is a FINISHER: full
    /// within kDragRefineFull of the target and fading to nothing by kDragRefineFade, so large
    /// motions stay with the solve's own posture choice (a minimal-norm fit closing a whole
    /// 15cm foot lift swung the straight leg back at the hip instead of flexing the knee, and
    /// stalled at half the lift; a nullspace posture bias and column-scaled least squares were
    /// both measured and rejected). Re-skins once when it changed anything.
    void refinePins(bool settling, const glm::vec3* dragTarget = nullptr);

    std::vector<Bone>                    m_bones;
    std::unordered_map<std::string, int> m_boneIndex; // bone name -> index into m_bones
    std::vector<std::string>             m_boneNames;  // parallel to m_bones (for the posing UI)
    glm::mat4                            m_transform{1.0f};
    std::vector<glm::vec3>               m_boneWorldPos; // current world position per bone (overlay/pick)
    // Per bone: its highlight TWIN (-1 if none). Figures split each limb segment into a bend
    // bone and a TWIST child whose two swing axes are locked (range under 2°) — the mid-limb
    // bone that spreads axial twist across the skin. The selection highlight covers the pair
    // (bend -> its twist child, twist -> its bend parent), so grabbing the upper-arm joint lights
    // the whole upper arm rather than the half the bend bone's own weights cover.
    std::vector<int>                     m_highlightTwin;
    std::vector<std::vector<int>>        m_children;   // anatomical children per bone (subtree walks)
    std::vector<int>                     m_mirrorBone; // the other side's bone per bone (self for centre)
    std::vector<glm::mat4>               m_poseGlobal;  // scratch for computeSkinMatrices (it runs per drag-move; no per-call allocation)
    std::vector<glm::vec3>               m_boneEuler;  // accumulated pose rotation per bone (degrees)
    // Pose translation per bone (parent-frame offset added to poseLocal). Rotations alone can't
    // move the skeleton root, so FBIK with pinned feet writes the hip's solved displacement here
    // (a crouch drops the pelvis). Round-trips through capture/applyPose as "@trans:<bone>" rows.
    std::vector<glm::vec3>               m_boneTranslation;
    int                                  m_selectedBone = -1;
    // Skinning data (see skinDualQuats()), recomputed whenever the pose changes; the GPU layer
    // uploads it lazily per frame in flight when m_skinVersion moves.
    std::vector<glm::vec4>               m_skinDualQuats;
    std::uint64_t                        m_skinVersion = 0;

    // Full-body IK: the rig (graph + constraints + masses, built lazily on the first IK drag),
    // plus the anatomical children lists and model-space bind joint positions the extraction walk
    // reads (built alongside — bind positions provide the rest aim offsets across the rigid
    // twist-bone links extraction looks through).
    std::unique_ptr<IkRig>        m_ikRig;
    std::vector<std::vector<int>> m_ikChildren;
    std::vector<glm::vec3>        m_ikBindPos;
    // WEIGHT-BEARING feet: each planted (pinned) foot bone and its drag-start model-space
    // rotation. The extraction preserves that world orientation while the pin holds, so the
    // sole stays flat on the floor as the body moves above it (a solver-side sole pin fought
    // the ankle and curled toes; orientation preservation at extraction fights nothing).
    std::vector<int>              m_ikFlatNodes;
    std::vector<glm::mat3>        m_ikFlatRot;
    // User joint pins (per bone, see togglePinSelectedBone): persistent until unpinned.
    std::vector<char>             m_bonePinned;
    // Bones exempt from the drag-tick rotational prior for the current drag: the LIMB chain of
    // each user pin (pin up to where its limb joins the axial skeleton — the rig's mass-based
    // junction, IkRig::userPinLimbNodes). Those joints are fully determined
    // by the pin's restoration every tick, and the prior — decaying toward the drag-START
    // pose — could only fight the pin there: a hand pinned through a crouch ratcheted 8cm off
    // its pin during the still hold as the prior pulled the arm back toward its standing pose.
    std::vector<char>             m_ikRotPriorExempt;
    // Drag-start Euler pose: the ROTATIONAL prior. The extraction's aim fit determines only
    // part of each joint's rotation (a single aim child leaves twist unwitnessed), and the
    // undetermined components RATCHET across ticks — the spine's forward-biased limits turned
    // that random walk into a visible bow whenever a hand was pulled up. During drag ticks the
    // fitted angles decay gently toward these start values: determined components are re-imposed
    // by the next fit anyway, so only the drift is cleaned.
    std::vector<glm::vec3>        m_ikStartEuler;
    // Previous drag tick's APPLIED per-bone deltas (Euler degrees / pose translation), for the
    // governor's reversal-gated micro-motion damping: a delta OPPOSING the previous tick's is
    // the loop's own tick-scale oscillation and is attenuated; sustained motion passes at full
    // rate (magnitude-gated damping shaved a raising arm's climb enough that the rotational
    // prior's take-back overcame it — the hand visibly sank mid-raise).
    std::vector<glm::vec3>        m_ikPrevEulerDelta;
    std::vector<glm::vec3>        m_ikPrevTransDelta;
    // Last tick's APPLIED worst-joint world speed — the velocity state of the governor's
    // motion shaping (see kIkAccel/kIkDecel in armatureik.cpp): the per-tick movement allowance
    // may grow at most kIkAccel over this (ease-in) and is bounded by the braking curve toward
    // the goals (ease-out), so gestures accelerate and decelerate like real limbs instead of
    // snapping to a constant governor rate. Zeroed when a tick applies nothing (frozen hold).
    float                         m_ikAppliedSpeed = 0.0f;
    // Last tick's APPLIED speed of the GRABBED joint itself: the motion shaping bounds the
    // effector's own step as well as the worst joint's (with the fold plane free to swing, the
    // elbow is often the worst joint, and the hand could then jump 15mm in one tick — exactly
    // the mechanical ramp the shaping exists to prevent).
    float                         m_ikAppliedEffSpeed = 0.0f;
    // Grab offset (model space) for a PROMOTED drag: the rig solves the limb's real end joint
    // (a finger grab drives the HAND — see IkRig::dragEffector), so the window's targets, which
    // track the grabbed joint, are shifted by (grabbed - solved) captured at drag start.
    glm::vec3                     m_ikGrabOffset{0.0f};
    // The solved effector's world rotation at drag start: the grab offset is carried through
    // the effector's rotation since (offset_now = R_now * R_start^-1 * offset), so a finger
    // grab keeps tracking the FINGER when the hand twists — with the solver now free to twist
    // the forearm, a constant offset missed the fingertip by up to twice its length.
    glm::mat3                     m_ikGrabRotStart{1.0f};
    // Previous drag target (model space): the world-space governor's per-event pose budget is
    // PROPORTIONAL to how far the target actually moved — a still-but-noisy cursor earns only a
    // millimeter budget (kills trembling), a fast pull earns the full step.
    glm::vec3                     m_ikPrevTarget{0.0f};
    bool                          m_ikPrevTargetValid = false;
    // Settle-freeze state: while the target is still, worst-goal-error minima are collected in
    // 6-tick windows; when a window fails to improve on the previous one, solving FREEZES until
    // the target moves again. Window minima are oscillation-robust (churn can't fake envelope
    // improvement), while genuine slow catch-up (feet re-planting) keeps improving and stays
    // live until done — the two things a per-tick movement test cannot tell apart.
    int                           m_ikStillTicks = 0;
    float                         m_ikErrCurMin = 1e30f;
    float                         m_ikErrPrevMin = 1e30f;
    bool                          m_ikFrozen = false;
    int                           m_ikSettleTicks = 0; ///< Animated release-settle tick budget.
    // The worst pin error at the release settle's first tick — the STRAIN the drag left in the
    // planted feet. A LIMB effector's pose-hold bound (the released hand/foot stays within 2cm
    // of where it was let go) scales up with it (3x, at most 10cm): a beyond-reach pull leaves a
    // foot centimetres in the air, and planting it costs the hand a few centimetres — a figure
    // standing on air is the worse artifact. A TRUNK effector (IkRig::effectorIsTrunk) always
    // gets the 10cm bound: planting the feet after a chest/hip drag necessarily moves the trunk.
    float                         m_ikSettleStrain = 0.0f;
    // Stillness is CUMULATIVE drift from this anchor, not per-tick deltas: a slowly creeping
    // target (sub-mm per event) must keep the solve live — it accumulates past the threshold and
    // re-anchors — while zero-mean cursor noise stays inside the ball and allows the freeze.
    glm::vec3                     m_ikStillAnchor{0.0f};
};

} // namespace pose

#endif // ARMATURE_H
