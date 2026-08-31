/**
 * @file meshik.cpp
 * @brief Model's full-body-IK integration: the engine boundary of the FBIK system (scene/ik/).
 *
 * Everything FBIK needs from the engine lives in this one file: building the per-figure IkRig
 * from the bind skeleton, detecting ground contacts, driving the per-tick drag solve with its
 * damped-motion governors (under-relaxation, proportional world-space cap, per-joint angular
 * cap, settle-freeze), the animated pose-holding release settle, and the ROTATION EXTRACTION
 * that converts the solver's joint positions back into the engine's per-channel Euler pose (+
 * the root's pose translation). The solver itself is Model-agnostic (scene/ik/, pure positions);
 * this file is the only place the two meet, so solver work and engine work can evolve
 * independently. See CLAUDE.md's FBIK section for the design history — most numeric policies
 * here trace to a specific user-visible failure, documented at their definitions.
 */

#include "mesh.h"

#include "ikmath.h" // shortestArc / signedAngleAround / eulerFromMatrix (pose extraction)
#include "ikrig.h"  // full-body IK orchestrator

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstddef>
#include <cstdlib>
#include <vector>

namespace pose {

namespace {

// A joint whose world height is below this is a ground contact for FBIK anchoring/balance (feet
// and toes standing, knees kneeling; a figure held above the floor has none and anchors at the
// skeleton root). World units (a figure imports at real scale; standing ankles sit ~0.09).
constexpr float kIkContactHeight = 0.15f;

// FBIK angular rate limit: no joint's Euler pose may change by more than this per solve (per
// mouse event). The solver's positional trust region bounds SOLVED displacement, but extraction
// turns positions into rotations, and a small capped motion at the chest is a large swing at the
// end of a long lever (the head, the far hand) — capping at the rotation level is what actually
// bounds visible per-event pose change, and it preserves bone lengths by construction. Mouse
// events are dense (60+/s), so 12 deg/event still allows very fast posing.
constexpr float kIkMaxJointDeltaDeg = 12.0f;

// Shortest signed angular difference a-b in degrees, wrapped to (-180, 180].
float wrappedAngleDelta(float a, float b) {
    float d = a - b;
    while (d > 180.0f) d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return d;
}

// FBIK world-space governor: if ANY joint's world position moved farther than this in one solve,
// the whole pose delta is scaled down to fit. Per-joint angular caps alone still COMPOUND down a
// chain (four capped joints swing a hand far); this is the final authority on how fast the pose
// may visibly change per mouse event, so IK can never "jump". Catch-up continues across events
// (the viewport re-issues the drag target on a timer while the button is held).
constexpr float kIkMaxWorldStep = 0.10f;

// Rotational-prior decay per drag tick (see Model::m_ikStartEuler): how fast fit-undetermined
// rotation components ease back toward the drag-start pose. Strong enough to keep unwitnessed
// twist from ratcheting into the joints' permissive clamp side (the spine bow), weak enough
// that the per-tick lag it adds to determined components is invisible.
constexpr float kIkRotationPrior = 0.05f;

// FBIK under-relaxation: the fraction of each solve's pose delta actually applied per event.
// 0.5 exactly cancels period-2 oscillation (a solver alternating between two near-equal
// configurations tick to tick — visible as trembling); convergence still completes across the
// 60 Hz drag ticks.
constexpr float kIkRelaxation = 0.5f;

// Micro-motion damping (the governor's per-joint soft deadzone), gated on REVERSAL: a joint's
// applied delta is scaled by e / (e + kIkMicroDamp*) — but only when it OPPOSES the previous
// tick's applied delta. The solve -> extract -> FK loop carries a residual tick-to-tick limit
// cycle (near-equal configurations alternating) that the uniform under-relaxation attenuates
// but sustains — at 60 Hz the user sees idle joints SHIMMER while pulling; a reversal IS that
// oscillation, and the soft factor kills a sub-0.1-degree alternation below perception while a
// genuine course change just eases through one damped tick. Gating on magnitude alone was
// wrong: it also shaved a raising arm's steady climb, and the rotational prior's take-back
// then overcame the climb — the hand visibly SANK mid-raise while the cursor rose. Angular in
// degrees, translation in world units.
constexpr float kIkMicroDampDeg = 0.15f;
constexpr float kIkMicroDampTrans = 5e-4f;

// Motion shaping (velocity-limited governor): the per-tick movement allowance may grow at most
// kIkAccel (world units / tick^2) over the PREVIOUS tick's applied speed — a brisk flick ramps
// up over ~5 ticks (~80 ms, a real limb's spin-up) instead of jumping to full rate in one —
// and never exceeds the braking curve sqrt(2 * kIkDecel * goalError) + the tracking allowance,
// so catch-up after the cursor stops DECELERATES smoothly into the pose instead of running at
// a constant rate and stopping flat (the profile harness showed a one-tick 19mm/tick jump and
// a 14mm hard stop on fast gestures — the "mechanical" look). The tracking term (3x target
// motion) keeps the braking curve from throttling pursuit of a MOVING cursor: the lag behind a
// live drag is meant to be consumed at speed.
constexpr float kIkAccel = 0.010f;
constexpr float kIkDecel = 0.004f;
// Symmetric braking limit: the allowance may also SHRINK by at most this per tick, so a
// stopping cursor ramps the pose down instead of collapsing its budget in one tick (the
// adaptive target filter converges fast on a stop, and the tracking term vanishing with it
// measured as a 23mm one-tick speed drop — a visible hitch). Purely an allowance: a pose that
// reaches its goals stops through vanishing solver proposals regardless.
constexpr float kIkBrake = 0.006f;

} // namespace

bool Model::beginIkDrag() {
    if (m_selectedBone < 0 || m_selectedBone >= static_cast<int>(m_bones.size()) ||
        m_bones.empty()) {
        return false;
    }
    if (!m_ikRig) {
        // First IK drag on this figure: build the rig (graph, constraints, masses) from the bind
        // skeleton, plus the anatomical children lists the extraction walk reads.
        std::vector<IkRigBone> rigBones(m_bones.size());
        std::vector<glm::vec3> bindPos(m_bones.size(), glm::vec3(0.0f));
        for (std::size_t i = 0; i < m_bones.size(); ++i) {
            const GpuBone& bone = m_bones[i];
            const glm::vec3 parentBind =
                bone.parent >= 0 ? bindPos[static_cast<std::size_t>(bone.parent)] : glm::vec3(0.0f);
            bindPos[i] = parentBind + glm::vec3(bone.localBind[3]);
            IkRigBone& rigBone = rigBones[i];
            rigBone.name = m_boneNames[i];
            rigBone.parent = bone.parent;
            rigBone.bindPos = bindPos[i];
            rigBone.orientAxes = glm::mat3(bone.orient);
            rigBone.rotMinDeg = bone.rotMin;
            rigBone.rotMaxDeg = bone.rotMax;
            rigBone.rotLimited = bone.rotLimited;
        }
        m_ikRig = std::make_unique<IkRig>();
        m_ikRig->build(rigBones);
        m_ikChildren.assign(m_bones.size(), {});
        for (std::size_t i = 0; i < m_bones.size(); ++i) {
            if (m_bones[i].parent >= 0) {
                m_ikChildren[static_cast<std::size_t>(m_bones[i].parent)].push_back(
                    static_cast<int>(i));
            }
        }
        m_ikBindPos = std::move(bindPos);
    }
    // Current model-space joint positions, plus the ground contacts (detected by WORLD height —
    // the model transform may have grounded/translated the figure). Thresholds scale with the
    // figure's size (a child's standing ankle sits proportionally lower).
    const float sizeScale = m_ikRig->sizeScale();
    std::vector<glm::vec3> positions(m_bones.size());
    std::vector<int> contacts;
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        positions[i] = glm::vec3(m_poseGlobal[i][3]);
        if (m_boneWorldPos[i].y < kIkContactHeight * sizeScale) {
            contacts.push_back(static_cast<int>(i));
        }
    }
    if (contacts.empty()) {
        // Nothing under the threshold (residuals left the figure hovering, or it was posed
        // airborne): anchor at whatever is LOWEST — combined with the rig's ground-healing pin
        // heights, a slightly-floating figure is pulled back onto the floor by its next drag
        // instead of drifting further with each one.
        float minY = 1e30f;
        for (std::size_t i = 0; i < m_bones.size(); ++i) {
            minY = std::min(minY, m_boneWorldPos[i].y);
        }
        for (std::size_t i = 0; i < m_bones.size(); ++i) {
            if (m_boneWorldPos[i].y < minY + 0.06f * sizeScale) {
                contacts.push_back(static_cast<int>(i));
            }
        }
    }
    m_ikPrevTargetValid = false;
    m_ikStillTicks = 0;
    m_ikErrCurMin = 1e30f;
    m_ikErrPrevMin = 1e30f;
    m_ikFrozen = false;
    m_ikSettleTicks = 0;
    m_ikPrevEulerDelta.assign(m_bones.size(), glm::vec3(0.0f));
    m_ikPrevTransDelta.assign(m_bones.size(), glm::vec3(0.0f));
    m_ikAppliedSpeed = 0.0f;
    m_ikGrabOffset = glm::vec3(0.0f); // reset BEFORE the rig call — a failed begin must not
                                      // leave the previous drag's promotion offset behind
    // The rig solves in MODEL space; the world floor (y = 0) sits at model height -ty once the
    // transform translates the figure vertically (the Ground button). Every bind-height ground
    // reference in the rig shifts by this, or pins would heal to a floor that no longer matches
    // the visible one. (The transform is translation-only today — nothing rotates models.)
    const float groundOffsetY = -m_transform[3][1];
    if (!m_ikRig->beginDrag(m_selectedBone, positions, contacts, groundOffsetY)) {
        return false;
    }
    // Token-mass grabs (a finger, a toe, a face bone) are PROMOTED by the rig to the limb's
    // first real-mass joint (the hand, the foot, the head) — a finger pull is an arm gesture.
    // The drag targets from the window track the GRABBED joint, so they are shifted by the
    // model-space grab offset before reaching the rig (constant over the drag: exact at grab,
    // and the small rotation drift of a rigid extremity is imperceptible against the gesture).
    if (m_ikRig->dragEffector() != m_selectedBone) {
        m_ikGrabOffset =
            glm::vec3(m_poseGlobal[static_cast<std::size_t>(m_selectedBone)][3]) -
            glm::vec3(m_poseGlobal[static_cast<std::size_t>(m_ikRig->dragEffector())][3]);
    }
    static const bool kIkTrace = std::getenv("POSESTUDIO_IK_TRACE") != nullptr;
    if (kIkTrace) {
        std::fprintf(stderr, "[ik] beginDrag effector=%s solved=%s pins:",
                     m_boneNames[static_cast<std::size_t>(m_selectedBone)].c_str(),
                     m_boneNames[static_cast<std::size_t>(m_ikRig->dragEffector())].c_str());
        for (const IkEffector& pin : m_ikRig->pins()) {
            std::fprintf(stderr, " %s(y=%.3f w=%.2f)", m_boneNames[static_cast<std::size_t>(pin.node)].c_str(), pin.target.y, pin.weight);
        }
        std::fprintf(stderr, " contacts=%zu\n", contacts.size());
        std::fflush(stderr);
    }
    // Capture each planted foot's current orientation for the flat-sole preservation (see
    // m_ikFlatNodes in mesh.h).
    m_ikStartEuler = m_boneEuler; // the rotational prior (see mesh.h)
    m_ikFlatNodes.clear();
    m_ikFlatRot.clear();
    for (const IkEffector& pin : m_ikRig->pins()) {
        if (pin.node >= 0 && pin.node < static_cast<int>(m_bones.size())) {
            m_ikFlatNodes.push_back(pin.node);
            m_ikFlatRot.push_back(glm::mat3(m_poseGlobal[static_cast<std::size_t>(pin.node)]));
        }
    }
    return true;
}

bool Model::dragIkTo(const glm::vec3& targetWorld) {
    if (!m_ikRig || !m_ikRig->dragActive() || m_bones.empty()) {
        return false;
    }
    // The rig solves in model space; frames seed the constraint orientation from the live pose.
    // The grab offset re-targets a promoted drag (finger -> hand etc., see beginIkDrag).
    const glm::vec3 target =
        glm::vec3(glm::inverse(m_transform) * glm::vec4(targetWorld, 1.0f)) - m_ikGrabOffset;

    // Settle-freeze (see the m_ikFrozen note in mesh.h): once the still-cursor error trend stops
    // improving, stop solving entirely — a genuinely frozen pose, zero trembling — and resume
    // the instant the cursor moves again. This, not budget tuning, is what makes a held pose
    // rock-still: any solving at all lets configuration flip-flops consume whatever budget
    // exists.
    const float targetDelta =
        m_ikPrevTargetValid ? glm::length(target - m_ikPrevTarget) : kIkMaxWorldStep;
    if (!m_ikPrevTargetValid) {
        m_ikStillAnchor = target;
    }
    m_ikPrevTarget = target;
    m_ikPrevTargetValid = true;
    if (glm::length(target - m_ikStillAnchor) > 0.004f) {
        // The target genuinely moved (possibly by slow accumulation): re-anchor and stay live.
        m_ikStillAnchor = target;
        m_ikStillTicks = 0;
        m_ikErrCurMin = 1e30f;
        m_ikErrPrevMin = 1e30f;
        m_ikFrozen = false;
    }
    if (m_ikRig->stepPending()) {
        // A balance step is in flight or its trigger is confirming: both live on the solve
        // ticks, so the settle-freeze must not engage — mid-step it would leave the foot
        // hanging mid-air, and mid-confirm it would race the pelvis-walk's final GATHERING
        // step and plant the figure in a mid-stride stance.
        m_ikStillTicks = 0;
        m_ikErrCurMin = 1e30f;
        m_ikErrPrevMin = 1e30f;
        m_ikFrozen = false;
    }
    if (m_ikFrozen) {
        m_ikAppliedSpeed = 0.0f; // a frozen tick moves nothing: the shaping restarts from rest
        return false; // settled: hold perfectly still until the cursor moves again
    }

    std::vector<glm::vec3> positions(m_bones.size());
    std::vector<glm::quat> frames(m_bones.size());
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        positions[i] = glm::vec3(m_poseGlobal[i][3]);
        frames[i] = glm::quat_cast(glm::mat3(m_poseGlobal[i]));
    }
    // Entry goal error (pre-solve), for the governor's error-scaled settle allowance below.
    float entryErr = glm::length(
        positions[static_cast<std::size_t>(m_ikRig->dragEffector())] - target);
    for (const IkEffector& pin : m_ikRig->pins()) {
        entryErr = std::max(
            entryErr,
            glm::length(positions[static_cast<std::size_t>(pin.node)] - pin.target));
    }
    if (!m_ikRig->solveDrag(target, positions, frames)) {
        m_ikAppliedSpeed = 0.0f; // deadband tick: nothing moved
        return false;
    }
    // Diagnostic: POSESTUDIO_IK_TRACE=1 prints per-tick drag state (world-space heights).
    static const bool kIkTickTrace = std::getenv("POSESTUDIO_IK_TRACE") != nullptr;
    if (kIkTickTrace) {
        const glm::vec3 effW = m_boneWorldPos[static_cast<std::size_t>(m_selectedBone)];
        std::fprintf(stderr, "[ik] tgt(%.3f %.3f %.3f) eff(%.3f %.3f %.3f) pins=%zu frozen=%d\n",
                     targetWorld.x, targetWorld.y, targetWorld.z, effW.x, effW.y, effW.z,
                     m_ikRig->pins().size(), m_ikFrozen ? 1 : 0);
    }

    // Pre-solve pose snapshot for the world-space governor below.
    const std::vector<glm::vec3> preEuler = m_boneEuler;
    const std::vector<glm::vec3> preTranslation = m_boneTranslation;
    std::vector<glm::vec3> preWorld(m_bones.size());
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        preWorld[i] = glm::vec3(m_poseGlobal[i][3]);
    }

    applyIkSolution(positions, m_ikRig->activeNodes(), /*rotationPrior=*/true);

    // World-space governor: every event, only a FRACTION of the solve's pose delta is applied
    // (kIkRelaxation, under-relaxation — damps the alternation between near-equal configurations
    // that reads as trembling), and total per-event movement is capped proportionally to the
    // target's own motion plus a small constant settle allowance. Catch-up (feet re-planting
    // after a crouch) flows through that allowance over ~a second of holding — visible as the
    // figure gently settling, which is exactly the natural motion a drag should show.
    // The floor gives catch-up (feet re-planting behind a fast gesture) a steady flow — but it
    // is scaled by the ENTRY goal error: with goals far off (a crouch's feet re-planting, a
    // hover healing), catch-up flows at the full 12mm/tick, while a near-converged pose during a
    // SLOW pull can only creep (3mm/tick). The fixed floor let solver churn flow at 12mm/tick
    // through the whole slow drag — idle joints visibly trembled while the user was pulling
    // (the settle-freeze only covers a STILL cursor; a creeping drag never freezes).
    const float capFloor = glm::clamp(0.25f * entryErr, 0.003f, 0.012f);
    float cap = glm::clamp(3.0f * targetDelta, capFloor, kIkMaxWorldStep);
    // Motion shaping (see kIkAccel/kIkDecel): ease-in — the allowance grows at most kIkAccel
    // over last tick's applied speed — and ease-out — never above the braking curve toward the
    // goals (the tracking term keeps a moving cursor's pursuit unthrottled).
    cap = std::min(cap, m_ikAppliedSpeed + kIkAccel);
    cap = std::min(cap, std::sqrt(2.0f * kIkDecel * entryErr) + 3.0f * targetDelta);
    cap = std::max(cap, std::min(m_ikAppliedSpeed - kIkBrake, kIkMaxWorldStep));
    float worst = 0.0f;
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        worst = std::max(worst, glm::length(glm::vec3(m_poseGlobal[i][3]) - preWorld[i]));
    }
    const float s = std::min(kIkRelaxation, (worst > 1e-6f) ? cap / worst : 1.0f);
    {
        for (std::size_t i = 0; i < m_bones.size(); ++i) {
            // Reversal-gated micro-motion damping (see kIkMicroDampDeg): a delta opposing the
            // previous tick's applied delta is the loop's own oscillation and eases in softly;
            // anything else passes at full rate — the residual tick-to-tick shimmer of idle
            // joints dies here without shaving sustained motion.
            glm::vec3 eulerDelta(0.0f);
            for (int a = 0; a < 3; ++a) {
                eulerDelta[a] = wrappedAngleDelta(m_boneEuler[i][a], preEuler[i][a]);
            }
            const glm::vec3 transDelta = m_boneTranslation[i] - preTranslation[i];
            float fRot = 1.0f;
            if (glm::dot(eulerDelta, m_ikPrevEulerDelta[i]) < 0.0f) {
                const float e = std::max(
                    {std::abs(eulerDelta.x), std::abs(eulerDelta.y), std::abs(eulerDelta.z)});
                fRot = e / (e + kIkMicroDampDeg);
            }
            float fTrans = 1.0f;
            if (glm::dot(transDelta, m_ikPrevTransDelta[i]) < 0.0f) {
                const float tLen = glm::length(transDelta);
                fTrans = tLen / (tLen + kIkMicroDampTrans);
            }
            for (int a = 0; a < 3; ++a) {
                m_boneEuler[i][a] = preEuler[i][a] + eulerDelta[a] * s * fRot;
            }
            m_boneTranslation[i] = preTranslation[i] + transDelta * s * fTrans;
            m_ikPrevEulerDelta[i] = eulerDelta * s * fRot;
            m_ikPrevTransDelta[i] = transDelta * s * fTrans;
            clampBoneEuler(static_cast<int>(i));
            recomposePoseLocal(i);
        }
        computeSkinMatrices();
    }
    // The velocity state for next tick's motion shaping: what actually moved this tick.
    {
        float applied = 0.0f;
        for (std::size_t i = 0; i < m_bones.size(); ++i) {
            applied = std::max(applied,
                               glm::length(glm::vec3(m_poseGlobal[i][3]) - preWorld[i]));
        }
        m_ikAppliedSpeed = applied;
    }
    // Track the still-cursor error trend for the settle-freeze (see mesh.h): collect worst-goal-
    // error minima in 6-tick windows; a window that fails to improve on the previous one means
    // any remaining motion is churn, not catch-up — freeze.
    float pinErr = 0.0f;
    for (const IkEffector& pin : m_ikRig->pins()) {
        pinErr = std::max(
            pinErr, glm::length(glm::vec3(m_poseGlobal[static_cast<std::size_t>(pin.node)][3]) -
                                pin.target));
    }
    const float err = std::max(
        pinErr,
        glm::length(
            glm::vec3(m_poseGlobal[static_cast<std::size_t>(m_ikRig->dragEffector())][3]) -
            target));
    m_ikErrCurMin = std::min(m_ikErrCurMin, err);
    ++m_ikStillTicks;
    if (m_ikStillTicks % 6 == 0) {
        // Freeze only with the pins decently PLANTED: freezing on a stalled catch-up would lock
        // the figure with its feet in the air. Unplantable pins (pathological pose) keep gently
        // settling instead — motion, not levitation.
        if (m_ikErrPrevMin < 1e29f && m_ikErrCurMin > m_ikErrPrevMin - 0.001f &&
            pinErr < 0.05f) {
            m_ikFrozen = true;
        }
        m_ikErrPrevMin = m_ikErrCurMin;
        m_ikErrCurMin = 1e30f;
    }
    return true;
}

bool Model::settleIkTick() {
    if (!m_ikRig || !m_ikRig->dragActive() || m_bones.empty() || m_ikRig->pins().empty()) {
        return false;
    }
    // The animated release settle: with the drag goal gone, relax the body onto its pins ONE
    // capped round per tick. During the drag an unreachable goal fights the pins every tick and
    // can hold an equilibrium with a foot hovering off its plant; on release those feet must
    // visibly LAND — the old one-shot multi-round settle applied the whole landing in a single
    // frame, which the user saw as the pose popping away from where they let go.
    const auto worstPinError = [&]() {
        float err = 0.0f;
        for (const IkEffector& pin : m_ikRig->pins()) {
            err = std::max(
                err, glm::length(glm::vec3(m_poseGlobal[static_cast<std::size_t>(pin.node)][3]) -
                                 pin.target));
        }
        return err;
    };
    const float errBefore = worstPinError();
    if (errBefore < 0.005f || ++m_ikSettleTicks > 45) {
        return false; // planted (or out of budget — never loop a stubborn pose forever)
    }
    const std::vector<glm::vec3> preEuler = m_boneEuler;
    const std::vector<glm::vec3> preTranslation = m_boneTranslation;
    std::vector<glm::vec3> positions(m_bones.size());
    std::vector<glm::quat> frames(m_bones.size());
    std::vector<glm::vec3> preWorld(m_bones.size());
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        positions[i] = glm::vec3(m_poseGlobal[i][3]);
        frames[i] = glm::quat_cast(glm::mat3(m_poseGlobal[i]));
        preWorld[i] = positions[i];
    }
    if (m_ikSettleTicks == 1) {
        // First settle tick: the CURRENT pose (mouse-up) becomes the settle's prior, and the
        // released joint is pinned right where the user let it go — the pose must HOLD.
        m_ikRig->beginSettle(positions);
    }
    if (!m_ikRig->settleToPins(positions, frames)) {
        return false;
    }
    applyIkSolution(positions, m_ikRig->activeNodes(), /*rotationPrior=*/false);
    // Per-tick smoothness cap (the drag governor's shape with a fixed allowance — there is no
    // target motion to scale by): ~2cm/tick ≈ 1.2 m/s, a quick but visible landing — shaped by
    // the same motion easing as the drag (the release CONTINUES at the drag's speed and brakes
    // into the plant, instead of popping to the fixed rate and stopping flat).
    constexpr float kSettleStepCap = 0.02f;
    const float cap =
        std::min({kSettleStepCap, m_ikAppliedSpeed + kIkAccel,
                  std::sqrt(2.0f * kIkDecel * errBefore)});
    float worst = 0.0f;
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        worst = std::max(worst, glm::length(glm::vec3(m_poseGlobal[i][3]) - preWorld[i]));
    }
    const float s = std::min(kIkRelaxation, (worst > 1e-6f) ? cap / worst : 1.0f);
    if (s < 1.0f) {
        for (std::size_t i = 0; i < m_bones.size(); ++i) {
            for (int a = 0; a < 3; ++a) {
                m_boneEuler[i][a] =
                    preEuler[i][a] + wrappedAngleDelta(m_boneEuler[i][a], preEuler[i][a]) * s;
            }
            m_boneTranslation[i] =
                preTranslation[i] + (m_boneTranslation[i] - preTranslation[i]) * s;
            clampBoneEuler(static_cast<int>(i));
            recomposePoseLocal(i);
        }
        computeSkinMatrices();
    }
    // Pose-hold bound: the released joint must stay where the user let it go — even with it
    // pinned, extraction residual can leak a few mm per tick, and cumulative drift is exactly
    // the "pose changed after release" complaint. Plus the monotone guard: a round that failed
    // to meaningfully improve the worst pin error is reverted and ends the settle — a release
    // can only ever be improved by settling, and it can never wander (the old 1e-4 threshold
    // let the solver fund centimeters of drift with 0.1mm of pin progress).
    bool holdViolated = false;
    if (m_ikRig->dragEffector() >= 0) {
        const glm::vec3 eff =
            glm::vec3(m_poseGlobal[static_cast<std::size_t>(m_ikRig->dragEffector())][3]);
        holdViolated = glm::length(eff - m_ikRig->settleEffectorTarget()) > 0.02f;
    }
    if (holdViolated || worstPinError() > errBefore - 5e-4f) {
        m_boneEuler = preEuler;
        m_boneTranslation = preTranslation;
        for (std::size_t i = 0; i < m_bones.size(); ++i) {
            recomposePoseLocal(i);
        }
        computeSkinMatrices();
        m_ikAppliedSpeed = 0.0f;
        return false;
    }
    // Velocity state for the next settle tick's motion shaping.
    float applied = 0.0f;
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        applied = std::max(applied, glm::length(glm::vec3(m_poseGlobal[i][3]) - preWorld[i]));
    }
    m_ikAppliedSpeed = applied;
    return true;
}

void Model::endIkDrag() {
    if (m_ikRig) {
        m_ikRig->endDrag();
    }
}

void Model::applyIkSolution(const std::vector<glm::vec3>& solved, const std::vector<char>& active,
                            bool rotationPrior) {
    if (solved.size() != m_bones.size() || active.size() != m_bones.size()) {
        return;
    }
    // Walk parents-before-children (the bone order guarantees it), fitting each active bone's
    // world rotation to where the solve put its active children, so descendants extract against
    // their parent's ALREADY-CLAMPED frame and constraint error never compounds down a limb.
    const int rigRoot = m_ikRig ? m_ikRig->rootNode() : -1;
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        GpuBone& bone = m_bones[i];
        const glm::mat4 parentGlobal =
            bone.parent >= 0 ? m_poseGlobal[static_cast<std::size_t>(bone.parent)]
                             : glm::mat4(1.0f);
        if (static_cast<int>(i) == rigRoot && active[i]) {
            // Rotations can't move the solve root (the hip — NOT the figure node real skeletons
            // root at; see IkRig::rootNode): absorb its solved displacement as the pose
            // translation in its parent's frame (this is what lets a feet-pinned crouch drop the
            // hip).
            const glm::vec3 local =
                glm::vec3(glm::inverse(parentGlobal) * glm::vec4(solved[i], 1.0f));
            m_boneTranslation[i] = local - glm::vec3(bone.localBind[3]);
        }
        // The solve root is translation-only here: the solver never rotates it (its edges are
        // frozen — pelvis orientation is an FK decision), so re-fitting its rotation from its
        // tiny child offsets (hip->pelvis is ~2cm) would only amplify solver/FK residual noise
        // into whole-body swings.
        if (active[i] && static_cast<int>(i) != rigRoot) {
            const glm::vec3 jointPos = glm::vec3(
                parentGlobal *
                glm::vec4(glm::vec3(bone.localBind[3]) + m_boneTranslation[i], 1.0f));
            // Primary aim child: the longest-offset active child (most reliable direction)...
            int primary = -1;
            float primaryLen = 0.02f; // sub-2cm offsets are too noise-sensitive to aim by
            for (const int c : m_ikChildren[i]) {
                if (!active[static_cast<std::size_t>(c)]) {
                    continue;
                }
                const float len =
                    glm::length(glm::vec3(m_bones[static_cast<std::size_t>(c)].localBind[3]));
                if (len > primaryLen) {
                    primaryLen = len;
                    primary = c;
                }
            }
            // ...extended THROUGH rigid links: a mid-limb twist bone's swing is locked, so its
            // child follows the chain rigidly and this bone's rotation is what actually places
            // it. Aiming at the twist JOINT itself — a point ON the bone line — constrains only
            // 2 of the 3 rotation DoF, and the unconstrained one is exactly what places the
            // grandchild (the shin off a thigh, the hand off a forearm): extraction then failed
            // to reproduce the solve and every drag move compounded the difference.
            int aimTarget = primary;
            while (aimTarget >= 0) {
                int next = -1;
                float nextLen = 1e-5f;
                for (const int c : m_ikChildren[static_cast<std::size_t>(aimTarget)]) {
                    if (!active[static_cast<std::size_t>(c)] || !m_ikRig->edgeRigid(c)) {
                        continue;
                    }
                    const float len = glm::length(m_ikBindPos[static_cast<std::size_t>(c)] -
                                                  m_ikBindPos[i]);
                    if (len > nextLen) {
                        nextLen = len;
                        next = c;
                    }
                }
                if (next < 0) {
                    break;
                }
                aimTarget = next;
            }
            if (aimTarget >= 0) {
                const glm::vec3 restOffset =
                    m_ikBindPos[static_cast<std::size_t>(aimTarget)] - m_ikBindPos[i];
                glm::vec3 aimDir = solved[static_cast<std::size_t>(aimTarget)] - jointPos;
                const float aimLen = glm::length(aimDir);
                if (aimLen > 1e-5f && glm::dot(restOffset, restOffset) > 1e-10f) {
                    aimDir /= aimLen;
                    const glm::mat3 parentRot = glm::mat3(parentGlobal);
                    // Start from the bone's CURRENT rotation (under the updated parent) so the
                    // shortest-arc fit stays temporally coherent — FABRIK positions carry no
                    // twist, and this preserves the pose's existing twist through the fit.
                    const glm::quat current = glm::quat_cast(parentRot * glm::mat3(bone.poseLocal));
                    const glm::vec3 restAim = glm::normalize(restOffset);
                    glm::quat fitted = shortestArc(current * restAim, aimDir) * current;
                    // Secondary children refine the twist about the aim direction (e.g. the hip
                    // fitting both thighs and the spine). Samples need real off-axis projections
                    // — a direction nearly parallel to the aim axis carries no twist information,
                    // only noise.
                    float twist = 0.0f;
                    int twistSamples = 0;
                    for (const int c : m_ikChildren[i]) {
                        if (c == primary || !active[static_cast<std::size_t>(c)]) {
                            continue;
                        }
                        const glm::vec3 rest =
                            glm::vec3(m_bones[static_cast<std::size_t>(c)].localBind[3]);
                        const glm::vec3 dir = solved[static_cast<std::size_t>(c)] - jointPos;
                        if (glm::length(rest) < 0.02f || glm::length(dir) < 0.02f) {
                            continue;
                        }
                        const glm::vec3 v = fitted * glm::normalize(rest);
                        const glm::vec3 d = glm::normalize(dir);
                        const glm::vec3 vPerp = v - aimDir * glm::dot(v, aimDir);
                        const glm::vec3 dPerp = d - aimDir * glm::dot(d, aimDir);
                        if (glm::dot(vPerp, vPerp) < 0.01f || glm::dot(dPerp, dPerp) < 0.01f) {
                            continue;
                        }
                        twist += signedAngleAround(v, d, aimDir);
                        ++twistSamples;
                    }
                    if (twistSamples > 0) {
                        fitted = glm::angleAxis(twist / static_cast<float>(twistSamples), aimDir) *
                                 fitted;
                    }
                    // Back to this joint's Euler channels: poseGlobal's rotation factors as
                    // parentRot · orient · R · orient⁻¹, so R = orient⁻¹ · parentRotᵀ · fit · orient.
                    const glm::mat3 channel = glm::mat3(bone.invOrient) * glm::transpose(parentRot) *
                                              glm::mat3_cast(fitted) * glm::mat3(bone.orient);
                    const glm::vec3 previous = m_boneEuler[i];
                    const glm::vec3 unlimited = eulerFromMatrix(channel, bone.rotationOrder);
                    for (int a = 0; a < 3; ++a) {
                        // Angular rate limit (kIkMaxJointDeltaDeg): bound per-event pose change.
                        m_boneEuler[i][a] =
                            previous[a] + glm::clamp(wrappedAngleDelta(unlimited[a], previous[a]),
                                                     -kIkMaxJointDeltaDeg, kIkMaxJointDeltaDeg);
                        if (rotationPrior && i < m_ikStartEuler.size()) {
                            // Rotational prior (see m_ikStartEuler): decay toward the drag-start
                            // angle. Components the fit determines are re-imposed next tick;
                            // undetermined drift (unwitnessed twist ratcheting into the clamps'
                            // permissive side — the spine bow) is cleaned instead.
                            m_boneEuler[i][a] +=
                                kIkRotationPrior *
                                wrappedAngleDelta(m_ikStartEuler[i][a], m_boneEuler[i][a]);
                        }
                    }
                    clampBoneEuler(static_cast<int>(i)); // the authoritative anatomical limits
                }
            }
        }
        // WEIGHT-BEARING feet (see m_ikFlatNodes): while its pin holds, a planted foot keeps
        // its DRAG-START world orientation — the sole stays flat on the floor as the shin
        // rotates above it. Skipped once the pins are gone (suspension lifted the figure), and
        // for a foot mid-STEP (it is swinging; the enforcement resumes at its landing and
        // re-flattens the sole through the per-event angular caps).
        if (!m_ikRig->pins().empty()) {
            for (std::size_t f = 0; f < m_ikFlatNodes.size(); ++f) {
                if (m_ikFlatNodes[f] != static_cast<int>(i) || !active[i] ||
                    m_ikFlatNodes[f] == m_ikRig->steppingPin()) {
                    continue;
                }
                const glm::mat3 parentRot = glm::mat3(parentGlobal);
                const glm::mat3 channel = glm::mat3(bone.invOrient) * glm::transpose(parentRot) *
                                          m_ikFlatRot[f] * glm::mat3(bone.orient);
                const glm::vec3 previous = m_boneEuler[i];
                const glm::vec3 unlimited = eulerFromMatrix(channel, bone.rotationOrder);
                for (int a = 0; a < 3; ++a) {
                    m_boneEuler[i][a] =
                        previous[a] + glm::clamp(wrappedAngleDelta(unlimited[a], previous[a]),
                                                 -kIkMaxJointDeltaDeg, kIkMaxJointDeltaDeg);
                }
                clampBoneEuler(static_cast<int>(i));
                break;
            }
        }
        recomposePoseLocal(i);
        m_poseGlobal[i] = parentGlobal * bone.poseLocal; // children below read the updated frame
    }
    // Deliberately NO computeSkinMatrices here: m_poseGlobal is already current (maintained
    // incrementally above — it is all the callers' governors read), and both callers ALWAYS
    // rescale the applied delta and run the one real skinning pass afterwards. The trailing
    // full pass here was pure duplicate work, discarded every tick.
}

} // namespace pose
