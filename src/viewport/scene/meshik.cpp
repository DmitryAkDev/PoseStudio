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
    m_ikAppliedEffSpeed = 0.0f;
    m_ikGrabOffset = glm::vec3(0.0f); // reset BEFORE the rig call — a failed begin must not
                                      // leave the previous drag's promotion offset behind
    // The rig solves in MODEL space; the world floor (y = 0) sits at model height -ty once the
    // transform translates the figure vertically (the Ground button). Every bind-height ground
    // reference in the rig shifts by this, or pins would heal to a floor that no longer matches
    // the visible one. (The transform is translation-only today — nothing rotates models.)
    const float groundOffsetY = -m_transform[3][1];
    std::vector<int> userPins;
    for (std::size_t i = 0; i < m_bonePinned.size() && i < m_bones.size(); ++i) {
        if (m_bonePinned[i]) {
            userPins.push_back(static_cast<int>(i));
        }
    }
    if (!m_ikRig->beginDrag(m_selectedBone, positions, contacts, groundOffsetY, &userPins)) {
        return false;
    }
    // Rotational-prior exemption for each user pin's limb chain (see m_ikRotPriorExempt).
    m_ikRotPriorExempt = m_ikRig->userPinLimbNodes();
    // Token-mass grabs (a finger, a toe, a face bone) are PROMOTED by the rig to the limb's
    // first real-mass joint (the hand, the foot, the head) — a finger pull is an arm gesture.
    // The drag targets from the window track the GRABBED joint, so they are shifted by the
    // model-space grab offset before reaching the rig (constant over the drag: exact at grab,
    // and the small rotation drift of a rigid extremity is imperceptible against the gesture).
    m_ikGrabRotStart = glm::mat3(m_poseGlobal[static_cast<std::size_t>(m_ikRig->dragEffector())]);
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
    // The promoted grab's offset rides the effector's rotation since drag start — for LIMB
    // effectors only. A finger on a twisting hand needs it (a constant offset missed the
    // fingertip by 18cm). A face bone promoted to the HEAD must NOT: the neck's aim fit
    // rotates the head with every head move, the long face offset then swings the target,
    // the head follows, the neck re-aims — a feedback loop with gain near one that made a
    // grabbed head tremble spastically (the eye-grab harness phase: 3.3m of accumulated
    // reversal). The head's own rotation is never solved, so a fixed offset is the honest
    // model there (its error stays within the neck's small range).
    const glm::mat3 grabRot =
        m_ikRig->effectorIsTrunk()
            ? glm::mat3(1.0f)
            : glm::mat3(m_poseGlobal[static_cast<std::size_t>(m_ikRig->dragEffector())]) *
                  glm::transpose(m_ikGrabRotStart);
    glm::vec3 target =
        glm::vec3(glm::inverse(m_transform) * glm::vec4(targetWorld, 1.0f)) - grabRot * m_ikGrabOffset;
    // The floor bounds the cursor too: the grabbed joint cannot be asked below its clearance
    // (the solver would refuse and the exact drag refinement would then push it through).
    target.y = std::max(target.y, -m_transform[3][1] +
                                      m_ikRig->floorClearance(m_ikRig->dragEffector()));

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
        m_ikAppliedEffSpeed = 0.0f;
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
        m_ikAppliedEffSpeed = 0.0f;
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
    // (the settle-freeze only covers a STILL cursor; a creeping drag never freezes). The 12mm
    // ceiling is load-bearing for the crouch: a 25-40mm ceiling let its re-planting feet wander
    // (synthetic crouch foot drift 0.03 -> 0.10-0.13) while barely speeding up a flick's
    // catch-up — the solver's own per-tick progress, not this floor, bounds that.
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
    float s = std::min(kIkRelaxation, (worst > 1e-6f) ? cap / worst : 1.0f);
    // The GRABBED joint's own step is shaped too (see m_ikAppliedEffSpeed): its allowance grows
    // at most kIkAccel over its own last applied speed (never below the settle floor).
    const std::size_t effIdx = static_cast<std::size_t>(m_ikRig->dragEffector());
    const float effMove = glm::length(glm::vec3(m_poseGlobal[effIdx][3]) - preWorld[effIdx]);
    const float effCap = std::max(capFloor, m_ikAppliedEffSpeed + kIkAccel);
    if (effMove * s > effCap && effMove > 1e-6f) {
        s = effCap / effMove;
    }
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
    refinePins(false, &target); // the APPLIED pose is what the user sees: user pins AND the
                                // grabbed joint exact (see mesh.h)
    // The velocity state for next tick's motion shaping: what actually moved this tick.
    {
        float applied = 0.0f;
        for (std::size_t i = 0; i < m_bones.size(); ++i) {
            applied = std::max(applied,
                               glm::length(glm::vec3(m_poseGlobal[i][3]) - preWorld[i]));
        }
        m_ikAppliedSpeed = applied;
        m_ikAppliedEffSpeed = glm::length(glm::vec3(m_poseGlobal[effIdx][3]) - preWorld[effIdx]);
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
    // Planted at 1.5mm: the pin refinement (refinePins) closes a contact's last millimetres
    // exactly, so the settle no longer stops at the old 5mm "close enough" band.
    if (errBefore < 0.0015f || ++m_ikSettleTicks > 45) {
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
        m_ikSettleStrain = errBefore; // see mesh.h: the hold bound scales with this
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
    refinePins(true, nullptr); // pins (and the held effector) exact through the settle too
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
        const float holdBound = m_ikRig->effectorIsTrunk()
                                    ? 0.10f
                                    : glm::clamp(3.0f * m_ikSettleStrain, 0.02f, 0.10f);
        holdViolated = glm::length(eff - m_ikRig->settleEffectorTarget()) > holdBound;
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
                    int witnessedAxis = -1; // the Euler channel the twist witness determined
                    float witnessBlend = 0.0f; // how fully (by the bend that draws the plane)
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
                    // TWIST WITNESS through a bend joint (see JointConstraint::twistAxis): a
                    // bone with a single aim child has no twist witness of its own, but when
                    // that child is a hinge / asymmetric-cone joint whose own child is active,
                    // the plane the solve bent the grandchild in IS the witness — the fold
                    // plane's azimuth about this segment is this bone's twist (the upper arm's
                    // twist places the elbow's fold plane; the thigh's the knee's). The solver
                    // bends within the authored twist range; this realizes the twist it chose.
                    // A bend to the joint's MINOR side flips the tangent, hence the half-turn
                    // wrap. Requires a real bend (2cm of perpendicular reach) to read a plane.
                    if (twistSamples == 0 && primary >= 0 && m_ikRig) {
                        int   grand = -1;
                        float grandLen = 0.02f;
                        for (const int c : m_ikChildren[static_cast<std::size_t>(primary)]) {
                            if (!active[static_cast<std::size_t>(c)]) {
                                continue;
                            }
                            const float len = glm::length(
                                glm::vec3(m_bones[static_cast<std::size_t>(c)].localBind[3]));
                            if (len > grandLen) {
                                grandLen = len;
                                grand = c;
                            }
                        }
                        glm::vec3 tangentRest;
                        const JointConstraint& grandEdge =
                            grand >= 0 ? m_ikRig->edgeConstraint(grand) : JointConstraint{};
                        if (grand >= 0 && grandEdge.twistMax - grandEdge.twistMin > 1e-6f &&
                            dominantBendTangent(
                                grandEdge,
                                glm::normalize(m_ikBindPos[static_cast<std::size_t>(grand)] -
                                               m_ikBindPos[static_cast<std::size_t>(primary)]),
                                tangentRest)) {
                            const glm::vec3 d = solved[static_cast<std::size_t>(grand)] -
                                                solved[static_cast<std::size_t>(primary)];
                            const glm::vec3 dPerp = d - aimDir * glm::dot(d, aimDir);
                            const float dLen = glm::length(d);
                            // The fold plane is only as defined as the BEND that draws it: a
                            // nearly straight limb's few centimetres of perpendicular offset are
                            // steered by millimetres of solver noise, and with the solve
                            // preferring the twist the pose already carries, that noise random-
                            // walked a straight leg's thigh twist to its -71 deg limit during a
                            // strained hand pull (the foot slid 8cm off its pin; the elbow never
                            // showed it — the arm rests with an 18 deg bend). The witnessed twist
                            // is therefore blended in by the bend: nothing under 3 deg, fully
                            // from 12 deg.
                            const float sinBend = dLen > 1e-4f ? glm::length(dPerp) / dLen : 0.0f;
                            const float bendBlend = glm::clamp((sinBend - 0.052f) / (0.208f - 0.052f),
                                                               0.0f, 1.0f);
                            if (bendBlend > 0.0f) {
                                // The EXACT inverse of the solve's twist search: the twist about
                                // this segment, relative to the swing-fitted frame, at which the
                                // grandchild's constraint clamps its solved direction closest —
                                // the azimuth heuristic (rest tangent vs solved bend) was off by
                                // degrees because a hinge axis is not perpendicular to its
                                // parent segment, so the bend circle's azimuth shifts with the
                                // bend angle; the wrist and hand carried 15-30mm of residual.
                                const float tw = fitTwistToDirection(
                                    grandEdge, fitted,
                                    glm::normalize(m_ikBindPos[static_cast<std::size_t>(grand)] -
                                                   m_ikBindPos[static_cast<std::size_t>(primary)]),
                                    d / dLen);
                                fitted = glm::angleAxis(tw * bendBlend, aimDir) * fitted;
                                witnessBlend = bendBlend;
                                // The witnessed twist lives in the Euler channel whose oriented
                                // axis runs along the segment (the rig's twist-axis rule): that
                                // channel is now DETERMINED, so the rotational prior below —
                                // which exists to clean UNDETERMINED drift — must leave it be
                                // (it took back 5% of a 70° twist every tick, a steady 3.5°
                                // fold-plane error that left the hand centimetres off).
                                float bestAlign = 0.0f;
                                for (int a = 0; a < 3; ++a) {
                                    const float align = std::abs(glm::dot(
                                        glm::normalize(glm::vec3(bone.orient[a])), restAim));
                                    if (align > bestAlign) {
                                        bestAlign = align;
                                        witnessedAxis = a;
                                    }
                                }
                            }
                        }
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
                        // The witnessed channel yields the prior only as far as the witness is
                        // trusted (the bend blend): a barely bent limb's plane is noise, and a
                        // fully exempt twist integrated that noise into a random walk (the
                        // synthetic rig's straight-resting arm lost its reach).
                        const float priorScale =
                            (a == witnessedAxis) ? (1.0f - witnessBlend) : 1.0f;
                        if (rotationPrior && priorScale > 0.0f && i < m_ikStartEuler.size() &&
                            !(i < m_ikRotPriorExempt.size() && m_ikRotPriorExempt[i])) {
                            // Rotational prior (see m_ikStartEuler): decay toward the drag-start
                            // angle. Components the fit determines are re-imposed next tick;
                            // undetermined drift (unwitnessed twist ratcheting into the clamps'
                            // permissive side — the spine bow) is cleaned instead. A user pin's
                            // limb chain is exempt (see m_ikRotPriorExempt).
                            m_boneEuler[i][a] +=
                                kIkRotationPrior * priorScale *
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

namespace {

// Solves the small dense system A·y = b in place (Gaussian elimination with partial pivoting);
// @p A is n×n row-major, @p b becomes y. Returns false on a singular system.
bool solveDense(std::vector<float>& A, std::vector<float>& b, int n) {
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        for (int row = col + 1; row < n; ++row) {
            if (std::abs(A[static_cast<std::size_t>(row * n + col)]) >
                std::abs(A[static_cast<std::size_t>(pivot * n + col)])) {
                pivot = row;
            }
        }
        const float pv = A[static_cast<std::size_t>(pivot * n + col)];
        if (std::abs(pv) < 1e-12f) {
            return false;
        }
        if (pivot != col) {
            for (int k = 0; k < n; ++k) {
                std::swap(A[static_cast<std::size_t>(col * n + k)],
                          A[static_cast<std::size_t>(pivot * n + k)]);
            }
            std::swap(b[static_cast<std::size_t>(col)], b[static_cast<std::size_t>(pivot)]);
        }
        for (int row = col + 1; row < n; ++row) {
            const float f = A[static_cast<std::size_t>(row * n + col)] / pv;
            if (f == 0.0f) {
                continue;
            }
            for (int k = col; k < n; ++k) {
                A[static_cast<std::size_t>(row * n + k)] -= f * A[static_cast<std::size_t>(col * n + k)];
            }
            b[static_cast<std::size_t>(row)] -= f * b[static_cast<std::size_t>(col)];
        }
    }
    for (int row = n - 1; row >= 0; --row) {
        float s = b[static_cast<std::size_t>(row)];
        for (int k = row + 1; k < n; ++k) {
            s -= A[static_cast<std::size_t>(row * n + k)] * b[static_cast<std::size_t>(k)];
        }
        b[static_cast<std::size_t>(row)] = s / A[static_cast<std::size_t>(row * n + row)];
    }
    return true;
}

// Exact user-pin refinement (Model::refineUserPins): iterations, convergence tolerance (m), the
// numeric-Jacobian probe (degrees), the per-iteration channel step cap (degrees — residuals are
// millimetres, so steps are fractions of a degree; the cap only guards a near-singular chain),
// the damped-least-squares damping (m/deg — the Jacobian entries are lever × π/180, ~0.01 for
// a shin, so this damps ~5%), the orientation residual's scale (metres per radian: 1° of pin
// rotation counts like 3.5mm — position wins when a limit forces a compromise, so a pinned
// joint holds its PLACE first and its orientation as far as the limb allows), and the root
// translation's unit (metres per DoF unit: a 2mm pelvis shift costs what a 1° rotation does,
// i.e. ~5× per centimetre of effect — the joints absorb the residual first).
constexpr int   kPinRefineIters = 10;
constexpr float kPinRefineTol = 1e-4f;
constexpr float kPinJacobianDeltaDeg = 0.2f;
constexpr float kPinRefineMaxStepDeg = 3.0f;
constexpr float kPinRefineDamping = 0.002f;
constexpr float kPinOrientScale = 0.2f;
constexpr float kPinRootUnit = 0.002f;
// CONTACT-pin refinement (the drag's ground contacts and the settle's held effector, which are
// soft in the solver; RELEASE SETTLE only — see refinePins): the residual band — a contact
// farther off than this is genuine catch-up (the settle's animated landing), NOT extraction
// residual, and is left alone; the per-tick correction cap, so the last centimetre of a landing
// stays eased instead of snapping; the orientation band, inside which the sole's drag-start
// orientation is re-imposed exactly (outside it the extraction's capped flat-sole hold brings
// it in); and the row weight that makes a USER pin outrank a contact sharing its chain.
constexpr float kContactRefineBand = 0.010f;
constexpr float kContactRefineStep = 0.003f;
constexpr float kContactRefineRotBand = 0.05235988f; // 3 degrees
constexpr float kPinUserRowWeight = 4.0f;
// The DRAG target's correction (the grabbed joint is closed onto the cursor every drag tick, see
// refinePins) is a FINISHER: full within kDragRefineFull of the target, fading linearly to
// nothing at kDragRefineFade. Per-tick residuals of an ordinary drag (the relaxation's leftover,
// ~1-2cm) close exactly; a large gap (a fast flick, a 15cm foot lift) stays with the solve's own
// posture choice until the limb is nearly there.
constexpr float kDragRefineFull = 0.02f;
constexpr float kDragRefineFade = 0.06f;

} // namespace

void Model::refinePins(bool settling, const glm::vec3* dragTarget) {
    if (!m_ikRig || !m_ikRig->dragActive() || m_bones.empty()) {
        return;
    }
    const int n = static_cast<int>(m_bones.size());
    const std::vector<IkEffector>& pins = m_ikRig->pins();
    const int stepping = m_ikRig->steppingPin();
    struct PinRef {
        int              node = -1;
        glm::vec3        target{0.0f};
        std::vector<int> chain; // the pin's parent first, up to (excluding) the limb junction
        int              flat = -1; // index into m_ikFlatNodes/m_ikFlatRot (orientation), or -1
        int              row = 0;   // first residual row
        int              rows = 3;  // 3 = position only, 6 = position + orientation
        float            weight = 1.0f; // residual row weight (user pins outrank contacts)
        bool             drag = false;   // the drag target: never served by the root stage
    };
    std::vector<PinRef> refs;
    int m = 0; // total residual rows
    bool anyUser = false; // the root stage serves USER pins only (see below)
    // Angle (radians) between @p node's current world rotation and its drag-start one.
    const auto flatRotError = [&](int node, int flat) {
        const glm::mat3 rel = glm::transpose(m_ikFlatRot[static_cast<std::size_t>(flat)]) *
                              glm::mat3(m_poseGlobal[static_cast<std::size_t>(node)]);
        const float c = glm::clamp((rel[0][0] + rel[1][1] + rel[2][2] - 1.0f) * 0.5f, -1.0f, 1.0f);
        return std::acos(c);
    };
    // kind: 0 = contact pin (settle only), 1 = user pin, 2 = the drag target.
    const auto addRef = [&](int node, const glm::vec3& target, int kind) {
        const bool user = kind == 1;
        if (node < 0 || node >= n || node == stepping) {
            return; // a foot mid-STEP is swinging: nothing to hold it to yet
        }
        for (const PinRef& r : refs) {
            if (r.node == node) {
                return;
            }
        }
        PinRef ref;
        ref.node = node;
        int flat = -1;
        for (std::size_t f = 0; f < m_ikFlatNodes.size(); ++f) {
            if (m_ikFlatNodes[f] == node) {
                flat = static_cast<int>(f);
                break;
            }
        }
        if (user) {
            ref.target = target;
            ref.weight = kPinUserRowWeight;
            ref.flat = flat;
            anyUser = true;
        } else if (kind == 2) {
            // The DRAG TARGET itself (see mesh.h): the grabbed joint is closed onto the
            // filtered cursor EXACTLY every drag tick through its own limb — the pin machinery
            // with the cursor as the pin. Whatever part of the target the limb can reach from
            // the current trunk it reaches now; the trunk, legs, and balance keep following on
            // the damped FABRIK dynamics underneath. Never band-gated (closing the gap IS the
            // point), no orientation rows, capped only against warps.
            const glm::vec3 pos(m_poseGlobal[static_cast<std::size_t>(node)][3]);
            const glm::vec3 delta = target - pos;
            const float dist = glm::length(delta);
            const float w = glm::clamp((kDragRefineFade - dist) / (kDragRefineFade - kDragRefineFull),
                                       0.0f, 1.0f);
            if (w <= 0.0f) {
                return; // far off: the solve's own posture choice drives this gap
            }
            ref.target = pos + delta * w;
            ref.flat = -1;
            ref.drag = true;
        } else {
            // CONTACT pins (the drag's ground contacts, the settle's held effector): held
            // exactly too, but ONLY IN THE RELEASE SETTLE, and only as RESIDUAL cleanup. During
            // the drag they are left to the solver: an exact per-tick hold there erased the
            // FK slip that IS the balance stepper's strain signal (the foot held until the
            // strain exceeded the band, then jumped 2cm in one tick and a strained release
            // landed 2cm off instead of 5mm), and the pelvis stage fought the solver's root
            // (fast-drag catch-up never completed, walk landings fell 2cm short). At rest —
            // the settle — exactness is the contract. A contact farther off than the band is
            // genuine catch-up (the settle's own animated landing) and is left alone; the
            // correction is capped per tick so the last centimetre lands eased (a 6-8mm cap
            // measurably worsened the settle's per-tick residual); and the sole's orientation
            // is re-imposed only once the extraction's flat-sole hold has brought it within
            // the rotation band.
            if (!settling) {
                return;
            }
            const glm::vec3 pos(m_poseGlobal[static_cast<std::size_t>(node)][3]);
            const glm::vec3 delta = target - pos;
            const float dist = glm::length(delta);
            if (dist > kContactRefineBand) {
                return;
            }
            ref.target = dist > kContactRefineStep ? pos + delta * (kContactRefineStep / dist)
                                                   : target;
            ref.flat = (flat >= 0 && flatRotError(node, flat) < kContactRefineRotBand) ? flat : -1;
        }
        // The pin's limb chain (see IkRig::limbJunction): its parent up to, excluding, the
        // junction where the limb joins the axial skeleton. A pin ON the solve root has no
        // chain (the root's virtual ancestors — the figure node — must never be rotated).
        const int junction = m_ikRig->limbJunction(node);
        const int rigRoot = m_ikRig->rootNode();
        for (int cur = m_bones[static_cast<std::size_t>(node)].parent;
             cur >= 0 && cur != junction && cur != rigRoot;
             cur = m_bones[static_cast<std::size_t>(cur)].parent) {
            ref.chain.push_back(cur);
        }
        ref.rows = ref.flat >= 0 ? 6 : 3;
        ref.row = m;
        m += ref.rows;
        refs.push_back(std::move(ref));
    };
    for (std::size_t p = 0; p < pins.size(); ++p) {
        addRef(pins[p].node, pins[p].target, m_ikRig->pinIsUser(p) ? 1 : 0);
    }
    if (settling && m_ikRig->dragEffector() >= 0 && !m_ikRig->effectorIsTrunk()) {
        // The release settle holds the let-go joint at its mouse-up position (the pose
        // contract): exact, through its own limb, like a contact. (Holding it with the drag
        // finisher instead was measured and rejected: the wider range let the settle snap the
        // arm 11cm in one tick.)
        addRef(m_ikRig->dragEffector(), m_ikRig->settleEffectorTarget(), 0);
    }
    if (!settling && dragTarget != nullptr && m_ikRig->dragEffector() >= 0 &&
        m_ikRig->dragEffector() != m_ikRig->rootNode()) {
        // A pelvis drag places the root directly in the solve; every other effector is closed
        // onto the cursor here.
        addRef(m_ikRig->dragEffector(), *dragTarget, 2);
    }
    if (refs.empty()) {
        return;
    }

    // Degrees of freedom, in two stages. Stage 1: every unlocked Euler channel of every chain
    // joint (union over pins — overlapping chains, e.g. a pinned hand and a pinned finger, are
    // solved jointly), plus the pin's OWN channels for an oriented pin (they steer its
    // orientation; position rows read 0). Stage 2, only if the joints cannot close the residual
    // (a limit-blocked chain, or a straight leg that cannot lengthen when the pelvis landed a
    // few millimetres too far from the planted foot): the solve root's pose TRANSLATION — the
    // pelvis shifts by the millimetres the anchored limb demands, as a real body's does. Its
    // unit (kPinRootUnit) prices a pelvis shift well above a joint rotation of equal effect.
    // The root stage exists for USER pins only: a pelvis shift on behalf of a CONTACT moved
    // the settle's other landings (a walk's trailing foot ended 2.6cm from where it had been
    // landing) — contacts get the joints, and what the joints cannot close stays.
    struct Dof {
        int  joint;
        int  axis;
        bool translation;
    };
    std::vector<Dof>  dofs;
    std::vector<char> dofJoint(m_bones.size(), 0);
    const auto addJointDofs = [&](int j) {
        if (dofJoint[static_cast<std::size_t>(j)]) {
            return;
        }
        dofJoint[static_cast<std::size_t>(j)] = 1;
        const GpuBone& b = m_bones[static_cast<std::size_t>(j)];
        for (int a = 0; a < 3; ++a) {
            if (b.rotLimited[a] && b.rotMax[a] - b.rotMin[a] < 1e-3f) {
                continue; // locked channel
            }
            dofs.push_back({j, a, false});
        }
    };
    for (const PinRef& ref : refs) {
        for (const int j : ref.chain) {
            addJointDofs(j);
        }
        if (ref.rows == 6) {
            addJointDofs(ref.node);
        }
    }
    const std::size_t rotDofs = dofs.size();
    const int rigRoot = m_ikRig->rootNode();
    if (anyUser && rigRoot >= 0 && rigRoot < n) {
        for (int a = 0; a < 3; ++a) {
            dofs.push_back({rigRoot, a, true});
        }
    }
    const std::size_t D = dofs.size();
    const auto influences = [](const PinRef& ref, const Dof& dof) {
        if (dof.translation) {
            // The root stage serves USER pins only — never the drag target: the pelvis is the
            // FABRIK layer's decision, and letting the cursor recruit it here dragged a pinned
            // foot 6mm off its pin under a chest drag (the two rows competed for the root).
            return !ref.drag;
        }
        if (dof.joint == ref.node) {
            return true;
        }
        for (const int c : ref.chain) {
            if (c == dof.joint) {
                return true;
            }
        }
        return false;
    };
    // The pin's world matrix with the chain's poseLocal re-composed from joint @p top down to
    // the pin (m_poseGlobal above @p top is read as-is; @p top is on the chain or the pin).
    const auto pinPoseFrom = [&](const PinRef& ref, int top) {
        int startIdx = -1;
        for (std::size_t c = 0; c < ref.chain.size(); ++c) {
            if (ref.chain[c] == top) {
                startIdx = static_cast<int>(c);
                break;
            }
        }
        const int parentOfTop = m_bones[static_cast<std::size_t>(top)].parent;
        glm::mat4 g = parentOfTop >= 0 ? m_poseGlobal[static_cast<std::size_t>(parentOfTop)]
                                       : glm::mat4(1.0f);
        for (int c = startIdx; c >= 0; --c) {
            g = g * m_bones[static_cast<std::size_t>(ref.chain[static_cast<std::size_t>(c)])].poseLocal;
        }
        return g * m_bones[static_cast<std::size_t>(ref.node)].poseLocal;
    };
    // Residual rows of @p ref for world matrix @p g: the position error, then (oriented pins)
    // the rotation vector taking g's rotation onto the drag-start rotation, scaled to metres.
    const auto residual = [&](const PinRef& ref, const glm::mat4& g, float* out) {
        const glm::vec3 pos(g[3]);
        for (int c = 0; c < 3; ++c) {
            out[c] = ref.target[c] - pos[c];
        }
        if (ref.rows == 6) {
            const glm::mat3 R(g);
            const glm::mat3& F = m_ikFlatRot[static_cast<std::size_t>(ref.flat)];
            glm::quat q = glm::quat_cast(F * glm::transpose(R));
            if (q.w < 0.0f) {
                q = -q;
            }
            const glm::vec3 v(q.x, q.y, q.z);
            const float vl = glm::length(v);
            const glm::vec3 omega =
                vl > 1e-9f ? v * (2.0f * std::atan2(vl, q.w) / vl) : glm::vec3(0.0f);
            for (int c = 0; c < 3; ++c) {
                out[3 + c] = omega[c] * kPinOrientScale;
            }
        }
    };
    // Full pose refresh (bones are parent-first): cheap, and the root stage moves everything.
    const auto refreshAll = [&]() {
        for (int i = 0; i < n; ++i) {
            const int p = m_bones[static_cast<std::size_t>(i)].parent;
            m_poseGlobal[static_cast<std::size_t>(i)] =
                (p >= 0 ? m_poseGlobal[static_cast<std::size_t>(p)] : glm::mat4(1.0f)) *
                m_bones[static_cast<std::size_t>(i)].poseLocal;
        }
    };
    const auto worstResidual = [&](std::vector<float>& res) {
        float worst = 0.0f;
        for (const PinRef& ref : refs) {
            residual(ref, m_poseGlobal[static_cast<std::size_t>(ref.node)],
                     &res[static_cast<std::size_t>(ref.row)]);
            for (int k = 0; k < ref.rows; ++k) {
                worst = std::max(worst, std::abs(res[static_cast<std::size_t>(ref.row + k)]));
            }
        }
        return worst;
    };
    // Row weights: where a user pin and a contact share a chain (a pinned knee over a planted
    // foot), the least-squares compromise favours the user's pin — it is the declared intent.
    std::vector<float> rowWeight(static_cast<std::size_t>(m), 1.0f);
    for (const PinRef& ref : refs) {
        for (int k = 0; k < ref.rows; ++k) {
            rowWeight[static_cast<std::size_t>(ref.row + k)] = ref.weight;
        }
    }

    std::vector<char> blocked(D, 0); // channels the limit clamp stopped: dropped from the fit
    for (std::size_t d = rotDofs; d < D; ++d) {
        blocked[d] = 1; // stage 1: joints only
    }
    bool rootStage = false;
    const auto enableRoot = [&]() {
        rootStage = true;
        for (std::size_t d = rotDofs; d < D; ++d) {
            blocked[d] = 0;
        }
    };
    std::vector<float> res(static_cast<std::size_t>(m), 0.0f);
    std::vector<float> J;
    std::vector<float> A;
    std::vector<float> y;
    float tmp[6];
    int   itersUsed = 0;
    for (int iter = 0; iter < 2 * kPinRefineIters && D > 0; ++iter) {
        if (worstResidual(res) < kPinRefineTol) {
            break;
        }
        if (!rootStage && iter >= kPinRefineIters) {
            enableRoot();
        }
        ++itersUsed;
        // Jacobian of the residual, rows = residual components, columns = channels: numeric
        // for the rotation channels; analytic for the root translation (a unit shift along
        // the root's parent-frame axis moves every pin by exactly that, orientations untouched).
        // Rows are scaled by their weight (a weighted least-squares fit).
        J.assign(static_cast<std::size_t>(m) * D, 0.0f);
        for (std::size_t d = 0; d < D; ++d) {
            if (blocked[d]) {
                continue;
            }
            const int j = dofs[d].joint;
            const int a = dofs[d].axis;
            if (dofs[d].translation) {
                const int rp = m_bones[static_cast<std::size_t>(j)].parent;
                const glm::mat3 parentRot =
                    rp >= 0 ? glm::mat3(m_poseGlobal[static_cast<std::size_t>(rp)]) : glm::mat3(1.0f);
                const glm::vec3 shift = parentRot[a] * kPinRootUnit;
                for (const PinRef& ref : refs) {
                    if (ref.drag) {
                        continue; // see influences(): the drag target never moves the root
                    }
                    for (int c = 0; c < 3; ++c) {
                        J[static_cast<std::size_t>(ref.row + c) * D + d] =
                            -shift[c] * rowWeight[static_cast<std::size_t>(ref.row + c)];
                    }
                }
                continue;
            }
            const float save = m_boneEuler[static_cast<std::size_t>(j)][a];
            m_boneEuler[static_cast<std::size_t>(j)][a] = save + kPinJacobianDeltaDeg;
            recomposePoseLocal(static_cast<std::size_t>(j));
            for (const PinRef& ref : refs) {
                if (!influences(ref, dofs[d])) {
                    continue;
                }
                residual(ref, pinPoseFrom(ref, j), tmp);
                for (int k = 0; k < ref.rows; ++k) {
                    J[static_cast<std::size_t>(ref.row + k) * D + d] =
                        (tmp[k] - res[static_cast<std::size_t>(ref.row + k)]) /
                        kPinJacobianDeltaDeg * rowWeight[static_cast<std::size_t>(ref.row + k)];
                }
            }
            m_boneEuler[static_cast<std::size_t>(j)][a] = save;
            recomposePoseLocal(static_cast<std::size_t>(j));
        }
        // Damped least squares: dTheta = -J^T (J J^T + lambda^2 I)^-1 r (J is the residual's
        // derivative).
        A.assign(static_cast<std::size_t>(m * m), 0.0f);
        y.assign(static_cast<std::size_t>(m), 0.0f);
        for (int i = 0; i < m; ++i) {
            for (int k = 0; k < m; ++k) {
                float s = 0.0f;
                for (std::size_t d = 0; d < D; ++d) {
                    s += J[static_cast<std::size_t>(i) * D + d] * J[static_cast<std::size_t>(k) * D + d];
                }
                A[static_cast<std::size_t>(i * m + k)] = s;
            }
            A[static_cast<std::size_t>(i * m + i)] += kPinRefineDamping * kPinRefineDamping;
            y[static_cast<std::size_t>(i)] =
                -res[static_cast<std::size_t>(i)] * rowWeight[static_cast<std::size_t>(i)];
        }
        if (!solveDense(A, y, m)) {
            break;
        }
        bool moved = false;
        for (std::size_t d = 0; d < D; ++d) {
            if (blocked[d]) {
                continue;
            }
            float dTheta = 0.0f;
            for (int k = 0; k < m; ++k) {
                dTheta += J[static_cast<std::size_t>(k) * D + d] * y[static_cast<std::size_t>(k)];
            }
            dTheta = glm::clamp(dTheta, -kPinRefineMaxStepDeg, kPinRefineMaxStepDeg);
            if (std::abs(dTheta) < 1e-6f) {
                continue;
            }
            const int j = dofs[d].joint;
            const int a = dofs[d].axis;
            if (dofs[d].translation) {
                m_boneTranslation[static_cast<std::size_t>(j)][a] += dTheta * kPinRootUnit;
                recomposePoseLocal(static_cast<std::size_t>(j));
                moved = true;
                continue;
            }
            const float requested = m_boneEuler[static_cast<std::size_t>(j)][a] + dTheta;
            m_boneEuler[static_cast<std::size_t>(j)][a] = requested;
            clampBoneEuler(j); // the authoritative anatomical limits
            if (std::abs(m_boneEuler[static_cast<std::size_t>(j)][a] - requested) > 1e-4f) {
                blocked[d] = 1; // at its limit in the needed direction: let the others absorb it
            }
            recomposePoseLocal(static_cast<std::size_t>(j));
            moved = true;
        }
        if (!moved) {
            if (!rootStage) {
                enableRoot(); // the joints have nothing left to give: the pelvis stage
                continue;
            }
            break;
        }
        refreshAll();
    }
    static const bool kPinTrace = std::getenv("POSESTUDIO_IK_PIN_TRACE") != nullptr;
    if (kPinTrace) {
        const float worst = worstResidual(res);
        if (worst > 5e-4f) {
            std::fprintf(stderr, "[pin] residual %.5f after %d iters (root stage %d), blocked:",
                         worst, itersUsed, rootStage ? 1 : 0);
            for (std::size_t d = 0; d < rotDofs; ++d) {
                if (blocked[d]) {
                    std::fprintf(stderr, " %s.%c",
                                 m_boneNames[static_cast<std::size_t>(dofs[d].joint)].c_str(),
                                 "xyz"[dofs[d].axis]);
                }
            }
            std::fprintf(stderr, "\n");
        }
    }
    computeSkinMatrices();
}

bool Model::togglePinSelectedBone() {
    if (m_selectedBone < 0 || m_selectedBone >= static_cast<int>(m_bones.size())) {
        return false;
    }
    if (m_bonePinned.size() != m_bones.size()) {
        m_bonePinned.assign(m_bones.size(), 0);
    }
    char& flag = m_bonePinned[static_cast<std::size_t>(m_selectedBone)];
    flag = flag ? 0 : 1;
    return flag != 0;
}

bool Model::hasPinnedBones() const {
    for (const char p : m_bonePinned) {
        if (p) {
            return true;
        }
    }
    return false;
}

void Model::unpinAllBones() {
    std::fill(m_bonePinned.begin(), m_bonePinned.end(), 0);
}

std::vector<int> Model::activeContactPins() const {
    std::vector<int> out;
    if (m_ikRig && m_ikRig->dragActive()) {
        const std::vector<IkEffector>& pins = m_ikRig->pins();
        for (std::size_t p = 0; p < pins.size(); ++p) {
            if (!m_ikRig->pinIsUser(p)) {
                out.push_back(pins[p].node);
            }
        }
    }
    return out;
}

} // namespace pose
