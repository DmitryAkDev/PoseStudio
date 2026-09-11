/**
 * @file armatureik.cpp
 * @brief Armature's full-body-IK integration: the engine boundary of the FBIK system (scene/ik/)
 *        — the drag lifecycle (beginIkDrag / dragIkTo / settleIkTick / endIkDrag) with its
 *        damped-motion governors, and the user-pin API.
 *
 * Everything FBIK needs from the engine lives in three translation units. This file builds the
 * per-figure IkRig from the bind skeleton, detects ground contacts, drives the per-tick drag
 * solve with its governors (under-relaxation, proportional world-space cap, motion shaping,
 * reversal-gated micro-damping, settle-freeze) and the animated pose-holding release settle;
 * armatureikextract.cpp holds the ROTATION EXTRACTION that converts the solver's joint positions
 * back into the engine's per-channel Euler pose (+ the root's pose translation); and
 * armatureikpins.cpp holds the exact joint-space pin refinement (damped least squares) with the
 * drag-tick floor lift. The governor constants the first two share sit in armatureik_detail.h.
 * The solver itself is armature-agnostic (scene/ik/, pure positions); these files are the only
 * place the two meet, so solver work and engine work can evolve independently. Vulkan-free like
 * the rest of the Armature, so the IK harness runs this exact loop. See CLAUDE.md's FBIK section
 * for the design history — most numeric policies here trace to a specific user-visible failure,
 * documented at their definitions.
 */

#include "armature.h"

#include "armatureik_detail.h" // the governor constants + wrappedAngleDelta
#include "ikrig.h"             // full-body IK orchestrator

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstddef>
#include <cstdlib>
#include <vector>

namespace pose {

namespace {

// POSESTUDIO_IK_TRACE=1: the per-drag (beginIkDrag) and per-tick (dragIkTo) diagnostics.
const bool kIkTrace = std::getenv("POSESTUDIO_IK_TRACE") != nullptr;

} // namespace

bool Armature::beginIkDrag() {
    if (m_selectedBone < 0 || m_selectedBone >= static_cast<int>(m_bones.size()) ||
        m_bones.empty()) {
        return false;
    }
    if (!m_ikRig) {
        // First IK drag on this figure: build the rig (graph, constraints, masses) from the bind
        // skeleton, plus the model-space bind joint positions the extraction walk reads.
        std::vector<IkRigBone> rigBones(m_bones.size());
        std::vector<glm::vec3> bindPos(m_bones.size(), glm::vec3(0.0f));
        for (std::size_t i = 0; i < m_bones.size(); ++i) {
            const Bone& bone = m_bones[i];
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
    m_ikInvTransform = glm::inverse(m_transform); // fixed for the drag (see armature.h)
    std::vector<int> userPins;
    for (std::size_t i = 0; i < m_bonePinned.size() && i < m_bones.size(); ++i) {
        if (m_bonePinned[i]) {
            userPins.push_back(static_cast<int>(i));
        }
    }
    if (!m_ikRig->beginDrag(m_selectedBone, positions, contacts, groundOffsetY, &userPins)) {
        return false;
    }
    // Rotational-prior exemption for each user pin's limb chain (see m_ikRotPriorExempt) — and
    // for the DRAGGED limb's own chain (a limb effector only: a trunk drag's "limb" is the
    // spine, whose unwitnessed twist is exactly what the prior exists to hold). The drag
    // finisher determines those joints every tick as a pin's restoration does; under the
    // prior they crept toward the drag-start pose between the solver's corrections — a lifted
    // foot's knee wandered outward 0.1mm a tick and snapped back 2-4mm every ten (the
    // reversal metric read 150mm of accumulated knee tremble over one 15cm foot lift).
    m_ikRotPriorExempt = m_ikRig->userPinLimbNodes();
    m_ikRotPriorSwingExempt.assign(m_bones.size(), 0);
    if (!m_ikRig->effectorIsTrunk()) {
        const int junction = m_ikRig->limbJunction(m_ikRig->dragEffector());
        for (int cur = m_ikRig->dragEffector(); cur >= 0 && cur != junction;
             cur = m_bones[static_cast<std::size_t>(cur)].parent) {
            m_ikRotPriorSwingExempt[static_cast<std::size_t>(cur)] = 1; // swing only: see armature.h
        }
    }
    // Token-mass grabs (a finger, a toe, a face bone) are PROMOTED by the rig to the limb's
    // first real-mass joint (the hand, the foot, the head) — a finger pull is an arm gesture.
    // The drag targets from the window track the GRABBED joint, so they are shifted by the
    // model-space grab offset before reaching the rig — captured here, then carried through the
    // effector's rotation since drag start on every tick for LIMB effectors (see dragIkTo: a
    // constant offset missed a twisting hand's fingertip by 18cm) and held fixed for a trunk
    // effector.
    m_ikGrabRotStart = glm::mat3(m_poseGlobal[static_cast<std::size_t>(m_ikRig->dragEffector())]);
    if (m_ikRig->dragEffector() != m_selectedBone) {
        m_ikGrabOffset =
            glm::vec3(m_poseGlobal[static_cast<std::size_t>(m_selectedBone)][3]) -
            glm::vec3(m_poseGlobal[static_cast<std::size_t>(m_ikRig->dragEffector())][3]);
    }
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
    // m_ikFlatNodes in armature.h).
    m_ikStartEuler = m_boneEuler; // the rotational prior (see armature.h)
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

bool Armature::dragIkTo(const glm::vec3& targetWorld) {
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
        glm::vec3(m_ikInvTransform * glm::vec4(targetWorld, 1.0f)) - grabRot * m_ikGrabOffset;
    // The floor bounds the cursor too: the grabbed joint cannot be asked below its clearance
    // (the solver would refuse and the exact drag refinement would then push it through).
    target.y = std::max(target.y, -m_transform[3][1] +
                                      m_ikRig->floorClearance(m_ikRig->dragEffector()));

    // Settle-freeze (see the m_ikFrozen note in armature.h): once the still-cursor error trend stops
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
    if (m_ikRig->stepPending() || m_ikRig->suspendPending()) {
        // A balance step is in flight or its trigger is confirming (or a lift is confirming):
        // all live on the solve ticks, so the settle-freeze must not engage — mid-step it
        // would leave the foot hanging mid-air, and mid-confirm it would race the pelvis-walk's
        // final GATHERING step and plant the figure in a mid-stride stance.
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
    // Diagnostic: POSESTUDIO_IK_TRACE=1 prints per-tick drag state (world-space heights), with
    // the worst contact-pin error at solve entry (the FK pose), in the solver's output, and —
    // appended at the end of the tick — in the applied pose.
    if (kIkTrace) {
        const glm::vec3 effW = m_boneWorldPos[static_cast<std::size_t>(m_selectedBone)];
        float solvedPinErr = 0.0f;
        for (const IkEffector& pin : m_ikRig->pins()) {
            solvedPinErr = std::max(
                solvedPinErr,
                glm::length(positions[static_cast<std::size_t>(pin.node)] - pin.target));
        }
        const float solvedEffErr = glm::length(
            positions[static_cast<std::size_t>(m_ikRig->dragEffector())] - target);
        std::fprintf(stderr,
                     "[ik] tgt(%.3f %.3f %.3f) eff(%.3f %.3f %.3f) pins=%zu frozen=%d pinErr "
                     "entry=%.4f solved=%.4f solvedEff=%.4f",
                     targetWorld.x, targetWorld.y, targetWorld.z, effW.x, effW.y, effW.z,
                     m_ikRig->pins().size(), m_ikFrozen ? 1 : 0, entryErr, solvedPinErr,
                     solvedEffErr);
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
    if (kIkTrace) {
        // Before the pin refinement: the governed pose's pin errors, lateral (XZ) and sink.
        float lat = 0.0f;
        float sink = 0.0f;
        for (const IkEffector& pin : m_ikRig->pins()) {
            const glm::vec3 p(m_poseGlobal[static_cast<std::size_t>(pin.node)][3]);
            lat = std::max(lat, glm::length(glm::vec2(p.x - pin.target.x, p.z - pin.target.z)));
            sink = std::max(sink, pin.target.y - p.y);
        }
        std::fprintf(stderr, " preLat=%.4f preSink=%.4f", lat, sink);
    }
    refinePins(false, &target); // the APPLIED pose is what the user sees: user pins AND the
                                // grabbed joint exact (see armature.h)
    if (kIkTrace) {
        float appliedPinErr = 0.0f;
        float lowestPinY = 1e30f;
        for (const IkEffector& pin : m_ikRig->pins()) {
            const glm::vec3 p(m_poseGlobal[static_cast<std::size_t>(pin.node)][3]);
            appliedPinErr = std::max(appliedPinErr, glm::length(p - pin.target));
            lowestPinY = std::min(lowestPinY, p.y - pin.target.y);
        }
        std::fprintf(stderr, " applied=%.4f pinDy=%.4f s=%.3f\n", appliedPinErr, lowestPinY, s);
        // Per-joint solver-vs-applied residual along each pin's chain and the effector's (mm):
        // where the extraction loses the solver's configuration.
        const auto chainResidual = [&](int node, int levels) {
            for (int cur = node, k = 0; cur >= 0 && k < levels;
                 cur = m_bones[static_cast<std::size_t>(cur)].parent, ++k) {
                const glm::vec3 applied(m_poseGlobal[static_cast<std::size_t>(cur)][3]);
                std::fprintf(stderr, " %s:%.1f", m_boneNames[static_cast<std::size_t>(cur)].c_str(),
                             glm::length(applied - positions[static_cast<std::size_t>(cur)]) * 1000.0f);
            }
        };
        std::fprintf(stderr, "[ik]   residual eff");
        chainResidual(m_ikRig->dragEffector(), 5);
        for (const IkEffector& pin : m_ikRig->pins()) {
            std::fprintf(stderr, " | pin");
            chainResidual(pin.node, 4);
        }
        std::fprintf(stderr, "\n");
    }
    // LIVE CONTACT RE-DETECTION (see IkRig::updateContacts): the APPLIED pose is what touches
    // the floor. A joint that came down onto it this tick is a support from the next solve on
    // (the hands in a deep crouch, a knee coming down); one the SOLVER could not hold on its
    // spot — the body pulling it off — is let go again.
    {
        std::vector<glm::vec3> applied(m_bones.size());
        for (std::size_t i = 0; i < m_bones.size(); ++i) {
            applied[i] = glm::vec3(m_poseGlobal[i][3]);
        }
        if (m_ikRig->updateContacts(applied, positions) && kIkTrace) {
            std::fprintf(stderr, "[ik] contacts now:");
            for (const IkEffector& pin : m_ikRig->pins()) {
                std::fprintf(stderr, " %s(y=%.3f)",
                             m_boneNames[static_cast<std::size_t>(pin.node)].c_str(), pin.target.y);
            }
            std::fprintf(stderr, "\n");
        }
    }
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
    // Track the still-cursor error trend for the settle-freeze (see armature.h): collect worst-goal-
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

bool Armature::settleIkTick() {
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
        m_ikSettleStrain = errBefore; // see armature.h: the hold bound scales with this
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
    // s <= kIkRelaxation < 1 always: the under-relaxed fraction is applied unconditionally.
    const float s = std::min(kIkRelaxation, (worst > 1e-6f) ? cap / worst : 1.0f);
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

void Armature::endIkDrag() {
    if (m_ikRig) {
        m_ikRig->endDrag();
    }
}

void Armature::holdNudgedBoneThroughDrag(int bone) {
    if (!m_ikRig || !m_ikRig->dragActive() || bone < 0 ||
        bone >= static_cast<int>(m_bones.size())) {
        return;
    }
    if (m_ikRotPriorExempt.size() != m_bones.size()) {
        m_ikRotPriorExempt.assign(m_bones.size(), 0);
    }
    m_ikRotPriorExempt[static_cast<std::size_t>(bone)] = 1;
    // A promoted grab (a finger driving the hand): the offset the window's targets are shifted
    // by is (grabbed - solved) in the effector's DRAG-START frame (see dragIkTo), so re-capture
    // it from the current positions through the effector's rotation since drag start.
    const int eff = m_ikRig->dragEffector();
    if (eff >= 0 && eff != m_selectedBone && eff < static_cast<int>(m_bones.size())) {
        const glm::mat3 grabRot =
            m_ikRig->effectorIsTrunk()
                ? glm::mat3(1.0f)
                : glm::mat3(m_poseGlobal[static_cast<std::size_t>(eff)]) *
                      glm::transpose(m_ikGrabRotStart);
        m_ikGrabOffset =
            glm::transpose(grabRot) *
            (glm::vec3(m_poseGlobal[static_cast<std::size_t>(m_selectedBone)][3]) -
             glm::vec3(m_poseGlobal[static_cast<std::size_t>(eff)][3]));
    }
}

bool Armature::togglePinSelectedBone() {
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

bool Armature::hasPinnedBones() const {
    for (const char p : m_bonePinned) {
        if (p) {
            return true;
        }
    }
    return false;
}

void Armature::unpinAllBones() {
    std::fill(m_bonePinned.begin(), m_bonePinned.end(), 0);
}

std::vector<int> Armature::activeContactPins() const {
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
