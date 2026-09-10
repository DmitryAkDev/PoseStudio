/**
 * @file armatureikextract.cpp
 * @brief Armature::applyIkSolution — the ROTATION EXTRACTION: converts an FBIK solve's joint
 *        positions back into the engine's pose (per-channel Euler rotations clamped to the
 *        authored limits, plus the solve root's pose translation).
 *
 * The walk is parents-first through the anatomical hierarchy so every bone is fitted against its
 * parent's already-clamped frame. Load-bearing details, each traced to a real failure: the solve
 * root is translation-only (its rotation is never re-fit — its tiny child offsets amplify noise
 * into whole-body swings); aim targets look THROUGH rigid twist-bone links to the first joint
 * whose position the rotation actually places; a single-aim-child bone takes its twist from the
 * TWIST WITNESS (the plane its hinge grandchild was bent in, blended by the bend that draws it);
 * the per-joint angular cap and the rotational prior bound per-tick change and clean undetermined
 * drift; and planted feet keep their drag-start world orientation while their pin holds. The
 * governor constants come from armatureik_detail.h. Vulkan-free, Qt-free.
 */

#include "armature.h"

#include "armatureik_detail.h" // kIkMaxJointDeltaDeg / kIkRotationPrior / wrappedAngleDelta
#include "ikconstraints.h"     // dominantBendTangent / fitTwistToDirection (the twist witness)
#include "ikmath.h"            // shortestArc / signedAngleAround / eulerFromMatrix
#include "ikrig.h"             // edgeRigid / edgeConstraint / rootNode / steppingPin

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace pose {

void Armature::applyIkSolution(const std::vector<glm::vec3>& solved, const std::vector<char>& active,
                            bool rotationPrior) {
    if (solved.size() != m_bones.size() || active.size() != m_bones.size()) {
        return;
    }
    // Walk parents-before-children (the bone order guarantees it), fitting each active bone's
    // world rotation to where the solve put its active children, so descendants extract against
    // their parent's ALREADY-CLAMPED frame and constraint error never compounds down a limb.
    const int rigRoot = m_ikRig->rootNode(); // callers (dragIkTo / settleIkTick) hold a live rig
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        Bone& bone = m_bones[i];
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
            for (const int c : m_children[i]) {
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
                for (const int c : m_children[static_cast<std::size_t>(aimTarget)]) {
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
                    for (const int c : m_children[i]) {
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
                    if (twistSamples == 0 && primary >= 0) {
                        int   grand = -1;
                        float grandLen = 0.02f;
                        for (const int c : m_children[static_cast<std::size_t>(primary)]) {
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

} // namespace pose
