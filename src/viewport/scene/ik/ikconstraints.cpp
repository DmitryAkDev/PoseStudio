/**
 * @file ikconstraints.cpp
 * @brief Position-level joint constraints for the FABRIK solver: deriving a segment's hinge /
 *        asymmetric-cone / locked constraint from its joint's authored per-axis Euler limits,
 *        clamping a proposed segment direction to it, and the twist machinery (the solve-side
 *        fold-plane search and its exact inverse, the extraction's twist witness).
 *
 * The derivation classifies the oriented axis most parallel to the child segment as twist and
 * the other two as swing: one free swing axis is a hinge (knee, elbow, finger), two authored
 * ranges make an ASYMMETRIC cone (a symmetric aperture let limbs fold the impossible way), both
 * locked is a rigid twist-bone pass-through. The clamp carries the STRAIGHT-LIMB ESCAPE (a
 * collinear chain can never decide a bend direction; the caller pushes a stalled one-sided joint
 * deeper into its dominant side, exploratorily) and clamps a hinge to its CIRCULARLY nearest
 * bound (a deep valid bend proposed past 180 degrees must not snap to the hyperextension side).
 * With twist freedom the whole joint frame may rotate about the parent segment within the
 * authored range: searched numerically (a coarse sweep, then a ternary refine), PREFERRING the
 * twist the current pose already carries so a reachable direction never flips its fold plane
 * from one solve to the next. Qt-free (std + GLM).
 */
#include "ikconstraints.h"

#include "ikmath.h"

#include <algorithm>
#include <cmath>

namespace pose {

namespace {

// An axis whose authored range is this small is "locked" for classification purposes (the format
// uses hard [0,0] clamps for forbidden axes; a couple of degrees of slop tolerates near-locks).
constexpr float kLockedRangeDegrees = 2.0f;

// Below this projected length a direction is treated as parallel to the hinge axis / degenerate.
constexpr float kDegenerate = 1e-5f;

} // namespace

JointConstraint deriveJointConstraint(const glm::mat3& orientAxes, const glm::vec3& restChildDir,
                                      const glm::vec3& rotMinDeg, const glm::vec3& rotMaxDeg,
                                      const glm::bvec3& rotLimited) {
    bool  locked[3];
    bool  limited[3];
    float rangeDeg[3];
    for (int a = 0; a < 3; ++a) {
        limited[a] = rotLimited[a];
        rangeDeg[a] = limited[a] ? (rotMaxDeg[a] - rotMinDeg[a]) : 360.0f;
        locked[a] = limited[a] && rangeDeg[a] < kLockedRangeDegrees;
    }

    // Twist axis = the oriented axis most parallel to the child segment (twist spins the segment
    // about itself without moving the child position — irrelevant to a positional constraint).
    int twist = 0;
    float bestDot = -1.0f;
    for (int a = 0; a < 3; ++a) {
        const float d = std::abs(glm::dot(glm::normalize(orientAxes[a]), restChildDir));
        if (d > bestDot) {
            bestDot = d;
            twist = a;
        }
    }
    const int s0 = (twist + 1) % 3;
    const int s1 = (twist + 2) % 3;

    JointConstraint out;
    if (locked[s0] && locked[s1]) {
        // Both swing axes forbidden (mid-limb twist bones): the segment can't leave its rest
        // direction at all — a zero-aperture cone.
        out.type = JointConstraint::Type::Cone;
        out.coneHalfAngle = 0.0f;
    } else if (locked[s0] || locked[s1]) {
        // One swing axis free: a hinge about it (knees, elbows, fingers).
        const int h = locked[s0] ? s1 : s0;
        out.type = JointConstraint::Type::Hinge;
        out.hingeAxis = glm::normalize(orientAxes[h]);
        out.minAngle = glm::radians(limited[h] ? rotMinDeg[h] : -180.0f);
        out.maxAngle = glm::radians(limited[h] ? rotMaxDeg[h] : 180.0f);
    } else if (!limited[s0] && !limited[s1]) {
        out.type = JointConstraint::Type::Free;
    } else {
        // Two swing axes with authored ranges: an ASYMMETRIC cone — the swing decomposes onto the
        // two oriented axes, each clamped to its own signed range (a real joint's reach is wildly
        // lopsided, and folding it to a symmetric aperture let limbs fold the impossible way).
        out.type = JointConstraint::Type::Cone;
        out.perAxis = true;
        out.swingAxis0 = glm::normalize(orientAxes[s0]);
        out.swingAxis1 = glm::normalize(orientAxes[s1]);
        out.swing0Min = glm::radians(limited[s0] ? rotMinDeg[s0] : -180.0f);
        out.swing0Max = glm::radians(limited[s0] ? rotMaxDeg[s0] : 180.0f);
        out.swing1Min = glm::radians(limited[s1] ? rotMinDeg[s1] : -180.0f);
        out.swing1Max = glm::radians(limited[s1] ? rotMaxDeg[s1] : 180.0f);
        // coneHalfAngle is left at its (never-consulted) default while perAxis is set — the
        // isotropic path and IkRig::edgeRigid only read it for non-perAxis cones.
    }
    return out;
}

namespace {

// The clamp proper, in one fixed frame (no twist freedom): see constrainSegmentDirection.
glm::vec3 constrainInFrame(const JointConstraint& constraint, const glm::quat& frame,
                           const glm::vec3& restDir, const glm::vec3& dir, float straightBias);

} // namespace

glm::vec3 constrainSegmentDirection(const JointConstraint& constraint, const glm::quat& frame,
                                    const glm::vec3& restDir, const glm::vec3& dir,
                                    float straightBias, const glm::quat* seedFrame) {
    if (constraint.type == JointConstraint::Type::Free) {
        return dir;
    }
    if (constraint.twistMax - constraint.twistMin <= 1e-6f) {
        return constrainInFrame(constraint, frame, restDir, dir, straightBias);
    }
    // TWIST FREEDOM: the whole joint frame (rest direction, hinge/swing axes) may rotate about
    // the parent segment's axis within the authored twist range — the fold plane swings to meet
    // the proposed direction. Searched numerically (a coarse sweep, then a ternary refine of the
    // best bracket): the residual as a function of twist is smooth but flat wherever the
    // direction is reachable, and ties resolve toward the LEAST twist (zero is evaluated first
    // and replaced only on strict improvement) so a reachable target never buys gratuitous
    // twist. ~20 clamp evaluations per placement, on a handful of edges: negligible next to the
    // solve.
    const glm::vec3 u = glm::normalize(frame * constraint.twistAxis);
    // The preferred twist: what the current pose already carries (see the header). The bend
    // axis is the hinge axis or the cone's dominant swing axis; its azimuth about u in the
    // chained frame vs the seed frame is the twist the chain dropped.
    float phiPref = 0.0f;
    if (seedFrame != nullptr) {
        glm::vec3 bendAxis = constraint.hingeAxis;
        if (constraint.type == JointConstraint::Type::Cone) {
            const float reach0 = std::max(-constraint.swing0Min, constraint.swing0Max);
            const float reach1 = std::max(-constraint.swing1Min, constraint.swing1Max);
            bendAxis = reach0 >= reach1 ? constraint.swingAxis0 : constraint.swingAxis1;
        }
        glm::vec3 hChain = frame * bendAxis;
        glm::vec3 hSeed = (*seedFrame) * bendAxis;
        hChain -= u * glm::dot(hChain, u);
        hSeed -= u * glm::dot(hSeed, u);
        if (glm::dot(hChain, hChain) > 0.01f && glm::dot(hSeed, hSeed) > 0.01f) {
            phiPref = glm::clamp(signedAngleAround(glm::normalize(hChain), glm::normalize(hSeed), u),
                                 constraint.twistMin, constraint.twistMax);
        }
    }
    constexpr float kContinuity = 1e-4f; // per radian of twist away from the preferred one
    const auto evalAt = [&](float phi, glm::vec3& out) {
        const glm::quat twisted = glm::normalize(glm::angleAxis(phi, u) * frame);
        out = constrainInFrame(constraint, twisted, restDir, dir, straightBias);
        return glm::dot(out, dir) - kContinuity * std::abs(phi - phiPref);
    };
    glm::vec3 best;
    float bestScore = evalAt(phiPref, best);
    float bestPhi = phiPref;
    constexpr int kSamples = 8;
    const float span = constraint.twistMax - constraint.twistMin;
    for (int i = 0; i < kSamples; ++i) {
        const float phi = constraint.twistMin + span * (static_cast<float>(i) + 0.5f) /
                                                    static_cast<float>(kSamples);
        glm::vec3 out;
        const float score = evalAt(phi, out);
        if (score > bestScore + 1e-6f) {
            bestScore = score;
            best = out;
            bestPhi = phi;
        }
    }
    const float step = span / static_cast<float>(kSamples);
    float lo = std::max(constraint.twistMin, bestPhi - step);
    float hi = std::min(constraint.twistMax, bestPhi + step);
    for (int k = 0; k < 7; ++k) {
        const float m1 = lo + (hi - lo) / 3.0f;
        const float m2 = hi - (hi - lo) / 3.0f;
        glm::vec3 o1;
        glm::vec3 o2;
        const float s1 = evalAt(m1, o1);
        const float s2 = evalAt(m2, o2);
        if (s1 >= s2) {
            hi = m2;
            if (s1 > bestScore + 1e-6f) {
                bestScore = s1;
                best = o1;
            }
        } else {
            lo = m1;
            if (s2 > bestScore + 1e-6f) {
                bestScore = s2;
                best = o2;
            }
        }
    }
    return best;
}

float fitTwistToDirection(const JointConstraint& constraint, const glm::quat& frame,
                          const glm::vec3& restDir, const glm::vec3& dir) {
    if (constraint.type == JointConstraint::Type::Free ||
        constraint.twistMax - constraint.twistMin <= 1e-6f) {
        return 0.0f;
    }
    const glm::vec3 u = glm::normalize(frame * constraint.twistAxis);
    constexpr float kContinuity = 1e-4f;
    const auto scoreAt = [&](float phi) {
        const glm::quat twisted = glm::normalize(glm::angleAxis(phi, u) * frame);
        const glm::vec3 out = constrainInFrame(constraint, twisted, restDir, dir, 0.0f);
        return glm::dot(out, dir) - kContinuity * std::abs(phi);
    };
    constexpr float kPi = 3.14159265f;
    constexpr int   kSamples = 12;
    float bestPhi = 0.0f;
    float bestScore = scoreAt(0.0f);
    for (int i = 0; i < kSamples; ++i) {
        const float phi = -kPi + 2.0f * kPi * (static_cast<float>(i) + 0.5f) /
                                    static_cast<float>(kSamples);
        const float score = scoreAt(phi);
        if (score > bestScore + 1e-6f) {
            bestScore = score;
            bestPhi = phi;
        }
    }
    const float step = 2.0f * kPi / static_cast<float>(kSamples);
    float lo = bestPhi - step;
    float hi = bestPhi + step;
    for (int k = 0; k < 9; ++k) {
        const float m1 = lo + (hi - lo) / 3.0f;
        const float m2 = hi - (hi - lo) / 3.0f;
        const float s1 = scoreAt(m1);
        const float s2 = scoreAt(m2);
        if (s1 >= s2) {
            hi = m2;
            if (s1 > bestScore + 1e-6f) {
                bestScore = s1;
                bestPhi = m1;
            }
        } else {
            lo = m1;
            if (s2 > bestScore + 1e-6f) {
                bestScore = s2;
                bestPhi = m2;
            }
        }
    }
    return bestPhi;
}

bool bendIsHingeLike(const JointConstraint& constraint) {
    if (constraint.type == JointConstraint::Type::Hinge) {
        return true;
    }
    if (constraint.type != JointConstraint::Type::Cone || !constraint.perAxis) {
        return false;
    }
    const float reach0 = std::max(-constraint.swing0Min, constraint.swing0Max);
    const float reach1 = std::max(-constraint.swing1Min, constraint.swing1Max);
    return std::min(reach0, reach1) < 0.2617994f && std::max(reach0, reach1) >= 0.34906585f;
}

bool dominantBendTangent(const JointConstraint& constraint, const glm::vec3& restDir,
                         glm::vec3& out) {
    if (!bendIsHingeLike(constraint)) {
        return false;
    }
    glm::vec3 axis(0.0f);
    float sign = 1.0f;
    if (constraint.type == JointConstraint::Type::Hinge) {
        axis = constraint.hingeAxis;
        sign = (constraint.maxAngle > -constraint.minAngle) ? 1.0f : -1.0f;
    } else if (constraint.type == JointConstraint::Type::Cone && constraint.perAxis) {
        const float reach0 = std::max(-constraint.swing0Min, constraint.swing0Max);
        const float reach1 = std::max(-constraint.swing1Min, constraint.swing1Max);
        if (std::max(reach0, reach1) < 0.34906585f) { // 20 degrees: no real bend to witness
            return false;
        }
        if (reach0 >= reach1) {
            axis = constraint.swingAxis0;
            sign = (constraint.swing0Max > -constraint.swing0Min) ? 1.0f : -1.0f;
        } else {
            axis = constraint.swingAxis1;
            sign = (constraint.swing1Max > -constraint.swing1Min) ? 1.0f : -1.0f;
        }
    } else {
        return false;
    }
    const glm::vec3 t = glm::cross(axis, restDir) * sign; // angleAxis(+a, axis) moves rest this way
    const float len = glm::length(t);
    if (len < 1e-4f) {
        return false;
    }
    out = t / len;
    return true;
}

namespace {

glm::vec3 constrainInFrame(const JointConstraint& constraint, const glm::quat& frame,
                           const glm::vec3& restDir, const glm::vec3& dir, float straightBias) {
    const glm::vec3 rest = frame * restDir; // the segment's rest direction in the current frame

    if (constraint.type == JointConstraint::Type::Cone) {
        if (constraint.perAxis) {
            // Asymmetric cone: decompose the swing carrying rest -> dir into its components about
            // the two oriented swing axes and clamp each to its own signed range. The swing's
            // rotation axis is perpendicular to rest, so it has no twist component to lose.
            const float d = glm::clamp(glm::dot(rest, dir), -1.0f, 1.0f);
            glm::vec3 w(0.0f); // rotation vector (axis * angle) of the proposed swing
            if (d < 1.0f - 1e-7f) {
                glm::vec3 axis = glm::cross(rest, dir);
                const float axisLen = glm::length(axis);
                axis = (axisLen > kDegenerate) ? axis / axisLen
                                               : frame * constraint.swingAxis0; // antiparallel
                w = axis * std::acos(d);
            }
            const glm::vec3 a0 = frame * constraint.swingAxis0;
            const glm::vec3 a1 = frame * constraint.swingAxis1;
            float w0 = glm::dot(w, a0);
            float w1 = glm::dot(w, a1);
            if (straightBias > 0.0f) {
                // Straight-limb escape for ball joints (same policy as the hinge push below): a
                // collinear chain can only fold if SOME joint proposes a bend — push the swing
                // deeper along the dominant side of whichever swing axis has the larger reach
                // (a thigh's big forward-kick range), stall-gated and best-kept by the caller.
                const float reach0 = std::max(-constraint.swing0Min, constraint.swing0Max);
                const float reach1 = std::max(-constraint.swing1Min, constraint.swing1Max);
                if (std::max(reach0, reach1) > glm::radians(30.0f)) {
                    if (reach0 >= reach1) {
                        w0 += (constraint.swing0Max > -constraint.swing0Min) ? straightBias
                                                                             : -straightBias;
                    } else {
                        w1 += (constraint.swing1Max > -constraint.swing1Min) ? straightBias
                                                                             : -straightBias;
                    }
                }
            }
            w0 = glm::clamp(w0, constraint.swing0Min, constraint.swing0Max);
            w1 = glm::clamp(w1, constraint.swing1Min, constraint.swing1Max);
            const glm::vec3 clamped = a0 * w0 + a1 * w1;
            const float angle = glm::length(clamped);
            return angle > kDegenerate ? glm::angleAxis(angle, clamped / angle) * rest : rest;
        }
        if (constraint.coneHalfAngle <= kDegenerate) {
            return rest; // locked segment: exactly the (frame-carried) rest direction
        }
        const float d = glm::clamp(glm::dot(rest, dir), -1.0f, 1.0f);
        const float angle = std::acos(d);
        if (angle <= constraint.coneHalfAngle) {
            return dir;
        }
        const glm::vec3 axis = glm::cross(rest, dir);
        if (glm::dot(axis, axis) < kDegenerate * kDegenerate) {
            return rest; // antiparallel with no defined plane: snap back to rest
        }
        // Rotate rest toward dir, stopping at the cone surface.
        return glm::angleAxis(constraint.coneHalfAngle, glm::normalize(axis)) * rest;
    }

    // Hinge: preserve the direction's component along the axis equal to rest's (a rotation about
    // the axis can't change it), and clamp the swing angle in the perpendicular plane.
    const glm::vec3 axis = frame * constraint.hingeAxis;
    const float restAlong = glm::dot(rest, axis);
    glm::vec3 restPerp = rest - axis * restAlong;
    glm::vec3 dirPerp = dir - axis * glm::dot(dir, axis);
    const float restPerpLen = glm::length(restPerp);
    const float dirPerpLen = glm::length(dirPerp);
    if (restPerpLen < kDegenerate || dirPerpLen < kDegenerate) {
        return rest; // segment (nearly) parallel to the hinge axis: no defined swing plane
    }
    restPerp /= restPerpLen;
    dirPerp /= dirPerpLen;
    float angle = std::atan2(glm::dot(glm::cross(restPerp, dirPerp), axis),
                             glm::dot(restPerp, dirPerp));
    if (straightBias > 0.0f) {
        // Straight-limb escape: push a one-sided hinge (knee: x in [-11, 155]) DEEPER into its
        // dominant side by the bias step. The caller fires this only when the solve stalled (the
        // straight-lock signature) and keeps its best state, so the push is exploratory: it
        // deepens a fold that positional passes alone can never initiate (a collinear chain has
        // no bend direction) and is discarded when it doesn't help. Incremental — not a floor —
        // because a crouch needs the knee driven progressively deeper, not parked at one angle.
        const bool deepSideIsMax = constraint.maxAngle > -constraint.minAngle;
        const float oneSided = std::max(constraint.maxAngle, -constraint.minAngle);
        if (oneSided > glm::radians(30.0f)) {
            angle += deepSideIsMax ? straightBias : -straightBias;
        }
    }
    if (angle < constraint.minAngle || angle > constraint.maxAngle) {
        // Clamp to the CIRCULARLY nearest bound: atan2 wraps to (-180, 180], so a deep valid bend
        // proposed past 180 (e.g. 200 = -160) must clamp to the 155 side, not snap to -11.
        auto circularDist = [](float x, float bound) {
            float d = std::abs(x - bound);
            constexpr float kTwoPi = 6.2831853f;
            while (d > kTwoPi * 0.5f) d = std::abs(d - kTwoPi);
            return d;
        };
        angle = circularDist(angle, constraint.minAngle) < circularDist(angle, constraint.maxAngle)
                    ? constraint.minAngle
                    : constraint.maxAngle;
    }
    const glm::vec3 swung = glm::angleAxis(angle, axis) * restPerp;
    return glm::normalize(axis * restAlong + swung * restPerpLen);
}

} // namespace

} // namespace pose
