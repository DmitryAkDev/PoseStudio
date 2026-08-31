#include "ikconstraints.h"

#include "ikmath.h"

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

glm::vec3 constrainSegmentDirection(const JointConstraint& constraint, const glm::quat& frame,
                                    const glm::vec3& restDir, const glm::vec3& dir,
                                    float straightBias) {
    if (constraint.type == JointConstraint::Type::Free) {
        return dir;
    }
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

} // namespace pose
