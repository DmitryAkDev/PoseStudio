/**
 * @file ikconstraints.h
 * @brief Joint rotation constraints for the FBIK solver: 1-DoF hinges and 3-DoF cones (step 3).
 *
 * The FABRIK solver works on joint POSITIONS, so constraints are expressed as limits on the
 * DIRECTION of a bone segment relative to its rest direction: a hinge confines the segment to a
 * plane and a signed angle range about the hinge axis (knees, elbows); a cone caps the segment's
 * deviation from its rest direction (ball joints — hips, shoulders, spine). Each joint's
 * constraint is DERIVED from the figure's own per-axis Euler limits (the same `rotation/{x,y,z}`
 * min/max/locked data the engine already enforces): the oriented axis most parallel to the child
 * segment is the twist axis, the other two are swing — one free swing axis makes a hinge, two make
 * a cone, none locks the segment. Twist ranges aren't modeled at position level; the engine's
 * per-axis Euler clamp (Model::clampBoneEuler) remains the authoritative constraint pass after
 * rotation extraction — these position-level limits keep FABRIK from ever PROPOSING a broken pose
 * (a knee folding backwards) so the extraction has something anatomical to fit.
 *
 * A load-bearing simplification: the figure format's bind transforms are translation-only (every
 * bone's rest frame is axis-aligned with model space — orientation lives separately in `orient`
 * and cancels at rest), so rest directions and oriented axes for EVERY joint live in one common
 * model-space frame — no per-bone frame conversion is needed when deriving constraints, and the
 * per-node frames the solver accumulates carry them into the posed configuration directly.
 * Qt-free (std + GLM).
 */

#ifndef IKCONSTRAINTS_H
#define IKCONSTRAINTS_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace pose {

/// Positional rotation limit of one bone segment (the edge from a joint to its anatomical child).
struct JointConstraint {
    enum class Type {
        Free,  ///< Unconstrained ball joint (no authored limits).
        Hinge, ///< 1 DoF: segment swings about hingeAxis only (its component along the axis is
               ///< preserved), within [minAngle, maxAngle] of the rest direction (signed, RH).
        Cone,  ///< 3 DoF: segment deviation from the rest direction capped. With perAxis set the
               ///< cone is ASYMMETRIC: the swing decomposes onto the two oriented swing axes and
               ///< each clamps to its own authored [min,max] — real joints are nothing like
               ///< symmetric (a thigh kicks far forward but barely backward), and the symmetric
               ///< aperture let FABRIK fold limbs through anatomically absurd configurations
               ///< (a thigh folded UP through the hip during a crouch). coneHalfAngle remains the
               ///< isotropic fallback when perAxis is false (halfAngle 0 locks the segment).
    };
    Type      type = Type::Free;
    glm::vec3 hingeAxis{0.0f, 0.0f, 1.0f}; ///< Rest/model space (bind frames are axis-aligned).
    float     minAngle = -3.14159265f;     ///< Hinge range about hingeAxis, radians, 0 = rest.
    float     maxAngle = 3.14159265f;
    float     coneHalfAngle = 3.14159265f; ///< Max deviation from the rest direction, radians.
    // Asymmetric cone (perAxis): per-swing-axis signed ranges, radians, 0 = rest.
    bool      perAxis = false;
    glm::vec3 swingAxis0{1.0f, 0.0f, 0.0f};
    glm::vec3 swingAxis1{0.0f, 0.0f, 1.0f};
    float     swing0Min = -3.14159265f, swing0Max = 3.14159265f;
    float     swing1Min = -3.14159265f, swing1Max = 3.14159265f;
    // TWIST FREEDOM (hinges and asymmetric cones): the bend plane may rotate about twistAxis —
    // the PARENT segment's rest direction (unit, rest space) — within [twistMin, twistMax]
    // (radians about +twistAxis; equal = none), the authored twist range of the joint that owns
    // that segment: a mid-limb twist bone (the upper arm's twist swings the elbow's fold plane,
    // the thigh's the knee's). Without it the solver carried no twist at all — its frames chain
    // swing-only from the root — so a limb could fold only in the plane its drag-start twist
    // left it in: a hand pulled in front of the chest stopped 10cm short of a plainly reachable
    // point, and no damping or prior tuning could touch that (IkRig::build derives it; the
    // Model's extraction realizes the chosen twist through the TWIST WITNESS, see
    // dominantBendTangent).
    glm::vec3 twistAxis{0.0f, 1.0f, 0.0f};
    float     twistMin = 0.0f;
    float     twistMax = 0.0f;
};

/// The direction (unit, rest space) in which @p constraint's segment moves when its joint bends
/// from rest toward its DOMINANT side — the bend plane's tangent at rest (a hinge's, or an
/// asymmetric cone's wider swing axis). False for constraints without a dominant bend (free,
/// isotropic, locked, or a cone whose reach is under 20°). The rotation extraction uses it as a
/// TWIST WITNESS: the grandchild's solved bend direction, compared against this tangent about the
/// parent segment, tells the parent twist bone how far to twist so the fold plane lands where the
/// solve put it — the one component a single aim child leaves unwitnessed.
bool dominantBendTangent(const JointConstraint& constraint, const glm::vec3& restDir,
                         glm::vec3& out);

/// The twist (radians about frame * twistAxis, RELATIVE to @p frame) at which @p constraint's
/// clamp brings @p dir closest — the exact inverse of the solve's twist search, for the rotation
/// extraction: given the direction the solve bent the segment in, how far its parent must twist
/// from the fitted frame for the bend to land there. A full turn is searched (the bend's minor
/// side is a half-turn away), preferring the least change. 0 when the constraint has no twist
/// freedom.
float fitTwistToDirection(const JointConstraint& constraint, const glm::quat& frame,
                          const glm::vec3& restDir, const glm::vec3& dir);

/// True when @p constraint bends essentially in ONE plane: a hinge, or an asymmetric cone whose
/// minor swing axis reaches under 15° (a knee's few degrees of lateral play). Twist freedom and
/// the twist witness apply to these joints ONLY — a 2-DoF ball joint (a shoulder, the spine)
/// chooses its own bend azimuth through its second swing axis, so its "fold plane" is no
/// witness of the parent's twist, and searching twist there only enlarged the solution space
/// into churn (the collar drove to its limit, the hand missed its own rest position by 5cm).
bool bendIsHingeLike(const JointConstraint& constraint);

/// Derives one segment's constraint from its parent joint's authored per-axis Euler limits.
/// @param orientAxes   The joint's oriented rotation axes (columns; rest/model space) — the frame
///                     its Euler channels rotate about (Model's `orient` rotation part).
/// @param restChildDir Rest direction of the segment (joint -> child, unit, rest/model space);
///                     classifies which oriented axis is twist (most parallel) vs swing.
/// @param rotMinDeg / rotMaxDeg / rotLimited  The joint's per-axis limits (degrees) and which axes
///                     carry them — the same data Model::clampBoneEuler enforces.
JointConstraint deriveJointConstraint(const glm::mat3& orientAxes, const glm::vec3& restChildDir,
                                      const glm::vec3& rotMinDeg, const glm::vec3& rotMaxDeg,
                                      const glm::bvec3& rotLimited);

/// Clamps proposed segment direction @p dir (unit) to @p constraint. @p frame carries rest space
/// into the current parent frame (accumulated along the solve traversal) and @p restDir is the
/// segment's rest direction (unit, rest space). @p straightBias, when > 0, nudges a one-sided
/// hinge (a knee/elbow, whose range extends far to one side only) at least that many radians into
/// its allowed side — FABRIK's escape hatch from the straight-limb singularity, where a fully
/// extended chain is collinear with its target and no positional pass can decide a bend direction.
/// @p seedFrame (optional) is the joint's ACTUAL current frame (the solve's frameSeed): with
/// twist freedom, the twist the current pose already carries — the angle between the chained
/// swing-only frame's bend axis and the seed's, about the twist axis — is the PREFERRED twist,
/// evaluated first and favoured by a tiny continuity penalty, so a reachable direction never
/// flips the fold plane between two equally-good twists from one solve to the next (the
/// extraction realizes whatever twist the solve chose, the next seed carries it back in).
/// Returns the clamped unit direction.
glm::vec3 constrainSegmentDirection(const JointConstraint& constraint, const glm::quat& frame,
                                    const glm::vec3& restDir, const glm::vec3& dir,
                                    float straightBias = 0.0f, const glm::quat* seedFrame = nullptr);

} // namespace pose

#endif // IKCONSTRAINTS_H
