/**
 * @file fabriksolver.h
 * @brief Multi-chain FABRIK solver over the re-rootable skeleton graph (step 2 of the FBIK
 *        system): branching chains, multiple pinned effectors, and per-segment joint constraints.
 *
 * FABRIK (Forward And Backward Reaching IK, Aristidou & Lasenby) iterates two positional passes:
 * a FORWARD pass from the leaves toward the root — each effector snaps to its target, then every
 * joint is re-placed at bone-length from its (already-moved) children, a branching joint taking
 * the centroid of its children's proposals (the multi-chain sub-base rule) — and a BACKWARD pass
 * from the root out — every child re-placed at bone-length from its parent. The graph is rooted
 * at the PELVIS and the root FLOATS: ground anchoring is the PINNED EFFECTORS' job (the planted
 * feet), which hold the root through their chains while per-pin reach LEASHES confine it to the
 * intersection of the limbs' reach balls (so a drag can never hoist the body off its planted
 * feet). A pinned effector ON the root (an explicit pelvis drag, or the airborne-figure
 * fallback) snaps the root instead. Bone lengths are preserved by construction; iteration
 * converges the effectors onto their targets, or as close as reach allows. After the two main
 * passes, each effector gets a single-chain RESTORATION pass (see the .cpp) that removes the
 * residual the sub-base centroid compromise leaves at every goal.
 *
 * Joint constraints (ikconstraints.h) are applied during the backward pass: a per-node rotation
 * frame is accumulated from the root outward (seeded with each bone's CURRENT pose rotation, so
 * an already-posed figure constrains about its posed frame, not its bind frame) and every
 * segment's proposed direction is clamped to its hinge/cone before placement.
 *
 * The solver only moves nodes marked active (the union of effector->root paths); everything else
 * is untouched and rides along rigidly when the Model re-poses. Qt-free (std + GLM).
 */

#ifndef FABRIKSOLVER_H
#define FABRIKSOLVER_H

#include "ikconstraints.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <vector>

namespace pose {

class SkeletonGraph;

/// One IK goal: drive @p node toward @p target. Pinned effectors (planted feet) are constraints
/// the solve must keep satisfied; the dragged joint is an ordinary unpinned goal.
struct IkEffector {
    int       node = -1;
    glm::vec3 target{0.0f};
    bool      pinned = false;
    float     weight = 1.0f;      ///< Pull strength (1 = snap): interior soft goals use less.
    float     leashRadius = -1.0f; ///< Pins only: max root distance from this pin (see solver).
};

/**
 * @class FabrikSolver
 * @brief Stateless multi-chain FABRIK: positions in, constrained positions out.
 */
class FabrikSolver {
public:
    struct Settings {
        int   maxIterations = 24;
        float tolerance = 5e-4f;                ///< Max effector error to converge (world units).
        float straightBias = 0.03490658503f;    ///< 2 deg: one-sided-hinge collinearity escape.
        /// Trust region: no joint may move farther than this from its entry position in ONE
        /// solve. The solver is called once per mouse EVENT, and without this cap any internal
        /// basin switch (a fold-escape kick engaging, restoration finding a new configuration)
        /// lands as a visible SNAP in a single event; capped, the same correction plays out
        /// smoothly across a few events (mouse events are dense, so catch-up is fast).
        float maxStepDisplacement = 0.05f;
        /// Scale on each pose-prior application, letting the caller raise maxIterations without
        /// stiffening the prior: the prior is applied PER ITERATION, so its per-solve pull
        /// compounds with the iteration count — raising iterations 6→10 unnormalized broke the
        /// hover-healing behavior by silently strengthening every stiffness. The rig passes
        /// (reference iterations / actual iterations).
        float priorIterationNorm = 1.0f;
        /// Yield factor for the ROOT's prior against DOWNWARD displacement (1 = no yield). The
        /// rig sets < 1 when the drag intent is downward (target below the grabbed joint): a
        /// body pushed down at the chest or pulled down by a hand should CROUCH — the root gives
        /// vertically, the legs fold onto the pinned feet — instead of the stiff root prior
        /// holding the pelvis at standing height. Lateral/upward root stiffness is unaffected
        /// (that stiffness is what prevents the swayback hip-slide).
        float rootDownYield = 1.0f;
        /// Downward bias (world units per iteration) applied to non-effector active nodes — the
        /// SUSPENSION gravity. When the figure hangs from the drag target (see IkRig's lift-off
        /// mode) this is what makes the free limbs and trunk settle vertically below the grab
        /// point, within their joint limits: dangling, without a physics engine.
        float gravityBias = 0.0f;
        /// Output DEADBAND (0 = off): an active node whose solved position ends within this of
        /// its entry position is snapped back to it exactly. The solve -> extract -> FK loop
        /// carries a sub-millimeter limit cycle (near-equal configurations alternating tick to
        /// tick) that under-relaxation damps but never kills — idle joints visibly SHIMMER
        /// while the user pulls. Sub-deadband proposals become bit-identical stillness; real
        /// corrections (multi-millimeter) pass untouched.
        float minStepDisplacement = 0.0f;
    };

    /// Solves @p positions (model space, per graph node) in place toward @p effectors, moving
    /// only nodes with @p active set. The graph MUST be rooted at the ANATOMICAL root (the
    /// rig's pelvis): the per-edge arrays are indexed by anatomical child, so an edge walked
    /// INVERTED under any other rooting would silently read the wrong bone's length, direction,
    /// and constraint (the abandoned contact-rooted design; SkeletonGraph still supports
    /// re-rooting for the rig's own bookkeeping, but never hand such a rooting to solve()). The rig roots the graph at
    /// the pelvis, so traversal order equals hierarchy order and every constraint is evaluated in
    /// its owning parent-side frame. Per-segment data is indexed by child bone (each non-root
    /// bone owns the edge to its parent): @p edgeRestDir[c] / @p edgeRestLength[c] = the
    /// segment's rest direction (unit, rest/model space) and length (bind data — morphs included
    /// — NOT measured from @p positions, which may arrive inconsistent when the root was placed
    /// directly), and @p edgeConstraint[c] its derived constraint. @p frameSeed is each bone's
    /// current rest->posed rotation (model space), seeding the constraint frames.
    ///
    /// @p posePrior (optional, with per-node @p priorWeights in (0,1]) is a SOFT restoring
    /// target per node — the drag-start pose. Redundant-body IK is ill-posed without it:
    /// interior joints (the spine during a hand drag) have no targets of their own, every
    /// forward pass drags them toward the effector, and nothing ever pulls them back — over the
    /// hundreds of per-mouse-event solves of one interactive drag the pose ratchets into
    /// grotesque contortions. The prior blends non-effector active nodes toward their start
    /// positions each iteration, making the solution the CLOSEST-TO-START pose that satisfies
    /// the targets and restoring the body as a drag returns. The weights are PER NODE — the
    /// rig's stiffness model: joints near the dragged effector move freely, the trunk and far
    /// limbs resist, so the dragged limb moves first and the body follows reluctantly, the way
    /// a real body recruits. Returns the worst effector error.
    static float solve(const SkeletonGraph& graph, const std::vector<char>& active,
                       const std::vector<IkEffector>& effectors,
                       const std::vector<glm::vec3>& edgeRestDir,
                       const std::vector<float>& edgeRestLength,
                       const std::vector<JointConstraint>& edgeConstraint,
                       const std::vector<glm::quat>& frameSeed, std::vector<glm::vec3>& positions,
                       const Settings& settings = {},
                       const std::vector<glm::vec3>* posePrior = nullptr,
                       const std::vector<float>* priorWeights = nullptr);
};

} // namespace pose

#endif // FABRIKSOLVER_H
