#include "fabriksolver.h"

#include "ikmath.h"
#include "skeletongraph.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#ifdef IK_NAN_TRAP
namespace {
void nanTrap(const char* phase, int iter, const std::vector<glm::vec3>& positions) {
    for (std::size_t i = 0; i < positions.size(); ++i) {
        if (!std::isfinite(positions[i].x + positions[i].y + positions[i].z)) {
            std::printf("NAN TRAP: phase %s iter %d node %zu\n", phase, iter, i);
            std::fflush(stdout);
            std::abort();
        }
    }
}
} // namespace
#define IK_NAN_CHECK(phase, iter) nanTrap(phase, iter, positions)
#else
#define IK_NAN_CHECK(phase, iter)
#endif

namespace pose {

namespace {

constexpr float kZeroLength = 1e-6f; // edges shorter than this stay coincident (no direction)

// The straight-limb escape only fires for effectors at least this far from their targets. A
// STUCK chain (a crouch on locked-straight legs) has centimeters of error; a CONVERGED chain
// carries only millimeter noise (constraint clamps, the caller's extraction residual) that also
// reads as "stalled" — kicking on that noise ratcheted held poses deeper every solve.
constexpr float kKickErrorFloor = 0.025f;

// Effector weight at which a goal fully rules its node's position (the pose prior stops
// applying there). Weaker soft goals yield the prior only PARTIALLY — linear in weight — so a
// goal fading in from zero weight perturbs the no-goal solution continuously (see the prior
// application in solve()).
constexpr float kPriorYieldWeight = 0.5f;

} // namespace

float FabrikSolver::solve(const SkeletonGraph& graph, const std::vector<char>& active,
                          const std::vector<IkEffector>& effectors,
                          const std::vector<glm::vec3>& edgeRestDir,
                          const std::vector<float>& edgeRestLength,
                          const std::vector<JointConstraint>& edgeConstraint,
                          const std::vector<glm::quat>& frameSeed,
                          std::vector<glm::vec3>& positions, const Settings& settings,
                          const std::vector<glm::vec3>* posePrior,
                          const std::vector<float>* priorWeights) {
    const int n = graph.nodeCount();
    const int root = graph.root();
    if (n <= 0 || root < 0 || static_cast<int>(positions.size()) != n) {
        return 0.0f;
    }

    const std::vector<int>& order = graph.traversalOrder();

    // Per-node solve data under the current rooting: edge length to the traversal parent (from
    // the BIND data — morphs included — never measured from @p positions, which may arrive
    // inconsistent when the root was placed directly on a drag target), the active children, and
    // the effector driving each node (if any).
    std::vector<float> lengthToParent(static_cast<std::size_t>(n), 0.0f);
    std::vector<std::vector<int>> activeChildren(static_cast<std::size_t>(n));
    std::vector<int> effectorAt(static_cast<std::size_t>(n), -1);
    for (std::size_t e = 0; e < effectors.size(); ++e) {
        const int node = effectors[e].node;
        if (node >= 0 && node < n) {
            effectorAt[static_cast<std::size_t>(node)] = static_cast<int>(e);
        }
    }
    for (const int node : order) {
        const int p = graph.parentOf(node);
        if (p >= 0) {
            lengthToParent[static_cast<std::size_t>(node)] =
                edgeRestLength[static_cast<std::size_t>(node)];
            if (active[static_cast<std::size_t>(node)] && active[static_cast<std::size_t>(p)]) {
                activeChildren[static_cast<std::size_t>(p)].push_back(node);
            }
        }
    }

    // Frames start from the CURRENT pose rotations, not identity: the backward pass overwrites
    // every ACTIVE node's frame from its placed swing, but an INACTIVE node's frame is read as-is
    // when a restoration chain anchors at it (a caller-frozen sub-base — the trunk during an
    // up-intent drag). Identity there evaluated the chain's first constraint in the bone's BIND
    // frame: with the chest pre-bent, the shoulder clamped toward anatomically wrong directions.
    std::vector<glm::quat> frame = frameSeed;

    // Reach leashes: with a FLOATING root, the drag goal must never hoist the body off its
    // planted contacts — anatomically, the hips can never be farther from a planted foot than
    // the leg is long. Each pinned effector contributes a ball (its chain length to the root)
    // that the root is confined to; without this, pulling a hand upward raised the pelvis beyond
    // leg reach, the feet could no longer touch their pins, and the figure ended up FLOATING.
    // A PINNED root (an explicit pelvis drag / the airborne fallback) is exempt: dragging the
    // hips up is allowed to lift the feet — that is the user's explicit intent.
    struct Leash {
        glm::vec3 target;
        float     maxReach;
    };
    std::vector<Leash> leashes;
    {
        const int rootEff = effectorAt[static_cast<std::size_t>(root)];
        const bool rootPinned =
            rootEff >= 0 && effectors[static_cast<std::size_t>(rootEff)].pinned;
        if (!rootPinned) {
            for (const IkEffector& e : effectors) {
                if (!e.pinned || e.node < 0 || e.node >= n ||
                    !active[static_cast<std::size_t>(e.node)]) {
                    continue;
                }
                if (e.leashRadius > 0.0f) {
                    // The rig supplies the radius: the DRAG-START root-to-pin distance (a stance
                    // the figure provably held), with fractional path-length headroom for
                    // bent-leg starts. The raw path-length SUM overcounts: the pelvis-to-hip-
                    // socket offset is lateral, not collinear with the leg, so a sum-based ball
                    // let the body lean until a foot became genuinely unreachable.
                    leashes.push_back({e.target, e.leashRadius});
                    continue;
                }
                float len = 0.0f;
                for (int cur = e.node; cur >= 0 && cur != root; cur = graph.parentOf(cur)) {
                    len += lengthToParent[static_cast<std::size_t>(cur)];
                }
                if (len > kZeroLength) {
                    leashes.push_back({e.target, 0.955f * len});
                }
            }
        }
    }

    // The rig roots the traversal at the ANATOMICAL root (the pelvis), so traversal order equals
    // hierarchy order and every constraint is evaluated in its owning (parent-side) frame. An
    // earlier design re-rooted the traversal at a ground contact and walked limbs INVERTED — but
    // a constraint frame accumulated from the child side skews by exactly the joints' own swings
    // (the lower thigh inherited the knee's bend and "kinked" at the rigid twist joint), and the
    // top-down restoration chains fought the bottom-up main pass every iteration. Ground
    // anchoring is the PINS' job now; the root floats (its solved displacement becomes the
    // Model's root pose translation) and is held by the planted limbs — or by a caller-provided
    // pinned root effector when nothing is planted.
    const int anatomicalRoot = root;

    // The root's ROTATION is never solved: nothing anatomical limits a root, so the solver would
    // happily pitch the pelvis through itself to absorb a crouch (torso riding along). Its edges
    // stay frozen at their CURRENT pose directions (frameSeed); folds go into the limbs, and
    // pelvis orientation remains an FK decision.

    // Constrained placement of anatomical-child `node` at bone-length from its (already-placed)
    // parent `p`, accumulating the child's rotation frame. Shared by the backward pass and the
    // per-pin restoration chains. @p bias is the straight-limb escape for this edge (see the
    // bias policy in the iteration loop).
    auto placeChild = [&](int node, int p, float bias) {
        const float boneLen = lengthToParent[static_cast<std::size_t>(node)];
        if (boneLen <= kZeroLength) {
            positions[static_cast<std::size_t>(node)] = positions[static_cast<std::size_t>(p)];
            frame[static_cast<std::size_t>(node)] = frame[static_cast<std::size_t>(p)];
            return;
        }
        const glm::vec3 restDir = edgeRestDir[static_cast<std::size_t>(node)];
        const glm::quat& parentFrame = frame[static_cast<std::size_t>(p)];
        const glm::vec3& parentPos = positions[static_cast<std::size_t>(p)];
        glm::vec3 dir = positions[static_cast<std::size_t>(node)] - parentPos;
        const float dirLen = glm::length(dir);
        dir = (dirLen > kZeroLength) ? dir / dirLen : parentFrame * restDir;

        glm::vec3 clamped;
        if (p == anatomicalRoot) {
            // Root-owned edge: frozen at its current pose direction (see the note above).
            clamped = frameSeed[static_cast<std::size_t>(p)] * restDir;
        } else {
            clamped = constrainSegmentDirection(edgeConstraint[static_cast<std::size_t>(node)],
                                                parentFrame, restDir, dir, bias);
        }
        positions[static_cast<std::size_t>(node)] = parentPos + clamped * boneLen;
        // Accumulate the child's frame from how far the segment actually swung from rest.
        // Renormalized: these products chain 15+ deep per pass over many iterations, and norm
        // drift is what once pushed shortestArc into its degenerate-input corner.
        frame[static_cast<std::size_t>(node)] =
            glm::normalize(shortestArc(parentFrame * restDir, clamped) * parentFrame);
#ifdef IK_NAN_TRAP
        {
            const glm::vec3& np = positions[static_cast<std::size_t>(node)];
            const glm::quat& nf = frame[static_cast<std::size_t>(node)];
            if (!std::isfinite(np.x + np.y + np.z) ||
                !std::isfinite(nf.w + nf.x + nf.y + nf.z)) {
                std::printf("NAN placeChild node %d p %d boneLen %g dirLen %g clamped(%g %g %g) "
                            "parentFrame(%g %g %g %g)\n",
                            node, p, boneLen, dirLen, clamped.x, clamped.y, clamped.z,
                            parentFrame.w, parentFrame.x, parentFrame.y, parentFrame.z);
                std::fflush(stdout);
                std::abort();
            }
        }
#endif
    };

    auto worstError = [&]() {
        float worst = 0.0f;
        for (const IkEffector& e : effectors) {
            // Inactive-node effectors are skipped everywhere else (leashes, restoration) because
            // the solve cannot move them; counting their error here made it a constant floor that
            // kept every iteration running and inflated the returned error past any freeze gate.
            if (e.node >= 0 && e.node < n && active[static_cast<std::size_t>(e.node)]) {
                worst = std::max(worst,
                                 glm::length(positions[static_cast<std::size_t>(e.node)] - e.target));
            }
        }
        return worst;
    };

    float error = worstError();
    const std::vector<glm::vec3> entryPositions = positions; // trust-region reference
    std::vector<int> chain;              // pin-restoration scratch
    std::vector<glm::vec3> chainBest;    // pin-restoration best-configuration scratch
    std::vector<glm::quat> chainBestFrame; // ... and its frames (restored together — see below)
    // Global best-state keeping: constrained greedy iteration is local and CAN wander — the
    // returned solution is the best state any iteration reached, so a solve never regresses.
    // Only POST-iteration states qualify: the entry state can satisfy every effector while being
    // edge-length INCONSISTENT (a directly-placed root with unmoved limbs), and only a backward
    // pass from the anchor re-establishes the lengths.
    std::vector<glm::vec3> bestPositions;
    float bestError = 1e30f;
    const auto kickFor = [&settings](float err) {
        return std::min(0.5f, settings.straightBias + 2.0f * err);
    };
    // At least one iteration always runs: even with every effector at its target the state can be
    // inconsistent (see the best-state note above) and needs one forward/backward reconciliation.
    for (int iter = 0;
         iter < settings.maxIterations && (iter == 0 || error > settings.tolerance); ++iter) {
        const bool earlyPhase = iter < settings.maxIterations - 4;
        // --- Forward pass (leaves -> root): reach for the targets. ---
        for (std::size_t idx = order.size(); idx-- > 0;) {
            const int node = order[idx];
            if (!active[static_cast<std::size_t>(node)]) {
                continue;
            }
            const std::vector<int>& kids = activeChildren[static_cast<std::size_t>(node)];
            glm::vec3 pos = positions[static_cast<std::size_t>(node)];
            if (!kids.empty()) {
                // Sub-base rule: each moved child proposes a position at bone-length back along
                // the child->node direction; a branching joint takes the centroid.
                glm::vec3 centroid(0.0f);
                for (const int c : kids) {
                    const glm::vec3& childPos = positions[static_cast<std::size_t>(c)];
                    const glm::vec3 toNode = pos - childPos;
                    const float len = glm::length(toNode);
                    const float boneLen = lengthToParent[static_cast<std::size_t>(c)];
                    centroid += (len > kZeroLength) ? childPos + toNode * (boneLen / len)
                                                    : childPos;
                }
                pos = centroid / static_cast<float>(kids.size());
            }
            const int e = effectorAt[static_cast<std::size_t>(node)];
            if (e >= 0) {
                // Effector: snap toward the target (leaf goals fully, interior soft goals by
                // weight so they bias the chain without hijacking it).
                const IkEffector& eff = effectors[static_cast<std::size_t>(e)];
                const float w = kids.empty() ? glm::clamp(eff.weight, 0.0f, 1.0f)
                                             : glm::clamp(eff.weight, 0.0f, 1.0f) * 0.5f;
                pos = glm::mix(pos, eff.target, w);
            }
            positions[static_cast<std::size_t>(node)] = pos;
        }
        // Suspension gravity (see Settings): pull every non-effector active node down a little
        // each iteration; the constrained backward pass re-imposes bone lengths and joint
        // limits, so the equilibrium is the body hanging below the grab point — limbs dangling.
        if (settings.gravityBias > 0.0f) {
            for (const int node : order) {
                if (active[static_cast<std::size_t>(node)] &&
                    effectorAt[static_cast<std::size_t>(node)] < 0) {
                    positions[static_cast<std::size_t>(node)].y -= settings.gravityBias;
                }
            }
        }
        // Soft pose prior (see the header): interior nodes ease back toward the drag-start pose
        // before the backward pass re-imposes structure, each by its own stiffness weight (×
        // priorIterationNorm, so the per-SOLVE pull is invariant to the iteration budget).
        // Effector/pin nodes are exempt — their targets rule — so the equilibrium is the
        // closest-to-start pose satisfying the goals. The ROOT can be set to YIELD downward
        // (rootDownYield < 1) so a downward drag crouches the body instead of fighting the
        // stiff standing-height anchor.
        if (posePrior != nullptr && priorWeights != nullptr &&
            static_cast<int>(posePrior->size()) == n &&
            static_cast<int>(priorWeights->size()) == n) {
            for (const int node : order) {
                if (!active[static_cast<std::size_t>(node)]) {
                    continue;
                }
                // Effector nodes yield the prior by their goal WEIGHT, not binarily: any goal at
                // half snap strength or more rules its node outright (the drag goal, every pin,
                // the engaged balance/suspension effectors — all fully exempt, exactly the old
                // behavior), while a WEAK soft goal (the balance pelvis effector fading in)
                // blends prior and goal continuously. The binary exemption made adding/removing
                // a soft effector restructure the whole solve in one tick — the rig's balance
                // engagement then TOGGLED between two visibly different solutions (idle-limb
                // trembling while pulling).
                float priorScale = 1.0f;
                const int e = effectorAt[static_cast<std::size_t>(node)];
                if (e >= 0) {
                    priorScale = 1.0f - glm::clamp(effectors[static_cast<std::size_t>(e)].weight /
                                                       kPriorYieldWeight,
                                                   0.0f, 1.0f);
                    if (priorScale <= 0.0f) {
                        continue;
                    }
                }
                glm::vec3& pos = positions[static_cast<std::size_t>(node)];
                glm::vec3 toPrior = (*posePrior)[static_cast<std::size_t>(node)] - pos;
                if (node == root && settings.rootDownYield < 1.0f && toPrior.y > 0.0f) {
                    // The root sits BELOW its prior and the drag intent is downward: restore
                    // the vertical component only weakly — the crouch is allowed to deepen.
                    toPrior.y *= settings.rootDownYield;
                }
                pos += toPrior * glm::min(1.0f, (*priorWeights)[static_cast<std::size_t>(node)] *
                                                    settings.priorIterationNorm) *
                       priorScale;
            }
        }
        // Apply the reach leashes (see above): sequentially project the root into every pinned
        // limb's reach ball (two passes settle the 2-ball intersection of a standing figure).
        if (!leashes.empty()) {
            glm::vec3& rootPos = positions[static_cast<std::size_t>(root)];
            for (int pass = 0; pass < 2; ++pass) {
                for (const Leash& leash : leashes) {
                    const glm::vec3 d = rootPos - leash.target;
                    const float len = glm::length(d);
                    if (len > leash.maxReach) {
                        rootPos = leash.target + d * (leash.maxReach / len);
                    }
                }
            }
        }
        IK_NAN_CHECK("forward", iter);

        // --- Backward pass (root -> leaves): restore lengths and constraints from the root out.
        // The root FLOATS at whatever the forward pass proposed (the planted limbs hold it); a
        // PINNED root effector (a dragged pelvis, or the airborne-figure fallback) snaps instead.
        const int rootEffector = effectorAt[static_cast<std::size_t>(root)];
        if (rootEffector >= 0 && effectors[static_cast<std::size_t>(rootEffector)].pinned) {
            positions[static_cast<std::size_t>(root)] =
                effectors[static_cast<std::size_t>(rootEffector)].target;
        }
        frame[static_cast<std::size_t>(root)] = frameSeed[static_cast<std::size_t>(root)];
        for (const int node : order) {
            if (node == root || !active[static_cast<std::size_t>(node)]) {
                continue;
            }
            const int p = graph.parentOf(node);
            if (p >= 0) {
                // No escape kicks in the main pass: the fold-need judgement lives in the
                // restoration chains, where it is geometrically well-defined per pin.
                placeChild(node, p, 0.0f);
            }
        }

        IK_NAN_CHECK("backward", iter);

        // --- Effector chain restoration: the sub-base centroid averaging is a compromise between
        // branches, which leaves residual error at every effector (a planted foot sliding by
        // centimeters, a dragged hand lagging its cursor). For EACH effector — pins and the drag
        // goal alike — run single-chain FABRIK from it up to its sub-base (the nearest branching
        // ancestor, another effector, or the root, held fixed): forward-reach snaps the effector
        // and walks up; the constrained backward walk re-imposes lengths and limits. Only that
        // limb moves — the body keeps the global solution. The fold-escape kick lives HERE and
        // only here, because fold need is geometrically well-defined per chain (see below) —
        // this is also what lets a straight arm bend its elbow to bring the hand inward.
        for (const IkEffector& e : effectors) {
            if (e.node < 0 || e.node >= n || !active[static_cast<std::size_t>(e.node)]) {
                continue;
            }
            chain.clear();
            int cur = e.node;
            chain.push_back(cur);
            while (true) {
                const int p = graph.parentOf(cur);
                if (p < 0) {
                    break; // cur is the root: it is the sub-base
                }
                chain.push_back(p);
                // The chain's fixed end: the anchor, an INACTIVE ancestor (a caller-frozen base
                // — e.g. the trunk during an upward limb drag; restoring through it moved nodes
                // the active mask promised were untouched), a branching ancestor, or ANOTHER
                // EFFECTOR — restoring through an interior goal (the dragged hip on the way to
                // the far foot's pin) would undo that goal every round and the two would fight
                // forever.
                if (p == root || !active[static_cast<std::size_t>(p)] ||
                    activeChildren[static_cast<std::size_t>(p)].size() > 1 ||
                    effectorAt[static_cast<std::size_t>(p)] >= 0) {
                    break;
                }
                cur = p;
            }
            if (chain.size() < 2) {
                continue;
            }
            // The chain's goal honours the effector's WEIGHT: a full-strength goal (every pin,
            // the drag effector) is restored exactly onto its target, while a soft goal fading
            // in is restored only toward the weight-blended point — hard-snapping regardless of
            // weight would defeat the continuous fade the forward pass and prior-yield implement
            // (latent today: every restored effector currently carries weight 1).
            const glm::vec3 goal =
                e.weight >= 1.0f
                    ? e.target
                    : glm::mix(positions[static_cast<std::size_t>(e.node)], e.target,
                               glm::clamp(e.weight, 0.0f, 1.0f));
            // Single-chain FABRIK on the limb, iterated: one round is not enough when the limb
            // must fold (the constrained backward walk re-orients the hinge planes, and the next
            // forward-reach exploits the new orientation). Rounds run pure until one stops
            // improving, then a kick (scaled by the chain's OWN error) tries to break a straight
            // lock — and the chain KEEPS ITS BEST configuration across rounds: constrained greedy
            // iteration can wander into legal-but-wrong basins, and a kicked round that helped is
            // kept while one that hurt is discarded, so restoration can never make a pin WORSE.
            //
            // The kick is gated on FOLD NEED — the pin target lying well INSIDE the chain's reach,
            // so a bend is geometrically required (a crouch: the sub-base dropped toward the
            // feet). Error size alone is NOT a fold signal: a standing leg whose foot pin drifted
            // a few cm has its target AT reach — kicking it (toward the thigh's dominant range
            // side = hip flexion) hoisted the knee of a planted leg during ordinary hand drags.
            float chainLen = 0.0f;
            for (std::size_t m = 0; m + 1 < chain.size(); ++m) {
                chainLen += lengthToParent[static_cast<std::size_t>(chain[m])];
            }
            // The kick fires only for targets requiring a DEEP fold (< 90% of chain length): a
            // crouch's knees and a reaching arm's elbow qualify and benefit even mid-bend, while
            // near-full-extension targets (a hovering figure healing down onto its pins at ~93%
            // reach, a leg planting at a leash boundary) are EXTENSION problems where the same
            // push over-folds the joint and holds the end off its target. This one threshold
            // separated the three scenarios that defeated extension-ratio and must-shorten gates.
            const bool foldNeeded =
                glm::length(positions[static_cast<std::size_t>(chain.back())] - goal) <
                0.90f * chainLen;
            chainBest.clear();
            chainBestFrame.clear();
            for (const int cnode : chain) {
                chainBest.push_back(positions[static_cast<std::size_t>(cnode)]);
                chainBestFrame.push_back(frame[static_cast<std::size_t>(cnode)]);
            }
            float chainBestErr =
                glm::length(positions[static_cast<std::size_t>(e.node)] - goal);
            float chainPrevErr = chainBestErr;

            // --- Analytic TWO-BONE assist (legs): greedy constrained rounds have a recurring
            // failure basin at near-full extension — a leg whose foot pin sits at ~95% reach
            // hard-stops centimeters short, and once there no amount of iteration recovers (the
            // "extend-to-plant" soft spot: crouch feet lagging, a sustained hand pull gradually
            // trading a planted foot away, strained releases stalling off their pins). For a
            // chain that collapses to exactly TWO effective segments — its interior joints all
            // rigid pass-throughs (mid-limb twist bones) except one (the knee) — the mid-joint
            // position is computed ANALYTICALLY (two-sphere intersection circle, taking the
            // point nearest the current mid joint so the existing bend side/pose continuity is
            // preserved), the pass-throughs are seeded along the segments, and the normal
            // constrained rounds below legalize the configuration against the joint limits.
            // Purely exploratory: chainBest keeps the seed only if it actually lands the
            // effector better, so a bad seed costs nothing. Arms (collar+shoulder+elbow chains
            // have >2 effective joints) don't match and keep the iterative path.
            if (chain.size() >= 4 && chainBestErr > settings.tolerance) {
                int mid = -1;
                int base = -1;
                // The MID joint is the chain's deepest-folding real joint (knee/elbow class,
                // widest dominant range, at least ~100°), the BASE the next real joint above it.
                // Selecting by fold capability, not adjacency: a metatarsal-pinned chain's first
                // two real joints are ankle+knee — a degenerate short-segment pair whose seed
                // always lost, leaving the leg with no assist at all (feet lagged their pins
                // whenever the pin sat below the ankle).
                const auto dominantRange = [&](std::size_t m) {
                    const JointConstraint& jc =
                        edgeConstraint[static_cast<std::size_t>(chain[m - 1])];
                    if (jc.type == JointConstraint::Type::Hinge) {
                        return jc.maxAngle - jc.minAngle;
                    }
                    if (jc.type == JointConstraint::Type::Cone && jc.perAxis) {
                        return std::max(jc.swing0Max - jc.swing0Min, jc.swing1Max - jc.swing1Min);
                    }
                    if (jc.type == JointConstraint::Type::Free) {
                        return 6.2831853f;
                    }
                    return 0.0f; // rigid (zero-swing) pass-through
                };
                // First DEEP-FOLD joint from the effector (>= 120°: knees ~166° and elbows
                // ~155° qualify; a ~110° ankle or a small wrist does not — those articulate
                // slightly inside the lower segment and the constrained walk absorbs them).
                // Taking the WIDEST joint instead broke arms whose shoulder is freer than the
                // elbow: the pair straddled the articulated elbow and fought its fold.
                // A qualifying mid must also carry a REAL lower segment (>= 20% of the chain,
                // scale-invariant): a FINGER effector's chain runs through the knuckles, and a
                // knuckle's ~140° curl range passed the fold gate — the assist then worked a
                // 4cm finger pair while the ELBOW (the joint that actually extends the arm
                // overhead) got no assist at all, stalling a raised-by-the-finger hand at chin
                // height (grabbing near a hand almost always picks a finger).
                for (std::size_t m = 1; m + 1 < chain.size(); ++m) {
                    if (dominantRange(m) < glm::radians(120.0f)) {
                        continue;
                    }
                    float lowerLen = 0.0f;
                    for (std::size_t k = 0; k < m; ++k) {
                        lowerLen += lengthToParent[static_cast<std::size_t>(
                            chain[static_cast<std::size_t>(k)])];
                    }
                    if (lowerLen < 0.2f * chainLen) {
                        continue;
                    }
                    mid = static_cast<int>(m);
                    break;
                }
                if (mid >= 0) {
                    for (std::size_t m = static_cast<std::size_t>(mid) + 1; m + 1 < chain.size();
                         ++m) {
                        if (dominantRange(m) > 1e-5f) {
                            base = static_cast<int>(m); // nearest real joint above the fold
                            break;
                        }
                    }
                }
                static const bool kTwoBoneTrace = std::getenv("IK_TWOBONE_TRACE") != nullptr;
                if (kTwoBoneTrace) {
                    std::printf("[2bone] eff=%d chainN=%zu mid=%d(node %d) base=%d(node %d)\n",
                                e.node, chain.size(), mid,
                                mid >= 0 ? chain[static_cast<std::size_t>(mid)] : -1, base,
                                base >= 0 ? chain[static_cast<std::size_t>(base)] : -1);
                }
                if (base > mid && mid > 0) {
                    // Segment rest offsets (kinks included) from the rest geometry: base->mid
                    // spans edges chain[mid..base-1], mid->effector spans chain[0..mid-1].
                    glm::vec3 upper(0.0f);
                    for (int k = mid; k < base; ++k) {
                        upper += edgeRestDir[static_cast<std::size_t>(chain[static_cast<std::size_t>(k)])] *
                                 lengthToParent[static_cast<std::size_t>(chain[static_cast<std::size_t>(k)])];
                    }
                    glm::vec3 lower(0.0f);
                    for (int k = 0; k < mid; ++k) {
                        lower += edgeRestDir[static_cast<std::size_t>(chain[static_cast<std::size_t>(k)])] *
                                 lengthToParent[static_cast<std::size_t>(chain[static_cast<std::size_t>(k)])];
                    }
                    const float len1 = glm::length(upper);
                    const float len2 = glm::length(lower);
                    const glm::vec3 basePos =
                        positions[static_cast<std::size_t>(chain[static_cast<std::size_t>(base)])];
                    glm::vec3 toTarget = goal - basePos;
                    float d = glm::length(toTarget);
                    if (len1 > kZeroLength && len2 > kZeroLength && d > kZeroLength) {
                        d = glm::clamp(d, std::abs(len1 - len2) + 1e-4f, len1 + len2 - 1e-4f);
                        const glm::vec3 axis = toTarget / glm::length(toTarget);
                        // Mid-joint circle: distance a from the base along the axis, radius r.
                        const float a = (len1 * len1 - len2 * len2 + d * d) / (2.0f * d);
                        const float r2 = len1 * len1 - a * a;
                        const glm::vec3 center = basePos + axis * a;
                        const glm::vec3 curMid =
                            positions[static_cast<std::size_t>(chain[static_cast<std::size_t>(mid)])];
                        glm::vec3 perp = (curMid - center) - axis * glm::dot(curMid - center, axis);
                        const float perpLen = glm::length(perp);
                        if (r2 > 1e-8f && perpLen > 1e-5f) {
                            const glm::vec3 midPos = center + perp * (std::sqrt(r2) / perpLen);
                            // Seed: mid joint on the circle, pass-throughs linearly along their
                            // segment (cumulative rest length), effector on the target.
                            float acc = 0.0f;
                            for (int k = base - 1; k >= mid; --k) {
                                acc += lengthToParent[static_cast<std::size_t>(
                                    chain[static_cast<std::size_t>(k)])];
                                positions[static_cast<std::size_t>(chain[static_cast<std::size_t>(k)])] =
                                    glm::mix(basePos, midPos, glm::min(1.0f, acc / len1));
                            }
                            acc = 0.0f;
                            for (int k = mid - 1; k >= 0; --k) {
                                acc += lengthToParent[static_cast<std::size_t>(
                                    chain[static_cast<std::size_t>(k)])];
                                positions[static_cast<std::size_t>(chain[static_cast<std::size_t>(k)])] =
                                    glm::mix(midPos, goal, glm::min(1.0f, acc / len2));
                            }
                        }
                    }
                }
            }
            for (int round = 0; round < 8 && chainBestErr > settings.tolerance; ++round) {
                const float pinErr =
                    glm::length(positions[static_cast<std::size_t>(e.node)] - goal);
                const bool chainStalled = round > 0 && pinErr > chainPrevErr - settings.tolerance;
                chainPrevErr = pinErr;
                const float chainBias =
                    (earlyPhase && chainStalled && foldNeeded && pinErr > kKickErrorFloor)
                        ? kickFor(pinErr)
                        : 0.0f;
                // Forward-reach from the pin toward the (fixed) sub-base.
                positions[static_cast<std::size_t>(chain[0])] = goal;
                for (std::size_t m = 1; m + 1 < chain.size(); ++m) {
                    const glm::vec3& prev = positions[static_cast<std::size_t>(chain[m - 1])];
                    glm::vec3 toNode = positions[static_cast<std::size_t>(chain[m])] - prev;
                    const float len = glm::length(toNode);
                    const float boneLen = lengthToParent[static_cast<std::size_t>(chain[m - 1])];
                    positions[static_cast<std::size_t>(chain[m])] =
                        (len > kZeroLength) ? prev + toNode * (boneLen / len) : prev;
                }
                // Constrained backward walk from the sub-base down to the pin.
                for (std::size_t m = chain.size() - 1; m-- > 0;) {
                    placeChild(chain[m], chain[m + 1], chainBias);
                }
#ifdef IK_DEBUG_PRINT
                std::printf("  [restore] pin %d chain %zu sub-base %d round %d err %g bias %g\n",
                            e.node, chain.size(), chain.back(), round,
                            glm::length(positions[static_cast<std::size_t>(e.node)] - goal),
                            chainBias);
#endif
                const float roundErr =
                    glm::length(positions[static_cast<std::size_t>(e.node)] - goal);
                if (roundErr < chainBestErr) {
                    chainBestErr = roundErr;
                    for (std::size_t m = 0; m < chain.size(); ++m) {
                        chainBest[m] = positions[static_cast<std::size_t>(chain[m])];
                        chainBestFrame[m] = frame[static_cast<std::size_t>(chain[m])];
                    }
                }
            }
            // Leave the limb in its best configuration, not the last round's — FRAMES included:
            // a later effector's chain may anchor at this effector's node (the "another effector"
            // sub-base case), and restoring positions while leaving the frames at a discarded
            // kicked round's values constrained that chain about an orientation this node no
            // longer holds.
            for (std::size_t m = 0; m < chain.size(); ++m) {
                positions[static_cast<std::size_t>(chain[m])] = chainBest[m];
                frame[static_cast<std::size_t>(chain[m])] = chainBestFrame[m];
            }
        }

        IK_NAN_CHECK("restore", iter);
        error = worstError();
        if (error < bestError) {
            bestError = error;
            bestPositions = positions;
        }
#ifdef IK_DEBUG_PRINT
        std::printf("[fabrik] iter %d error %g\n", iter, error);
        for (const IkEffector& e : effectors) {
            std::printf("  eff node %d pinned %d err %g\n", e.node, e.pinned ? 1 : 0,
                        glm::length(positions[static_cast<std::size_t>(e.node)] - goal));
        }
#endif
    }
    if (!bestPositions.empty()) {
        positions = bestPositions;
    }
    // Trust region (see Settings): clamp every active node's displacement from its entry
    // position, so per-event pose change is bounded and internal basin switches can never snap.
    if (settings.maxStepDisplacement > 0.0f || settings.minStepDisplacement > 0.0f) {
        for (int i = 0; i < n; ++i) {
            if (!active[static_cast<std::size_t>(i)]) {
                continue;
            }
            const glm::vec3 delta =
                positions[static_cast<std::size_t>(i)] - entryPositions[static_cast<std::size_t>(i)];
            const float len = glm::length(delta);
            if (settings.minStepDisplacement > 0.0f && len < settings.minStepDisplacement) {
                // Output deadband (see Settings): sub-threshold motion is the loop's own
                // micro-oscillation, not a correction — hold the node perfectly still.
                positions[static_cast<std::size_t>(i)] = entryPositions[static_cast<std::size_t>(i)];
            } else if (settings.maxStepDisplacement > 0.0f && len > settings.maxStepDisplacement) {
                positions[static_cast<std::size_t>(i)] =
                    entryPositions[static_cast<std::size_t>(i)] +
                    delta * (settings.maxStepDisplacement / len);
            }
        }
    }
    return bestError;
}

} // namespace pose
