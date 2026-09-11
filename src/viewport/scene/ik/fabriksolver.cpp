/**
 * @file fabriksolver.cpp
 * @brief The multi-chain FABRIK solve: forward and backward passes over the pelvis-rooted graph,
 *        per-effector RESTORATION chains, the hard-pin policy, reach leashes, the floor
 *        constraint, the soft pose prior, and best-state keeping.
 *
 * Every policy here fixed a measured failure (see tools/ikharness): constraint frames are
 * evaluated on the OWNING (parent) side and seeded from the current pose; the root's rotation is
 * never solved; bone lengths come from bind data, never from the entry positions; the fold-escape
 * kick lives ONLY in the restoration chains and is triple-gated (stall, an error floor, and
 * geometric FOLD NEED); a chain that collapses to a two-bone limb gets an analytic seed on its
 * joint's FLEXION side; the global and per-chain best states are kept (greedy constrained
 * iteration wanders); hard pins back the soft goals off toward their solve-entry positions and
 * get a final kick-free polish; and the floor is applied where lengths are set (inside
 * placeChild), never as a post-hoc clamp. Diagnostics: IK_CHAIN_TRACE=<node>, IK_FLOOR_TRACE,
 * IK_TWOBONE_TRACE, POSESTUDIO_IK_SOLVER_TRACE, and the IK_NAN_TRAP compile-time guard.
 * Qt-free (std + GLM).
 */
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

// Forward-pass centroid weight of a branch that serves a PINNED effector, at non-root branching
// joints (a plain average let a soft drag goal pull the chest out of a pinned hand's reach).
constexpr float kPinPriority = 3.0f;

// Hard-pin policy (see IkEffector::hard): after each iteration, if any hard pin misses by more
// than kHardPinTol the goals back off by kGoalBackoffStep (their effective targets slide toward
// the nodes' iteration-entry positions), so the pose converges to the best reach that keeps
// every hard pin exactly held. Hard pins weigh kHardPinErrorWeight in the best-state metric, so
// a pin-violating state can never be kept over a pin-holding one.
constexpr float kHardPinTol = 0.005f;
constexpr float kGoalBackoffStep = 0.25f;
constexpr float kHardPinErrorWeight = 10.0f;

// Restoration rounds per effector chain per iteration (see restoreChain in solve()), and the
// rounds of the final hard-pin POLISH. Eight rounds converge a limb chain (a leg, an arm to its
// collar) but under-converge a hard pin's LONG chain — a hand pinned through a pelvis crouch
// restores through the whole spine (11 joints) and stalled 4-6cm off its pin every iteration,
// which also fired the goal back-off spuriously and stopped the hips early. Raising the per-
// iteration rounds instead is chaotic (24 hard-pin rounds passed the suite, 40 slipped the same
// pin by 5cm, 40 everywhere broke a synthetic crouch): each iteration's main passes perturb the
// chain again, so the round count reshapes the whole solve. The polish runs ONCE on the final
// best state, where nothing perturbs the chain afterwards — the pin lands as exactly as the
// joint limits allow regardless of how the iterations went, and it can never make a pin worse
// (the chain keeps its best configuration).
constexpr int kRestoreRounds = 8;
constexpr int kHardPinPolishRounds = 24;

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
    // Nodes on a PINNED effector's path to the root: at a branching joint the forward pass's
    // centroid gives a pin-serving branch more say than a goal-serving one (see the sub-base
    // rule below) — a pin is a constraint, the drag goal only a wish, and an equal vote let
    // the chest settle where a pinned hand's arm could no longer reach it.
    std::vector<char> servesPin(static_cast<std::size_t>(n), 0);
    for (const IkEffector& e : effectors) {
        if (!e.pinned || e.node < 0 || e.node >= n) {
            continue;
        }
        for (int cur = e.node; cur >= 0; cur = graph.parentOf(cur)) {
            servesPin[static_cast<std::size_t>(cur)] = 1;
        }
    }

    // Frames start from the CURRENT pose rotations, not identity: the backward pass overwrites
    // every ACTIVE node's frame from its placed swing, but an INACTIVE node's frame is read as-is
    // when a restoration chain anchors at it (a caller-frozen sub-base — the trunk during an
    // up-intent drag). Identity there evaluated the chain's first constraint in the bone's BIND
    // frame: with the chest pre-bent, the shoulder clamped toward anatomically wrong directions.
    std::vector<glm::quat> frame = frameSeed;

    // RIGID multi-child joints (see JointConstraint::parentAxis): a joint with several ACTIVE
    // children — the pelvis with both hip sockets, the upper chest with both collars — placed
    // with ONE shared rotation (fitted to all of them, clamped on the parent's own limits)
    // instead of one swing per edge. EXPERIMENT, DEFAULT OFF (IK_RIGID_GROUPS=1 enables it):
    // per-edge placement lets the solve bend the pelvis between its sockets, and the
    // extraction's rigid pelvis then lands the second foot 2-3cm high after a stepping lean on
    // the newest and oldest rigs; every rigid formulation measured (rigid in the main pass and
    // the restorations; rigid only in a final polish; free restorations then a rigid
    // re-landing every iteration) fixed those landings but tripled the standing knees'
    // tick-to-tick reversals under a lateral lean on the base rig (the roll a lean needs is
    // discovered by the free sockets; rigid, the two sockets ask for opposite rotations and
    // the lean lands in the knees). The extraction's bisector fit plus the settle's contact
    // refinement carry the landing instead (Armature::applyIkSolution / refinePins).
    static const bool kRigidGroups = std::getenv("IK_RIGID_GROUPS") != nullptr;
    std::vector<glm::quat> groupLocal(static_cast<std::size_t>(n), glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
    std::vector<char>      placedInGroup(static_cast<std::size_t>(n), 0);
    const auto isGrouped = [&](int p) {
        return kRigidGroups && p >= 0 && p != root &&
               activeChildren[static_cast<std::size_t>(p)].size() > 1;
    };
    // Each grouped rotation starts at the POSE's own (the bone's rotation relative to its
    // parent bone) and is under-relaxed toward every iteration's fit: refitted outright from
    // the forward pass's proposals it alternated between two pelvis tilts iteration to
    // iteration, and the knees shook under every lean and walk.
    for (int p = 0; p < n; ++p) {
        const int q = graph.parentOf(p);
        if (q >= 0 && isGrouped(p)) {
            groupLocal[static_cast<std::size_t>(p)] = glm::normalize(
                glm::inverse(frameSeed[static_cast<std::size_t>(q)]) * frameSeed[static_cast<std::size_t>(p)]);
        }
    }
    static const float kGroupRelaxation = [] {
        const char* v = std::getenv("IK_GROUP_RELAX"); // A/B diagnostics
        return v != nullptr ? static_cast<float>(std::atof(v)) : 0.5f;
    }();
    // The restoration chains EXPLORE with free socket edges (the pre-group behaviour: a foot's
    // chain may swing its own hip socket, and the stretch it leaves is what lets a lateral
    // lean discover the pelvis ROLL that serves both feet — held rigid there, the two sockets
    // demanded opposite rotations, no roll ever developed, the whole lean landed in the knees
    // and they flipped every tick). Rigidity is imposed by the main pass and by the FINAL
    // polish (see the end of solve()), so the returned state is one a rigid pelvis realizes.
    bool socketsRigid = true;

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
                    // The rig supplies the radius (the DRAG-START socket-to-pin distance — a
                    // stance the figure provably held — with fractional path-length headroom
                    // for bent-leg starts) and the ball's center offset (see
                    // IkEffector::leashOffset): the ball confines the SOCKET, expressed on the
                    // root through the fixed root->socket offset.
                    leashes.push_back({e.target - e.leashOffset, e.leashRadius});
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
    // Armature's root pose translation) and is held by the planted limbs — or by a caller-provided
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
    // IK_CHAIN_TRACE=<effector node>: per-round error and per-edge clamp angles of that
    // effector's restoration chain (diagnostic only).
    static const int kChainTraceNode = [] {
        const char* v = std::getenv("IK_CHAIN_TRACE");
        return v != nullptr ? std::atoi(v) : -1;
    }();
    bool chainTracing = false;
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
        } else if (isGrouped(p) && socketsRigid) {
            // A grouped child (see groupLocal): rigid with its parent's shared rotation.
            clamped = glm::normalize(parentFrame * (groupLocal[static_cast<std::size_t>(p)] * restDir));
        } else {
            clamped = constrainSegmentDirection(edgeConstraint[static_cast<std::size_t>(node)],
                                                parentFrame, restDir, dir, bias,
                                                &frameSeed[static_cast<std::size_t>(p)]);
        }
        // THE FLOOR (see Settings::floorClearance): a child placed below its clearance is
        // swung UP to the floor plane at bone length — a constraint like the joint limits,
        // applied where lengths are set, so the kept state stays reproducible by rotations (a
        // post-hoc clamp lifted joints without their parents, and the extraction of that
        // broken-length state rotated a pinned foot and sank the other 6cm). When even the
        // plane is out of the segment's reach (the parent itself is under the floor), the
        // child is lifted onto it and the next passes restore lengths around that.
        // Free joints only: an effector has its own target (pins sit AT their clearance, and
        // the Armature clamps the drag goal) — projecting those too made the planted feet chatter
        // at the boundary every pass (steps failed to trigger, releases landed 2cm off).
        if (settings.floorClearance != nullptr &&
            static_cast<int>(settings.floorClearance->size()) == n &&
            effectorAt[static_cast<std::size_t>(node)] < 0) {
            const float minY =
                settings.floorY + (*settings.floorClearance)[static_cast<std::size_t>(node)];
            if (parentPos.y + clamped.y * boneLen < minY) {
                static const bool kFloorTrace = std::getenv("IK_FLOOR_TRACE") != nullptr;
                if (kFloorTrace) {
                    std::printf("[floor] node=%d lifted from y=%.3f to %.3f", node,
                                parentPos.y + clamped.y * boneLen, minY);
                    std::putchar(10);
                }
                const float dy = (minY - parentPos.y) / boneLen;
                if (dy < 1.0f) {
                    const float xzLen = std::sqrt(std::max(0.0f, 1.0f - dy * dy));
                    glm::vec2 xz(clamped.x, clamped.z);
                    const float l = glm::length(xz);
                    xz = (l > 1e-6f) ? xz * (xzLen / l) : glm::vec2(xzLen, 0.0f);
                    clamped = glm::vec3(xz.x, dy, xz.y);
                } else {
                    clamped = glm::vec3(0.0f, 1.0f, 0.0f);
                }
            }
        }
        if (chainTracing) {
            const glm::vec3 restWorld = parentFrame * restDir;
            const float wantDeg = glm::degrees(std::acos(glm::clamp(glm::dot(dir, restWorld), -1.0f, 1.0f)));
            const float gotDeg = glm::degrees(std::acos(glm::clamp(glm::dot(clamped, restWorld), -1.0f, 1.0f)));
            const float clampDeg = glm::degrees(std::acos(glm::clamp(glm::dot(dir, clamped), -1.0f, 1.0f)));
            const JointConstraint& jc = edgeConstraint[static_cast<std::size_t>(node)];
            std::printf("      [edge] node=%d p=%d type=%d wantSwing=%.1f gotSwing=%.1f clamp=%.1f deg\n",
                        node, p, static_cast<int>(jc.type), wantDeg, gotDeg, clampDeg);
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

    // The shared rotation of a grouped parent @p p from its children's PROPOSED positions (the
    // forward pass's): the longest edge is aligned exactly (shortest arc), the others fit the
    // twist about it, and the result is clamped as a rotation vector on the parent's three
    // authored channels — then every child is placed with it through placeChild's rigid path.
    auto placeGroup = [&](int p) {
        const std::vector<int>& kids = activeChildren[static_cast<std::size_t>(p)];
        const glm::quat& F = frame[static_cast<std::size_t>(p)];
        const glm::vec3& pp = positions[static_cast<std::size_t>(p)];
        int   prim = kids[0];
        float primLen = -1.0f;
        for (const int c : kids) {
            if (lengthToParent[static_cast<std::size_t>(c)] > primLen) {
                primLen = lengthToParent[static_cast<std::size_t>(c)];
                prim = c;
            }
        }
        const auto proposedDir = [&](int c) {
            const glm::vec3 d = positions[static_cast<std::size_t>(c)] - pp;
            const float l = glm::length(d);
            return l > kZeroLength ? d / l
                                   : glm::normalize(F * edgeRestDir[static_cast<std::size_t>(c)]);
        };
        const glm::vec3 primRest = glm::normalize(F * edgeRestDir[static_cast<std::size_t>(prim)]);
        const glm::vec3 primDir = proposedDir(prim);
        glm::quat R = shortestArc(primRest, primDir);
        float twistSum = 0.0f;
        int   twistCount = 0;
        for (const int c : kids) {
            if (c == prim) {
                continue;
            }
            glm::vec3 v = R * (F * edgeRestDir[static_cast<std::size_t>(c)]);
            glm::vec3 d = proposedDir(c);
            v -= primDir * glm::dot(v, primDir);
            d -= primDir * glm::dot(d, primDir);
            if (glm::dot(v, v) > 0.01f && glm::dot(d, d) > 0.01f) {
                twistSum += signedAngleAround(glm::normalize(v), glm::normalize(d), primDir);
                ++twistCount;
            }
        }
        if (twistCount > 0) {
            R = glm::normalize(glm::angleAxis(twistSum / static_cast<float>(twistCount), primDir) * R);
        }
        // Clamp on the parent's channels (rotation-vector components ≈ Euler angles at the
        // ranges involved: a pelvis tilts ±25°).
        {
            const JointConstraint& jc = edgeConstraint[static_cast<std::size_t>(prim)];
            glm::quat q = R;
            if (q.w < 0.0f) {
                q = -q;
            }
            const glm::vec3 v(q.x, q.y, q.z);
            const float vl = glm::length(v);
            const glm::vec3 w = vl > 1e-9f ? v * (2.0f * std::atan2(vl, q.w) / vl) : glm::vec3(0.0f);
            glm::vec3 clampedW(0.0f);
            for (int a = 0; a < 3; ++a) {
                const glm::vec3 ax = glm::normalize(F * jc.parentAxis[a]);
                clampedW += ax * glm::clamp(glm::dot(w, ax), jc.parentMin[a], jc.parentMax[a]);
            }
            const float cl = glm::length(clampedW);
            R = cl > 1e-6f ? glm::angleAxis(cl, clampedW / cl) : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        }
        const glm::quat fit = glm::normalize(glm::inverse(F) * R * F);
        glm::quat& kept = groupLocal[static_cast<std::size_t>(p)];
        kept = glm::normalize(glm::slerp(kept, glm::dot(kept, fit) < 0.0f ? -fit : fit,
                                         kGroupRelaxation));
        for (const int c : kids) {
            placeChild(c, p, 0.0f); // the rigid path: parentFrame * groupLocal * rest (+ floor)
            placedInGroup[static_cast<std::size_t>(c)] = 1;
        }
    };

    auto worstError = [&]() {
        float worst = 0.0f;
        for (const IkEffector& e : effectors) {
            // Inactive-node effectors are skipped everywhere else (leashes, restoration) because
            // the solve cannot move them; counting their error here made it a constant floor that
            // kept every iteration running and inflated the returned error past any freeze gate.
            if (e.node >= 0 && e.node < n && active[static_cast<std::size_t>(e.node)]) {
                const float err =
                    glm::length(positions[static_cast<std::size_t>(e.node)] - e.target);
                worst = std::max(worst, e.hard ? err * kHardPinErrorWeight : err);
            }
        }
        return worst;
    };
    // Any hard pin's true error (see the hard-pin policy): what the goal back-off watches.
    auto worstHardPinError = [&]() {
        float worst = 0.0f;
        for (const IkEffector& e : effectors) {
            if (e.hard && e.node >= 0 && e.node < n && active[static_cast<std::size_t>(e.node)]) {
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
    std::vector<glm::quat> bestFrame; // the best state's frames, for the hard-pin polish
    float bestError = 1e30f;
    const auto kickFor = [&settings](float err) {
        return std::min(0.5f, settings.straightBias + 2.0f * err);
    };
    // At least one iteration always runs: even with every effector at its target the state can be
    // inconsistent (see the best-state note above) and needs one forward/backward reconciliation.
    // Effective per-iteration targets: hard pins and ground-contact pins always keep their own;
    // the soft goals and a DRAGGED root slide toward their nodes' iteration-entry positions by
    // the current back-off (0 = the true targets) once a hard pin has failed to hold.
    std::vector<glm::vec3> effTarget(effectors.size());
    float goalBackoff = 0.0f;
    bool anyHardPin = false;
    // Diagnostics for the POSESTUDIO_IK_SOLVER_TRACE dump: each effector's restoration chain
    // (length, sub-base, best error) from the last iteration that ran it.
    static const bool kSolverTrace = std::getenv("POSESTUDIO_IK_SOLVER_TRACE") != nullptr;
    std::vector<std::size_t> traceChainLen(kSolverTrace ? effectors.size() : 0, 0);
    std::vector<int> traceSubBase(kSolverTrace ? effectors.size() : 0, -1);
    std::vector<float> traceChainErr(kSolverTrace ? effectors.size() : 0, -1.0f);
    int traceIters = 0;
    for (const IkEffector& e : effectors) {
        anyHardPin = anyHardPin || (e.hard && e.node >= 0 && e.node < n);
    }
    // Single-chain restoration of effector @p ei: single-chain FABRIK from the effector up to
    // its sub-base (the nearest branching ancestor, another effector, an inactive ancestor, or
    // the root, held fixed) — forward-reach snaps the effector and walks up; the constrained
    // backward walk re-imposes lengths and limits — iterated for up to @p maxRounds rounds,
    // keeping the chain's BEST configuration. Called per effector after every iteration's main
    // passes, and once more as the hard-pin POLISH on the final best state (see below).
    // @p earlyPhase enables the straight-limb escape kick. With @p throughBranches the chain
    // runs THROUGH branching ancestors up to the root (still stopping at other effectors and
    // inactive nodes): a HARD pin's chain. A pinned hand's limb stops at the chest, and once a
    // drag has pulled the chest a centimetre out of that arm's reach nothing in the solve
    // brings it back — the pin misses, the goals back off to 100% (the drag makes no progress
    // at all), and the Armature's root-translation refinement then holds the pin by shifting
    // the pelvis, which the next solve undoes: the legs trembled under every pinned-hand drag.
    // Restored through the spine, the pin bends the trunk to itself first and the goal's chain
    // (restored after, from the moved chest) adapts — the pin is the declared intent.
    const auto restoreChain = [&](std::size_t ei, int maxRounds, bool earlyPhase,
                                  bool throughBranches) {
        const IkEffector& e = effectors[ei];
        if (e.node < 0 || e.node >= n || !active[static_cast<std::size_t>(e.node)]) {
            return;
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
                (!throughBranches && activeChildren[static_cast<std::size_t>(p)].size() > 1) ||
                effectorAt[static_cast<std::size_t>(p)] >= 0) {
                break;
            }
            cur = p;
        }
        if (chain.size() < 2) {
            return;
        }
        // The chain's goal honours the effector's WEIGHT: a full-strength goal (every pin,
        // the drag effector) is restored exactly onto its target, while a soft goal fading
        // in is restored only toward the weight-blended point — hard-snapping regardless of
        // weight would defeat the continuous fade the forward pass and prior-yield implement
        // (latent today: every restored effector currently carries weight 1).
        const glm::vec3& eTarget = effTarget[ei]; // backed-off for soft goals (hard-pin policy)
        const glm::vec3 goal =
            e.weight >= 1.0f
                ? eTarget
                : glm::mix(positions[static_cast<std::size_t>(e.node)], eTarget,
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
                    float perpLen = glm::length(perp);
                    // WHICH SIDE of the axis the mid joint bulges to: the current knee's side
                    // keeps pose continuity, but only while that side is the joint's FLEXION
                    // side. A knee that drifted onto its hyperextension/lateral side (the
                    // constrained rounds leave a blocked knee wherever the clamp stopped it)
                    // was re-seeded there every solve, the clamp cut the seed's 21° swing to
                    // its 12° of hyperextension-plus-play, and the foot sat 7cm off its pin
                    // with the thigh creeping toward the fix at 0.2°/round: every walk landing
                    // ended 5cm inside its stance and 1.5cm through the floor. Flexing a hinge
                    // swings the lower segment along its dominant tangent, so the mid joint
                    // bulges the OPPOSITE way — that is the side a bendable joint can hold.
                    {
                        const JointConstraint& midJc = edgeConstraint[static_cast<std::size_t>(
                            chain[static_cast<std::size_t>(mid - 1)])];
                        const glm::vec3& midRest =
                            edgeRestDir[static_cast<std::size_t>(chain[static_cast<std::size_t>(mid - 1)])];
                        glm::vec3 tangent(0.0f);
                        const bool hasTangent = dominantBendTangent(midJc, midRest, tangent);
                        if (hasTangent) {
                            const glm::quat& midFrame =
                                frame[static_cast<std::size_t>(chain[static_cast<std::size_t>(mid)])];
                            glm::vec3 pref = -(midFrame * tangent);
                            // A HINGE with twist freedom (an elbow) folds in the plane the pose
                            // CURRENTLY carries, not the chained frame's zero-twist plane: the
                            // chained frames are swing-only, so the zero-twist flexion side can
                            // sit a quarter turn from where the extraction's twist channel
                            // actually is (a return-to-rest parks the upper-arm twist at its
                            // limit), and an elbow seeded there is one the pose cannot realize —
                            // the solver's arm and the applied arm disagreed by 5cm at the elbow
                            // every tick until a fast flick ran away. The twist search and the
                            // witness then move the plane from the current one continuously.
                            if (midJc.type == JointConstraint::Type::Hinge &&
                                midJc.twistMax - midJc.twistMin > 1e-6f) {
                                const glm::vec3 u = glm::normalize(midFrame * midJc.twistAxis);
                                const glm::quat& seedFrame = frameSeed[static_cast<std::size_t>(
                                    chain[static_cast<std::size_t>(mid)])];
                                glm::vec3 hChain = midFrame * midJc.hingeAxis;
                                glm::vec3 hSeed = seedFrame * midJc.hingeAxis;
                                hChain -= u * glm::dot(hChain, u);
                                hSeed -= u * glm::dot(hSeed, u);
                                if (glm::dot(hChain, hChain) > 0.01f && glm::dot(hSeed, hSeed) > 0.01f) {
                                    const float phiCur = glm::clamp(
                                        signedAngleAround(glm::normalize(hChain), glm::normalize(hSeed), u),
                                        midJc.twistMin, midJc.twistMax);
                                    pref = glm::angleAxis(phiCur, u) * pref;
                                }
                            }
                            pref -= axis * glm::dot(pref, axis);
                            const float prefLen = glm::length(pref);
                            if (prefLen > 1e-5f && (perpLen <= 1e-5f || glm::dot(perp, pref) < 0.0f)) {
                                perp = pref;
                                perpLen = prefLen;
                            }
                            // (A cone-aware elbow azimuth — walking the mid-joint circle for
                            // the azimuth the SHOULDER's cone admits nearest the preferred one
                            // — was built and measured for a hand planted on the floor under
                            // a descending trunk (the live-contact round): it did not make the
                            // arm's chain converge (the collar and clavicle cones and the kinked
                            // rigid wrist link are what the restoration rounds diverge on), it
                            // slid the planted hands 50% farther, and it perturbed the
                            // suspension's dangling arms. Dropped; the joint-space refinement
                            // holds live contacts instead.)
                        }
                        // A near-hinge CONE (a knee) has no twist of its own: its fold plane is
                        // carried by the thigh's swing, so the ONE azimuth on the circle where
                        // that plane contains the goal is where the knee belongs — over the
                        // foot. The forward-of-the-thigh-frame side above put the knee straight
                        // ahead of the hip socket while the foot stands 12cm outboard of it; the
                        // shin then had to angle out to the pin through a plane far from the
                        // knee's, which the solve could only serve with THIGH TWIST (when the
                        // knee had that freedom: every crouch ran it to the 75° limit, knees
                        // 10-20cm inward, planted feet rotated 45°) or not at all (11cm chain
                        // error, the foot buried). Solved on the circle: the knee's hinge axis
                        // — the base frame carried down the seeded upper segment by shortest arc
                        // — must be perpendicular to the shin, on the flexion side. An ELBOW (a
                        // true hinge under a twist bone) keeps the side rule: its fold plane IS a
                        // free degree of freedom the solve's twist search chooses.
                        if (hasTangent && midJc.type == JointConstraint::Type::Cone &&
                            midJc.perAxis && r2 > 1e-8f && perpLen > 1e-5f) {
                            const float reach0 = std::max(-midJc.swing0Min, midJc.swing0Max);
                            const float reach1 = std::max(-midJc.swing1Min, midJc.swing1Max);
                            const glm::vec3 hingeRest =
                                reach0 >= reach1 ? midJc.swingAxis0 : midJc.swingAxis1;
                            const glm::vec3 e1 = perp / perpLen;
                            const glm::vec3 e2 = glm::cross(axis, e1);
                            const float rad = std::sqrt(r2);
                            const glm::quat& baseFrame =
                                frame[static_cast<std::size_t>(chain[static_cast<std::size_t>(base)])];
                            // f(psi) = shin · hinge axis at knee K(psi); side > 0 = flexion side.
                            const auto evalAt = [&](float psi, float& side) {
                                const glm::vec3 K =
                                    center + (e1 * std::cos(psi) + e2 * std::sin(psi)) * rad;
                                const glm::vec3 upDir = glm::normalize(K - basePos);
                                glm::quat f = baseFrame;
                                for (int k = base - 1; k >= mid; --k) {
                                    const glm::vec3& restK = edgeRestDir[static_cast<std::size_t>(
                                        chain[static_cast<std::size_t>(k)])];
                                    f = glm::normalize(shortestArc(f * restK, upDir) * f);
                                }
                                const glm::vec3 shin = goal - K;
                                side = glm::dot(shin, f * tangent);
                                return glm::dot(shin, f * hingeRest);
                            };
                            constexpr int   kAzimuthSamples = 24;
                            constexpr float kPi = 3.14159265f;
                            const float step = 2.0f * kPi / static_cast<float>(kAzimuthSamples);
                            float bestPsi = 0.0f;
                            float bestCost = 1e30f;
                            bool  found = false;
                            float side = 0.0f;
                            float prevF = evalAt(-kPi, side);
                            for (int i = 1; i <= kAzimuthSamples; ++i) {
                                const float psi = -kPi + step * static_cast<float>(i);
                                const float fv = evalAt(psi, side);
                                if ((fv <= 0.0f) != (prevF <= 0.0f)) {
                                    float lo = psi - step;
                                    float hi = psi;
                                    float flo = prevF;
                                    for (int b = 0; b < 12; ++b) {
                                        const float m = 0.5f * (lo + hi);
                                        const float fm = evalAt(m, side);
                                        if ((fm <= 0.0f) == (flo <= 0.0f)) {
                                            lo = m;
                                            flo = fm;
                                        } else {
                                            hi = m;
                                        }
                                    }
                                    const float root = 0.5f * (lo + hi);
                                    evalAt(root, side);
                                    if (side > 0.0f) {
                                        // Flexion side. Of several (rare), the one nearest the
                                        // seed side above (pose continuity).
                                        const float cost = -std::cos(root);
                                        if (cost < bestCost) {
                                            bestCost = cost;
                                            bestPsi = root;
                                            found = true;
                                        }
                                    }
                                }
                                prevF = fv;
                            }
                            if (found) {
                                perp = e1 * std::cos(bestPsi) + e2 * std::sin(bestPsi);
                                perpLen = 1.0f;
                            }
                            static const bool kAzimuthTrace = std::getenv("IK_TWOBONE_TRACE") != nullptr;
                            if (kAzimuthTrace) {
                                std::printf("[2bone] eff=%d knee azimuth %s psi=%.1f deg (cur side dot %.3f)\n",
                                            e.node, found ? "found" : "NOT found",
                                            glm::degrees(bestPsi),
                                            glm::dot(glm::normalize(curMid - center), e1));
                            }
                        }
                    }
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
        chainTracing = (e.node == kChainTraceNode);
        if (chainTracing) {
            std::printf("    [chain] eff=%d len=%zu subBase=%d seedErr=%.4f chainLen=%.3f goalDist=%.3f\n",
                        e.node, chain.size(), chain.back(),
                        glm::length(positions[static_cast<std::size_t>(e.node)] - goal), chainLen,
                        glm::length(positions[static_cast<std::size_t>(chain.back())] - goal));
        }
        for (int round = 0; round < maxRounds && chainBestErr > settings.tolerance; ++round) {
            const float pinErr =
                glm::length(positions[static_cast<std::size_t>(e.node)] - goal);
            const bool chainStalled = round > 0 && pinErr > chainPrevErr - settings.tolerance;
            if (chainTracing) {
                std::printf("     [round %d] entryErr=%.4f stalled=%d\n", round, pinErr, chainStalled ? 1 : 0);
            }
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
        if (chainTracing) {
            std::printf("    [chain] eff=%d bestErr=%.4f\n", e.node, chainBestErr);
        }
        chainTracing = false;
        // Leave the limb in its best configuration, not the last round's — FRAMES included:
        // a later effector's chain may anchor at this effector's node (the "another effector"
        // sub-base case), and restoring positions while leaving the frames at a discarded
        // kicked round's values constrained that chain about an orientation this node no
        // longer holds.
        for (std::size_t m = 0; m < chain.size(); ++m) {
            positions[static_cast<std::size_t>(chain[m])] = chainBest[m];
            frame[static_cast<std::size_t>(chain[m])] = chainBestFrame[m];
        }
        if (kSolverTrace) {
            traceChainLen[ei] = chain.size();
            traceSubBase[ei] = chain.back();
            traceChainErr[ei] = chainBestErr;
        }
    };

    for (int iter = 0;
         iter < settings.maxIterations && (iter == 0 || error > settings.tolerance); ++iter) {
        const bool earlyPhase = iter < settings.maxIterations - 4;
        for (std::size_t ei = 0; ei < effectors.size(); ++ei) {
            const IkEffector& e = effectors[ei];
            const bool backsOff = !e.hard && (!e.pinned || e.node == root);
            // Back off toward the SOLVE-ENTRY position (not the node's current one, which the
            // early iterations already advanced — that "retreat" never retreated): at full
            // back-off the goal asks for no progress this solve at all.
            effTarget[ei] =
                (backsOff && goalBackoff > 0.0f && e.node >= 0 && e.node < n)
                    ? glm::mix(e.target, entryPositions[static_cast<std::size_t>(e.node)],
                               goalBackoff)
                    : e.target;
        }
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
                // the child->node direction; a branching joint takes the centroid — WEIGHTED
                // toward pin-serving branches at non-root joints (kPinPriority; the root keeps
                // the plain average — its reach is governed by the leashes). See servesPin.
                glm::vec3 centroid(0.0f);
                float weightSum = 0.0f;
                for (const int c : kids) {
                    const glm::vec3& childPos = positions[static_cast<std::size_t>(c)];
                    const glm::vec3 toNode = pos - childPos;
                    const float len = glm::length(toNode);
                    const float boneLen = lengthToParent[static_cast<std::size_t>(c)];
                    const float w =
                        (node != root && servesPin[static_cast<std::size_t>(c)]) ? kPinPriority
                                                                                  : 1.0f;
                    centroid += w * ((len > kZeroLength) ? childPos + toNode * (boneLen / len)
                                                         : childPos);
                    weightSum += w;
                }
                pos = centroid / weightSum;
            }
            const int e = effectorAt[static_cast<std::size_t>(node)];
            if (e >= 0) {
                // Effector: snap toward the target (leaf goals fully, interior soft goals by
                // weight so they bias the chain without hijacking it).
                const IkEffector& eff = effectors[static_cast<std::size_t>(e)];
                const float w = kids.empty() ? glm::clamp(eff.weight, 0.0f, 1.0f)
                                             : glm::clamp(eff.weight, 0.0f, 1.0f) * 0.5f;
                pos = glm::mix(pos, effTarget[static_cast<std::size_t>(e)], w);
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
        if (settings.floorClearance != nullptr &&
            static_cast<int>(settings.floorClearance->size()) == n) {
            // The root is placed by nobody: hold it above its own clearance.
            glm::vec3& rootPos = positions[static_cast<std::size_t>(root)];
            const float minY =
                settings.floorY + (*settings.floorClearance)[static_cast<std::size_t>(root)];
            if (rootPos.y < minY) {
                rootPos.y = minY;
            }
        }
        IK_NAN_CHECK("forward", iter);

        // --- Backward pass (root -> leaves): restore lengths and constraints from the root out.
        // The root FLOATS at whatever the forward pass proposed (the planted limbs hold it); a
        // PINNED root effector (a dragged pelvis, or the airborne-figure fallback) snaps instead.
        const int rootEffector = effectorAt[static_cast<std::size_t>(root)];
        if (rootEffector >= 0 && effectors[static_cast<std::size_t>(rootEffector)].pinned) {
            positions[static_cast<std::size_t>(root)] =
                effTarget[static_cast<std::size_t>(rootEffector)];
        }
        frame[static_cast<std::size_t>(root)] = frameSeed[static_cast<std::size_t>(root)];
        socketsRigid = false; // the main pass and the first restoration explore per edge
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
        // Hard pins first, through the trunk (see restoreChain); then everything else from its
        // sub-base — a goal whose chain hangs off a node the pins just moved adapts to them.
        // Sockets are FREE here (see socketsRigid): this pass DISCOVERS what each limb wants
        // of its socket — a lateral lean's pelvis roll is the mean of the two legs' wishes.
        for (std::size_t ei = 0; ei < effectors.size(); ++ei) {
            if (effectors[ei].hard) {
                restoreChain(ei, kRestoreRounds, earlyPhase, true);
            }
        }
        for (std::size_t ei = 0; ei < effectors.size(); ++ei) {
            if (!effectors[ei].hard) {
                restoreChain(ei, kRestoreRounds, earlyPhase, false);
            }
        }
        // Then RIGIDIFY: each group's rotation is fitted from where the free restoration left
        // its sockets (the stretch's rigid part — the roll — is kept, the rest removed) and
        // the children re-placed with it; every chain is restored once more from those rigid
        // sockets so the pins land on a configuration a rigid pelvis realizes. Rigidity only
        // in the main pass never found the roll (the two sockets ask for opposite rotations
        // and a fit from the forward pass's proposals has none): the lean then went into the
        // knees, which flipped every tick.
        if (kRigidGroups) {
            socketsRigid = true;
            std::fill(placedInGroup.begin(), placedInGroup.end(), 0);
            for (const int node : order) {
                if (node == root || !active[static_cast<std::size_t>(node)] ||
                    placedInGroup[static_cast<std::size_t>(node)]) {
                    continue;
                }
                const int p = graph.parentOf(node);
                if (isGrouped(p)) {
                    placeGroup(p);
                }
            }
            for (std::size_t ei = 0; ei < effectors.size(); ++ei) {
                if (effectors[ei].hard) {
                    restoreChain(ei, kRestoreRounds, false, true);
                }
            }
            for (std::size_t ei = 0; ei < effectors.size(); ++ei) {
                if (!effectors[ei].hard) {
                    restoreChain(ei, kRestoreRounds, false, false);
                }
            }
        }

        IK_NAN_CHECK("restore", iter);
        traceIters = iter + 1;
        if (anyHardPin && worstHardPinError() > kHardPinTol) {
            goalBackoff = std::min(1.0f, goalBackoff + kGoalBackoffStep);
        }
        error = worstError();
        if (error < bestError) {
            bestError = error;
            bestPositions = positions;
            bestFrame = frame;
        }
    }
    if (!bestPositions.empty()) {
        positions = bestPositions;
        frame = bestFrame;
    }
    // HARD-PIN POLISH (see kHardPinPolishRounds): one well-converged restoration of each hard
    // pin's chain on the final state — no kick (pure), frames consistent with the kept state,
    // sockets rigid (the kept state is a rigidified one).
    if (anyHardPin) {
        socketsRigid = true;
        for (std::size_t ei = 0; ei < effectors.size(); ++ei) {
            if (effectors[ei].hard) {
                restoreChain(ei, kHardPinPolishRounds, false, true);
            }
        }
        bestError = worstError();
    }
    if (kSolverTrace) {
        std::printf("    [solver] iters=%d backoff=%.2f best=%.4f\n", traceIters, goalBackoff,
                    bestError);
        for (std::size_t ei = 0; ei < effectors.size(); ++ei) {
            const IkEffector& e = effectors[ei];
            const float err = (e.node >= 0 && e.node < n)
                                  ? glm::length(positions[static_cast<std::size_t>(e.node)] - e.target)
                                  : -1.0f;
            std::printf("      eff node=%d pinned=%d hard=%d w=%.2f err=%.4f chainLen=%zu "
                        "subBase=%d chainErr=%.4f\n",
                        e.node, e.pinned ? 1 : 0, e.hard ? 1 : 0, e.weight, err,
                        traceChainLen[ei], traceSubBase[ei], traceChainErr[ei]);
        }
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
