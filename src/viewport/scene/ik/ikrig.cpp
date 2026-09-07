#include "ikrig.h"

#include "balancecontroller.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace pose {

namespace {

// Contacts within this GRAPH PATH LENGTH (summed bone rest lengths, world units) of the dragged
// effector are released rather than pinned: dragging a foot lifts the foot and its entire toe
// subtree (~0.15m away) instead of fighting its own pins, and a shin/knee drag frees the foot
// below it (~0.5m), while a hip/pelvis drag keeps both feet planted (~1m away — the crouch).
// METRIC, not hop-count: hop rules break both ways on real rigs — the trunk packs tiny 2cm
// bones (a thigh is only 2 hops from the hip), while a foot's small-toe tip segments are 3-4
// hops from the ankle, and leaving THOSE pinned made a foot drag fight its own toe pins (the
// solver's compromise slid the foot sideways off the user's target).
constexpr float kEffectorLiftReach = 0.65f;

// How far inside the support polygon the CoM must sit to count as balanced (world units).
constexpr float kBalanceMargin = 0.03f;

// The balance pelvis effector is a soft goal: bias the chain without hijacking the user's drag.
constexpr float kPelvisWeight = 0.6f;

// When the drag target and every pin are already within this of their joints, the move is a
// no-op: skip the solve entirely. Without this, holding the cursor still keeps running
// solve -> extract cycles whose millimeter residuals random-walk (and once ratcheted) the pose.
constexpr float kGoalDeadband = 2e-3f;

// A node participates as a DANGLING segment (suspension) only with REAL segment mass — the
// balance mass model classifies body parts; face/finger/helper bones get token mass and ride
// rigidly (gravity once pulled the hinged JAW open while the figure dangled). NOT used for
// contact eligibility: individual toe bones split the "toe" mass share far below any threshold,
// and dropping them shrank the support polygon enough to destabilize balance during arm drags.
constexpr float kRealMassThreshold = 0.003f;

// Token-limb promotion thresholds (see beginDrag). A grabbed joint promotes to its limb's first
// real-mass joint only while BOTH hold: its OWN segment mass is token (below kTokenBoneMass —
// fingers/carpals/face bones get the ~0.0015 unclassified mass and split toe shares run ~0.0004,
// while classified end joints like a hand at 0.006 or metatarsals at 0.005 do not qualify), and
// its SUBTREE mass is token too (below kTokenLimbMass — a wrist's subtree carries the whole hand
// ~0.03, so a mid-limb twist-bone grab stays where the user grabbed). The own-mass test is what
// keeps a FINGERLESS rig's leaf hand from promoting up the arm.
constexpr float kTokenBoneMass = 0.005f;
constexpr float kTokenLimbMass = 0.01f;

// Low-pass factor for the smoothed balance correction (see m_balanceCorrection in the header):
// the raw need flickers at tick scale as the CoM dances on the polygon's inset boundary, and
// the engagement blend must never see that flicker.
constexpr float kBalanceSmoothing = 0.25f;

// Smoothed-correction magnitude at which balance reaches full engagement: below it the balanced
// re-solve's influence (the output blend AND the correction target) ramps linearly from zero,
// so the balance response is CONTINUOUS in the CoM position — no engagement boundary for the
// solve to flip across. See the balance block in solveDrag for the full design rationale.
constexpr float kBalanceRamp = 0.02f;

// Solver output deadband for interactive drag solves (FabrikSolver::Settings::
// minStepDisplacement): kills the sub-millimeter solve->extract->FK limit cycle that reads as
// idle joints SHIMMERING while the user pulls. Under the smallest governor allowance (~3mm per
// tick), real corrections always propose more than this.
constexpr float kOutputDeadband = 8e-4f;

// BALANCE-DRIVEN STEPPING (see updateStepping): when the post-solve balance need stays above
// kStepNeedThreshold for kStepConfirmTicks consecutive ticks — the drag holds the CoM off the
// support polygon harder than leaning can absorb — the figure re-plants a foot at its BALANCED
// position: the drag-start stance shape re-centered on the current CoM. Only standing feet
// step (a pin whose target sits at its bind height, i.e. ground-healed), only with at least
// two of them (stepping the single support would be a fall), and only for a re-plant of at
// least kStepMinDistance. The swing is an eased glide of the pin target with a sine lift arc,
// paced at ~kStepSpeed per tick; sequential steps under a continuing drag read as the figure
// WALKING to follow it.
// 0.025 (was 0.035): recalibrated with the Armature's per-tick FLOOR LIFT (see
// kFloorLiftTol in armatureik.cpp). Before it, a hard lateral lean sank the far foot through
// the floor, and that sink inflated the strain signal (kStepPinErrThreshold) enough to step; with
// the feet held on the floor the honest lateral slip stays under the strain threshold and a 45cm
// lateral chest drag leaned the hip 19cm on planted feet without ever stepping — a person would
// have stepped long before. The balance-EFFORT path is the principled trigger for that lean (the
// pelvis correction fights the imposed lean every tick), and 0.025 restores the steps (3, the
// chest within 13cm of a 45cm target, was 22cm) while every other harness phase — the hip walks,
// the strained pull, the pinned-foot chest drag, the crouches — stays bit-identical. Lowering the
// strain threshold instead (2.5cm) also stepped, but reshaped the walks' step counts.
constexpr float kStepNeedThreshold = 0.025f;
constexpr float kStepPinErrThreshold = 0.04f; ///< Entry pin error = a foot dragged off its plant.
constexpr float kStepStanceThreshold = 0.12f; ///< Explicit-anchor drags: stance error that steps.
constexpr int   kStepConfirmTicks = 12;
constexpr float kStepMinDistance = 0.06f;
constexpr float kStepSpeed = 0.015f;
constexpr int   kStepMinTicks = 12;
constexpr int   kStepMaxTicks = 30;
constexpr float kStepHeightFactor = 0.25f;
constexpr float kStepHeightMin = 0.02f;
constexpr float kStepHeightMax = 0.08f;
constexpr float kStepClearance = 0.10f; ///< Min XZ distance between a landing and another pin.

// SUSPENSION (lift-off) entry: the pull must be mostly vertical (y > this fraction of the
// strain), farther beyond reach than kSuspendStrain, with the root hard against its leash, for
// kSuspendConfirmTicks consecutive ticks (~0.25 s) — a deliberate sustained lift, never a fast
// lateral gesture. kSuspendGravity is the per-iteration downward bias that makes the released
// body hang below the grab point (see FabrikSolver::Settings::gravityBias).
constexpr float kSuspendStrain = 0.15f;
constexpr float kSuspendUpFraction = 0.65f;
constexpr int   kSuspendConfirmTicks = 15;
constexpr float kSuspendGravity = 0.012f;

// Root prior yield against downward drags (see FabrikSolver::Settings::rootDownYield): pushing
// the chest (or pulling a hand) DOWN crouches the body — the pelvis gives vertically and the
// legs fold onto the pinned feet — while lateral/upward root stiffness stays full.
constexpr float kRootDownYield = 0.25f;

// Trunk stiffness multiplier under UPWARD drag intent (see m_trunkChain): pulling a hand up is
// arm + shoulder-girdle work — a spine cannot lengthen — so the spine holds its posture instead
// of being recruited (recruiting it pitched the chest and swung the head down into a bow).
constexpr float kUpTrunkStiffen = 4.0f;

// Strength of the solver's soft pose prior (see FabrikSolver::solve): how hard interior joints
// ease back toward the drag-start pose each iteration. Enough to make the body settle to the
// closest-to-start pose that satisfies the goals (without it, per-mouse-event solves ratcheted
// the trunk into contortions); small enough that effector chains still recruit the body when a
// reach genuinely needs it.
constexpr float kPosePriorWeight = 0.1f;

// Solver iterations per tick. Deliberately LOW-ish: the drag runs at ~60 ticks/s, and
// convergence is meant to happen ACROSS ticks — the visible motion IS the iteration process,
// evolving smoothly like a damped system. A large per-tick budget re-resolves the whole
// configuration every event, and near-equal configurations flip with millimeter target changes
// (cursor noise), which the user sees as trembling. Every solve passes
// kPriorIterationNorm = (the 6 iterations the stiffness model was tuned at) / kIterationsPerTick
// so raising the budget sharpens CONVERGENCE without silently stiffening the per-solve prior
// (unnormalized, 10 iterations compounded the prior ~1.7x and broke hover-healing).
constexpr int   kIterationsPerTick = 10;
constexpr float kPriorIterationNorm = 6.0f / static_cast<float>(kIterationsPerTick);

} // namespace

void IkRig::build(const std::vector<IkRigBone>& bones) {
    const std::size_t n = bones.size();
    m_parents.assign(n, -1);
    m_bindPos.assign(n, glm::vec3(0.0f));
    m_edgeRestDir.assign(n, glm::vec3(0.0f, 1.0f, 0.0f));
    m_edgeRestLen.assign(n, 0.0f);
    m_edgeConstraint.assign(n, JointConstraint{});
    m_pelvis = -1;

    std::vector<std::string> names(n);
    int anatomicalRoot = -1;
    for (std::size_t i = 0; i < n; ++i) {
        m_parents[i] = bones[i].parent;
        m_bindPos[i] = bones[i].bindPos;
        names[i] = bones[i].name;
        if (bones[i].parent < 0 && anatomicalRoot < 0) {
            anatomicalRoot = static_cast<int>(i);
        }
        if (bones[i].parent >= 0 && static_cast<std::size_t>(bones[i].parent) < n) {
            const IkRigBone& parent = bones[static_cast<std::size_t>(bones[i].parent)];
            const glm::vec3 offset = bones[i].bindPos - parent.bindPos;
            const float len = glm::length(offset);
            m_edgeRestLen[i] = len;
            if (len > 1e-6f) {
                m_edgeRestDir[i] = offset / len;
            }
            // The edge parent->bone is articulated by the PARENT joint's rotation channels.
            m_edgeConstraint[i] = deriveJointConstraint(parent.orientAxes, m_edgeRestDir[i],
                                                        parent.rotMinDeg, parent.rotMaxDeg,
                                                        parent.rotLimited);
        }
    }
    // The FBIK root ("the pelvis") is the first MULTI-CHILD descendant of the anatomical root,
    // not the anatomical root itself: real figures root at a FIGURE NODE sitting at the ORIGIN
    // (floor level) with the hip as its lone child a meter up. Rooting the solve there aimed
    // every root mechanism at the wrong point — reach leashes measured ~2m paths and never bound
    // (the single-pull float), the root stiffness pinned a point on the floor, and balance
    // targeted the origin. The walk is generation-agnostic; the figure node stays outside the
    // active solve and rides along.
    m_pelvis = anatomicalRoot;
    std::vector<int> childCount(n, 0);
    std::vector<int> onlyChild(n, -1);
    for (std::size_t i = 0; i < n; ++i) {
        if (bones[i].parent >= 0) {
            ++childCount[static_cast<std::size_t>(bones[i].parent)];
            onlyChild[static_cast<std::size_t>(bones[i].parent)] = static_cast<int>(i);
        }
    }
    if (m_pelvis >= 0) {
        int walk = m_pelvis;
        while (walk >= 0 && childCount[static_cast<std::size_t>(walk)] == 1) {
            walk = onlyChild[static_cast<std::size_t>(walk)];
        }
        // A skeleton that never branches (a pure chain: a tail/snake rig, a minimal test rig)
        // has no multi-child descendant — the walk lands on the LEAF, and rooting there would
        // flag every other bone as a non-body "figure-node ancestor". Such a rig keeps its
        // anatomical root as the FBIK root instead.
        if (walk >= 0 && childCount[static_cast<std::size_t>(walk)] > 1) {
            m_pelvis = walk;
        }
    }

    // The pelvis's strict ancestors — the figure-node chain — are NOT body parts, and they are
    // excluded from the rig wholesale: no graph edge (the hip→origin link is not a bone the
    // solver may compress), no ground-contact eligibility (the figure node sits ON the floor, so
    // contact detection planted it and pinned the whole figure to the origin — the solver's
    // per-tick compromise between that phantom pin and the real leg pins ratcheted the hip
    // floorward, the single-pull whole-body sink), and no balance mass.
    m_bodyNode.assign(n, 1);
    std::vector<int> graphParents = m_parents;
    if (m_pelvis >= 0) {
        graphParents[static_cast<std::size_t>(m_pelvis)] = -1;
        for (int cur = anatomicalRoot; cur >= 0 && cur != m_pelvis;
             cur = onlyChild[static_cast<std::size_t>(cur)]) {
            m_bodyNode[static_cast<std::size_t>(cur)] = 0;
        }
    }
    m_graph.build(graphParents);
    // Root the graph at the pelvis right away: build() defaults to the first parentless node,
    // which after the exclusion above can be the (isolated) figure node. beginDrag() re-asserts
    // this per drag, but a freshly-built rig should never sit in a nonsensical rooting.
    if (m_pelvis >= 0) {
        m_graph.setRoot(m_pelvis);
    }
    // TWIST FREEDOM for the bend joints (see JointConstraint::twistAxis): the edge J->C
    // articulated by joint J may rotate its bend plane about the segment T->J within T's
    // authored twist range, T being J's parent and its twist axis the oriented axis most
    // parallel to that segment (a mid-limb twist bone's one free channel; on rigs without twist
    // bones, the limb root's own twist channel). The range is expressed about +T->J, so a twist
    // axis authored pointing the other way flips it. The pelvis (never rotated by the solve) and
    // the figure-node chain contribute none. Realized by the Model's twist witness.
    for (std::size_t c = 0; c < n; ++c) {
        JointConstraint& jc = m_edgeConstraint[c];
        if (!bendIsHingeLike(jc)) {
            continue; // only single-plane benders need (or can use) a movable fold plane
        }
        // Near-hinge cones (the knees) take the freedom too: a deep crouch drops and rolls the
        // pelvis, and a knee that can fold only in its rest plane cannot keep its foot planted
        // — the solver's own chain error on one foot reached 11cm (the foot buried to the ankle
        // during a chest push-down). First tried before the landing round, when it perturbed
        // a strained release; the leash cap and the contact refinement now absorb that.
        const int j = m_parents[c];
        if (j < 0) {
            continue;
        }
        const int t = m_parents[static_cast<std::size_t>(j)];
        if (t < 0 || !m_bodyNode[static_cast<std::size_t>(t)] || t == m_pelvis) {
            continue;
        }
        if (!edgeRigid(j)) {
            continue; // T must be a PURE twist bone (swings locked): its one channel IS the twist
        }
        const glm::vec3 seg = m_bindPos[static_cast<std::size_t>(j)] - m_bindPos[static_cast<std::size_t>(t)];
        const float segLen = glm::length(seg);
        if (segLen < 1e-6f) {
            continue;
        }
        const glm::vec3 u = seg / segLen;
        const IkRigBone& tb = bones[static_cast<std::size_t>(t)];
        int twist = 0;
        float bestDot = -1.0f;
        float signedDot = 0.0f;
        for (int a = 0; a < 3; ++a) {
            const float d = glm::dot(glm::normalize(tb.orientAxes[a]), u);
            if (std::abs(d) > bestDot) {
                bestDot = std::abs(d);
                signedDot = d;
                twist = a;
            }
        }
        if (bestDot < 0.9f) {
            continue; // the channel would swing the segment more than spin it: not a twist
        }
        float lo = glm::radians(-90.0f);
        float hi = glm::radians(90.0f);
        if (tb.rotLimited[twist]) {
            if (tb.rotMaxDeg[twist] - tb.rotMinDeg[twist] < 2.0f) {
                continue; // locked twist: no freedom
            }
            lo = glm::radians(tb.rotMinDeg[twist]);
            hi = glm::radians(tb.rotMaxDeg[twist]);
        }
        if (signedDot < 0.0f) {
            const float flippedLo = -hi;
            hi = -lo;
            lo = flippedLo;
        }
        jc.twistAxis = u;
        jc.twistMin = lo;
        jc.twistMax = hi;
    }
    m_edgeTwistBuilt.assign(n, glm::vec2(0.0f));
    for (std::size_t c = 0; c < n; ++c) {
        const JointConstraint& jc = m_edgeConstraint[c];
        if (jc.type == JointConstraint::Type::Cone && jc.twistMax - jc.twistMin > 1e-6f) {
            m_edgeTwistBuilt[c] = glm::vec2(jc.twistMin, jc.twistMax);
        }
    }
    m_masses = BalanceController::assignMasses(names);
    for (std::size_t i = 0; i < n; ++i) {
        if (!m_bodyNode[i]) {
            m_masses[i] = 0.0f;
        }
    }
    // Per-node subtree mass (own + all descendants): the token-limb promotion and the
    // trunk-junction search read it. Figure skeletons list parents before children (hierarchy
    // order), so one reverse pass accumulates leaves into roots.
    m_subtreeMass = m_masses;
    for (std::size_t i = n; i-- > 1;) {
        const int p = m_parents[i];
        if (p >= 0) {
            m_subtreeMass[static_cast<std::size_t>(p)] += m_subtreeMass[i];
        }
    }
    // Figure size relative to the adult reference every world-space threshold here was tuned
    // on: the pelvis bind height (~1m standing). A 0.54-scale child's pelvis-to-foot path is
    // ~0.5m — less than the ADULT contact-release reach — so unscaled thresholds released the
    // child's feet on a hip drag and the figure translated rigidly instead of crouching. The
    // dead zone keeps every adult rig (real figures' pelvis heights vary ~1.0-1.1) at EXACTLY
    // the verified 1.0 tuning; only genuinely small/large figures engage the scaling.
    m_sizeScale = 1.0f;
    if (m_pelvis >= 0 && static_cast<std::size_t>(m_pelvis) < n) {
        const float raw = m_bindPos[static_cast<std::size_t>(m_pelvis)].y;
        if (raw <= 0.85f || raw >= 1.15f) {
            m_sizeScale = glm::clamp(raw, 0.25f, 2.5f);
        }
    }
    // Floor clearances (see floorClearance()): a joint that rests within the contact band of
    // the floor keeps its rest height (the ankle above the sole, the toes just off the ground);
    // any other joint gets a flesh radius. Relative to the bind floor — the per-drag ground
    // offset is added by the solve.
    m_floorClearance.assign(n, 0.0f);
    for (std::size_t i = 0; i < n; ++i) {
        if (!m_bodyNode[i]) {
            continue;
        }
        const float bindY = m_bindPos[i].y;
        m_floorClearance[i] = bindY < 0.15f * m_sizeScale ? std::max(bindY, 0.0f)
                                                          : 0.015f * m_sizeScale;
    }
}

float IkRig::pathLenToRoot(int node) const {
    float len = 0.0f;
    for (int cur = node; cur >= 0 && cur != m_pelvis;
         cur = m_parents[static_cast<std::size_t>(cur)]) {
        len += m_edgeRestLen[static_cast<std::size_t>(cur)];
    }
    return len;
}

float IkRig::socketLeash(int node, const glm::vec3& target,
                         const std::vector<glm::vec3>& positions, glm::vec3& offsetOut) const {
    const glm::vec3& rootPos = positions[static_cast<std::size_t>(m_pelvis)];
    int socket = node;
    while (socket >= 0 && m_parents[static_cast<std::size_t>(socket)] >= 0 &&
           m_parents[static_cast<std::size_t>(socket)] != m_pelvis) {
        socket = m_parents[static_cast<std::size_t>(socket)];
    }
    if (socket < 0 || m_parents[static_cast<std::size_t>(socket)] != m_pelvis) {
        // The pin is the root itself (or above it): a plain root-centered ball.
        offsetOut = glm::vec3(0.0f);
        return std::max(glm::length(rootPos - target), 0.9f * pathLenToRoot(node));
    }
    const glm::vec3& socketPos = positions[static_cast<std::size_t>(socket)];
    offsetOut = socketPos - rootPos;
    const float limb = pathLenToRoot(node) - m_edgeRestLen[static_cast<std::size_t>(socket)];
    // Never beyond what the limb can SPAN (its straight rest path): a leash measured from an
    // overstretched stance — a step landed while its leg could not yet reach the spot — recorded
    // the impossible distance as the allowed maximum, and two such balls pinned the root
    // between them (every forward-pass move toward one foot left the other's ball and was
    // projected straight back), so the release settle could never bring the feet down: the
    // figure stood 4cm in the air after every stepping chest drag. Capped, the projection pulls
    // the root back INTO reach and the feet plant.
    return std::min(std::max(glm::length(socketPos - target), 0.9f * limb), 0.995f * limb);
}

bool IkRig::beginDrag(int effectorNode, const std::vector<glm::vec3>& positions,
                      const std::vector<int>& contactNodes, float groundOffsetY,
                      const std::vector<int>* userPins) {
    const int n = m_graph.nodeCount();
    if (!built() || m_pelvis < 0 || effectorNode < 0 || effectorNode >= n ||
        static_cast<int>(positions.size()) != n ||
        !m_bodyNode[static_cast<std::size_t>(effectorNode)]) {
        m_effector = -1;
        return false;
    }
    m_effector = effectorNode;
    m_groundOffsetY = groundOffsetY;
    // Knee fold-plane freedom per drag (see m_edgeTwistBuilt): on for trunk gestures and for
    // the dragged limb's own knee, off for other limb drags.
    {
        const bool trunkDrag = effectorIsTrunk();
        std::vector<char> onEffectorChain(static_cast<std::size_t>(n), 0);
        for (int cur = m_effector; cur >= 0 && cur != m_pelvis;
             cur = m_parents[static_cast<std::size_t>(cur)]) {
            onEffectorChain[static_cast<std::size_t>(cur)] = 1;
        }
        for (std::size_t c = 0; c < m_edgeTwistBuilt.size(); ++c) {
            const glm::vec2& built = m_edgeTwistBuilt[c];
            if (built.y - built.x <= 1e-6f) {
                continue;
            }
            const bool enabled = trunkDrag || onEffectorChain[c];
            m_edgeConstraint[c].twistMin = enabled ? built.x : 0.0f;
            m_edgeConstraint[c].twistMax = enabled ? built.y : 0.0f;
        }
    }

    // Per-node subtree mass (own + all descendants), used twice below: the token-limb
    // rigidification and the trunk-junction search (built once with the masses).
    const std::vector<float>& subtreeMass = m_subtreeMass;

    // TOKEN-LIMB PROMOTION: when the grabbed joint sits in a token-mass extremity (clicking near
    // a hand almost always picks a FINGER; a face grab picks a nose/brow bone), the drag
    // EFFECTOR is promoted to the limb's first real-mass joint (the hand, the foot, the head) —
    // the Model compensates the target by the grab offset, so the drag still tracks where the
    // user grabbed. This is what makes a finger pull behave as an ARM gesture, the way a person
    // raises an arm when tugged by one finger; the digits, no longer on the effector path, ride
    // along rigidly like any inactive subtree, and fingers stay posable by FK (plain drag /
    // gizmo). Solving with the FINGERTIP as the effector was tried in two forms and both left
    // the hand far short of a reachable overhead target: free finger joints absorbed the goal
    // (the knuckle hyperextended to its +50° limit while the arm stayed near-horizontal), and
    // per-drag RIGIDIFIED finger edges still equilibrated ~25cm short — the fingertip goal
    // under-recruited the collar (14° vs the hand-drag's 35° shrug), and no stiffness or prior
    // variant closed that gap. Driving the limb's real end joint IS the verified arm-raise path.
    while (m_effector != m_pelvis) {
        const int p = m_parents[static_cast<std::size_t>(m_effector)];
        if (p < 0 || m_masses[static_cast<std::size_t>(m_effector)] >= kTokenBoneMass ||
            subtreeMass[static_cast<std::size_t>(m_effector)] > kTokenLimbMass) {
            break;
        }
        m_effector = p;
    }

    // Planted contacts: everything grounded, minus what the drag itself is lifting (contacts
    // within kEffectorLiftReach of the effector, measured as tree path length — see the constant
    // above), and minus non-body nodes (the origin-level figure node reads as "on the floor"
    // every frame). The path length is computed via the effector's ancestor chain (LCA): the
    // skeleton is a tree, so the effector->contact path runs up one side and down the other.
    std::vector<int>   effAncestor;   // effector, its parent, ... up to the anatomical top
    std::vector<float> effAncestorLen; // cumulative edge length from the effector to each
    {
        float acc = 0.0f;
        for (int cur = m_effector; cur >= 0; cur = m_parents[static_cast<std::size_t>(cur)]) {
            effAncestor.push_back(cur);
            effAncestorLen.push_back(acc);
            if (m_parents[static_cast<std::size_t>(cur)] >= 0) {
                acc += m_edgeRestLen[static_cast<std::size_t>(cur)];
            }
        }
    }
    const auto pathLengthToEffector = [&](int c) {
        float acc = 0.0f;
        for (int cur = c; cur >= 0; cur = m_parents[static_cast<std::size_t>(cur)]) {
            for (std::size_t a = 0; a < effAncestor.size(); ++a) {
                if (effAncestor[a] == cur) {
                    return acc + effAncestorLen[a]; // met the effector's chain: cur is the LCA
                }
            }
            acc += m_edgeRestLen[static_cast<std::size_t>(cur)];
        }
        return 1e30f; // disconnected (excluded figure-node chain)
    };
    // A contact must DESCEND FROM THE PELVIS (the anatomical body tree). Merged follower
    // figures' scene nodes are appended as SIBLINGS of the pelvis under the figure node and sit
    // at the ORIGIN (y=0) — an eyelash follower's scene node read as "planted" and was pinned
    // there, the round-8 phantom-anchor bug returning through the addon path (its pin stalled
    // every drag and anchored the figure to the origin). Structure, not mass, is the test: the
    // individual toe bones split the "toe" mass share to near-token values, and a mass gate that
    // caught the follower node also dropped the toes from the support polygon (a smaller
    // footprint destabilized balance during ordinary arm drags).
    auto descendsFromPelvis = [&](int node) {
        for (int cur = m_parents[static_cast<std::size_t>(node)]; cur >= 0;
             cur = m_parents[static_cast<std::size_t>(cur)]) {
            if (cur == m_pelvis) {
                return true;
            }
        }
        return false;
    };
    std::vector<int> planted;
    for (const int c : contactNodes) {
        if (c >= 0 && c < n && c != m_effector && m_bodyNode[static_cast<std::size_t>(c)] &&
            descendsFromPelvis(c) &&
            pathLengthToEffector(c) > kEffectorLiftReach * m_sizeScale) {
            planted.push_back(c);
        }
    }

    // The solve traverses the ANATOMICAL hierarchy (rooted at the pelvis) so every constraint is
    // evaluated in its owning parent-side frame; ground anchoring is the pins' job. The pelvis
    // root FLOATS, held by the planted limbs — its solved displacement becomes the Model's root
    // pose translation — except when nothing is planted (an airborne figure): then the root is
    // pinned at its current position, the classic anchored-pelvis fallback.
    m_graph.setRoot(m_pelvis);

    // Only the MOST PROXIMAL planted contact of each limb becomes a pin (the ankle — a contact
    // none of whose ancestors are themselves planted): pinning every toe/metatarsal joint
    // individually made the solver BEND the foot and toe joints to hold their pins whenever the
    // ankle shifted a few millimeters — visible toe curling and tiptoeing on gentle drags. With
    // only the ankle pinned, the foot and toes stay off the active set and ride along rigidly,
    // keeping their shape. The support polygon still spans ALL contacts (toes give the real
    // footprint). Each pin carries its reach-leash radius (see FabrikSolver): the DRAG-START
    // root-to-pin distance — a stance the figure provably held — with fractional path-length
    // headroom so a bent-leg start may still extend.
    m_pins.clear();
    std::vector<glm::vec2> support;
    support.reserve(planted.size());
    const glm::vec3& rootStart = positions[static_cast<std::size_t>(m_pelvis)];
    float snapSum = 0.0f;
    int snapCount = 0;
    for (const int c : planted) {
        // GROUND-HEALING pin targets: a contact hovering NEAR its natural floor height is pinned
        // AT that height (bind y), not where it currently hangs — every drag then re-plants
        // slightly-lifted feet instead of preserving (and compounding) the hover: pins that
        // anchored "wherever the foot is" let per-drag residuals stack until the figure floated.
        // Bounded: a joint far from its bind height (a kneeling knee, a deliberately airborne
        // pose) keeps its current height.
        glm::vec3 target = positions[static_cast<std::size_t>(c)];
        const float floorY = m_bindPos[static_cast<std::size_t>(c)].y + m_groundOffsetY;
        if (std::abs(target.y - floorY) < 0.12f * m_sizeScale) {
            snapSum += floorY - target.y;
            ++snapCount;
            target.y = floorY;
        }
        support.emplace_back(target.x, target.z);
        bool ancestorPlanted = false;
        for (int cur = m_parents[static_cast<std::size_t>(c)]; cur >= 0 && !ancestorPlanted;
             cur = m_parents[static_cast<std::size_t>(cur)]) {
            for (const int other : planted) {
                if (other == cur) {
                    ancestorPlanted = true;
                    break;
                }
            }
        }
        if (ancestorPlanted) {
            continue;
        }
        IkEffector pin{c, target, true, 1.0f, -1.0f};
        pin.leashRadius = socketLeash(c, target, positions, pin.leashOffset);
        m_pins.push_back(pin);
    }
    // USER pins: joints the user explicitly pinned are held exactly where they are — no ground
    // healing (the pin is wherever the user left it), no lift-reach release, and they are never
    // stepped (a pinned foot is intent, even when it stands on the ground). A pin on the
    // effector or inside its subtree would fight the drag itself, so it sits out this drag:
    // dragging a pinned hand simply moves the pin. A user pin that coincides with a contact
    // pin takes the contact's place with user semantics.
    m_pinUser.assign(m_pins.size(), 0);
    m_userPinLimb.assign(static_cast<std::size_t>(n), 0);
    std::size_t userPinCount = 0;
    if (userPins != nullptr) {
        for (const int u : *userPins) {
            if (u < 0 || u >= n || u == m_effector || !m_bodyNode[static_cast<std::size_t>(u)]) {
                continue;
            }
            // The subtree exclusion is for LIMB drags (a pinned toe under a dragged foot would
            // fight the drag). A PELVIS drag has the whole body in its subtree — there the user
            // pins are exactly the anchors the drag must respect (a crouch under a held hand).
            bool inEffectorSubtree = false;
            if (m_effector != m_pelvis) {
                for (int cur = m_parents[static_cast<std::size_t>(u)]; cur >= 0;
                     cur = m_parents[static_cast<std::size_t>(cur)]) {
                    if (cur == m_effector) {
                        inEffectorSubtree = true;
                        break;
                    }
                }
            }
            if (inEffectorSubtree) {
                continue;
            }
            const glm::vec3& held = positions[static_cast<std::size_t>(u)];
            IkEffector userPin{u, held, true, 1.0f, -1.0f};
            userPin.leashRadius = socketLeash(u, held, positions, userPin.leashOffset);
            userPin.hard = true; // beats every goal (see the solver's hard-pin policy)
            bool replaced = false;
            for (std::size_t p = 0; p < m_pins.size(); ++p) {
                if (m_pins[p].node == u) {
                    m_pins[p] = userPin;
                    m_pinUser[p] = 1;
                    replaced = true;
                    break;
                }
            }
            if (!replaced) {
                m_pins.push_back(userPin);
                m_pinUser.push_back(1);
            }
            ++userPinCount;
            // The pin's limb chain (see userPinLimbNodes()): pin up to, excluding, its junction.
            // The trunk keeps its prior even under a pelvis drag: exempting it let the hips drop
            // further while the held hand drifted MORE (3.2 -> 3.8cm) — the spine's damping is
            // what the pin's restoration works against.
            const int junction = limbJunction(u, subtreeMass);
            for (int cur = u; cur >= 0 && cur != junction;
                 cur = m_parents[static_cast<std::size_t>(cur)]) {
                m_userPinLimb[static_cast<std::size_t>(cur)] = 1;
            }
        }
    }
    // Airborne fallback: nothing planted AND nothing user-pinned — pin the root in place (a
    // user-pinned hand is an anchor in its own right; the body may swing from it).
    if (planted.empty() && userPinCount == 0 && m_effector != m_pelvis) {
        m_pins.push_back({m_pelvis, positions[static_cast<std::size_t>(m_pelvis)], true, 1.0f});
        m_pinUser.push_back(0);
    }
    m_supportHull = BalanceController::supportPolygon(std::move(support));
    m_balanceCorrection = glm::vec2(0.0f);
    m_suspended = false;
    m_suspendTicks = 0;
    // Pins snapped DOWN (bind below current => negative snap sum): a hovering figure heals.
    m_healing = snapCount > 0 && snapSum < -0.005f;

    // Stepping state (see updateStepping): per-pin footprints (each planted contact's XZ offset
    // from its owning pin — the pin it, or its nearest planted ancestor, resolved to), which
    // pins are steppable standing feet, and the stance SHAPE (each steppable pin's offset from
    // the stance center) that a step re-centers on the CoM.
    m_stepPin = -1;
    m_stepTick = 0;
    m_stepTotal = 0;
    m_imbalanceTicks = 0;
    m_stepsTaken = 0;
    m_pinFootprint.assign(m_pins.size(), {});
    for (const int c : planted) {
        int owner = -1;
        for (int cur = c; cur >= 0 && owner < 0; cur = m_parents[static_cast<std::size_t>(cur)]) {
            for (std::size_t p = 0; p < m_pins.size(); ++p) {
                if (m_pins[p].node == cur) {
                    owner = static_cast<int>(p);
                    break;
                }
            }
        }
        if (owner >= 0) {
            const glm::vec3& t = m_pins[static_cast<std::size_t>(owner)].target;
            m_pinFootprint[static_cast<std::size_t>(owner)].push_back(
                glm::vec2(positions[static_cast<std::size_t>(c)].x,
                          positions[static_cast<std::size_t>(c)].z) -
                glm::vec2(t.x, t.z));
        }
    }
    m_pinSteppable.assign(m_pins.size(), 0);
    m_pinStanceOffset.assign(m_pins.size(), glm::vec2(0.0f));
    {
        glm::vec2 stanceCenter(0.0f);
        int stanceCount = 0;
        for (std::size_t p = 0; p < m_pins.size(); ++p) {
            const int node = m_pins[p].node;
            // A steppable pin is a STANDING FOOT: a low joint whose pin target sits at its bind
            // height (ground-healed). A kneeling knee or an airborne-kept contact never steps.
            if (m_pins.size() >= 2 && !m_pinUser[p] &&
                m_bindPos[static_cast<std::size_t>(node)].y < 0.2f * m_sizeScale &&
                std::abs(m_pins[p].target.y -
                         (m_bindPos[static_cast<std::size_t>(node)].y + m_groundOffsetY)) <
                    0.02f * m_sizeScale) {
                m_pinSteppable[p] = 1;
                stanceCenter += glm::vec2(m_pins[p].target.x, m_pins[p].target.z);
                ++stanceCount;
            }
        }
        m_stanceRootOffset = glm::vec2(0.0f);
        if (stanceCount >= 2) {
            stanceCenter /= static_cast<float>(stanceCount);
            for (std::size_t p = 0; p < m_pins.size(); ++p) {
                if (m_pinSteppable[p]) {
                    m_pinStanceOffset[p] =
                        glm::vec2(m_pins[p].target.x, m_pins[p].target.z) - stanceCenter;
                }
            }
            // Where the stance naturally sits relative to the hip (see m_stanceRootOffset):
            // a pelvis-drag landing re-creates THIS relationship at the drag target.
            const glm::vec3& rootPos = positions[static_cast<std::size_t>(m_graph.root())];
            m_stanceRootOffset = stanceCenter - glm::vec2(rootPos.x, rootPos.z);
        } else {
            // Fewer than two standing feet: stepping the single support would be a fall.
            m_pinSteppable.assign(m_pins.size(), 0);
        }
    }

    // Active subgraph: the paths joining effector and pins to the root. Everything else —
    // fingers during an arm drag, the face — rides along rigidly.
    std::vector<int> targets{m_effector};
    for (const IkEffector& pin : m_pins) {
        targets.push_back(pin.node);
    }
    m_active = m_graph.markActivePaths(targets);
    m_startPose = positions; // the solver's soft prior: ease back toward the drag-start pose
    // If the pins were ground-snapped (the figure started hovering), heal the PRIOR pose by the
    // same shift: the stiff root prior otherwise anchors the pelvis at its hovering start height
    // and fights the snapped pins to a stalemate part-way off the floor.
    if (snapCount > 0) {
        const float snapMean = snapSum / static_cast<float>(snapCount);
        for (glm::vec3& p : m_startPose) {
            p.y += snapMean;
        }
    }

    // Stiffness model (see m_priorWeights): prior weight grows with METRIC graph distance from
    // the effector (summed bone rest lengths — ~0.8m of body away = trunk-grade), NOT hop count.
    // The dragged joint's neighborhood is nearly free (0.3x), the trunk ~1.3x, far limbs up to
    // 2.5x — natural recruitment order, and the strongest guard against the body contorting to
    // serve a limb-scale drag. Hop counts inflate through small-boned regions: a FINGER effector
    // (what clicking near a hand usually picks) put its own elbow 5 hops and shoulder 7 hops
    // away — trunk-grade stiffness for the dragged ARM itself — and the prior then fought the
    // arm's extension, stalling a finger-raised hand at chin height while a hand-raised one
    // reached overhead.
    m_priorWeights.assign(static_cast<std::size_t>(n), kPosePriorWeight);
    {
        std::vector<float> dist(static_cast<std::size_t>(n), -1.0f);
        std::vector<std::vector<int>> kids(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            const int p = m_parents[static_cast<std::size_t>(i)];
            if (p >= 0 && m_bodyNode[static_cast<std::size_t>(i)]) {
                kids[static_cast<std::size_t>(p)].push_back(i);
            }
        }
        std::vector<int> stack{m_effector};
        dist[static_cast<std::size_t>(m_effector)] = 0.0f;
        while (!stack.empty()) {
            const int cur = stack.back();
            stack.pop_back();
            const float dc = dist[static_cast<std::size_t>(cur)];
            const int p = m_parents[static_cast<std::size_t>(cur)];
            if (p >= 0 && m_bodyNode[static_cast<std::size_t>(p)] &&
                dist[static_cast<std::size_t>(p)] < 0.0f) {
                dist[static_cast<std::size_t>(p)] = dc + m_edgeRestLen[static_cast<std::size_t>(cur)];
                stack.push_back(p);
            }
            for (const int c : kids[static_cast<std::size_t>(cur)]) {
                if (dist[static_cast<std::size_t>(c)] < 0.0f) {
                    dist[static_cast<std::size_t>(c)] = dc + m_edgeRestLen[static_cast<std::size_t>(c)];
                    stack.push_back(c);
                }
            }
        }
        for (int i = 0; i < n; ++i) {
            const float d =
                dist[static_cast<std::size_t>(i)] < 0.0f ? 2.0f : dist[static_cast<std::size_t>(i)];
            m_priorWeights[static_cast<std::size_t>(i)] =
                kPosePriorWeight * glm::clamp(0.3f + d / (0.8f * m_sizeScale), 0.3f, 2.5f);
        }
    }
    // Pin-serving chains stay nearly prior-free: the prior pulls toward absolute START
    // POSITIONS, and once the trunk leans, dragging a planted leg's joints back toward where
    // they USED to be swings the limb away from its pin (a foot ended 18cm off, held there by
    // its own prior). Those chains have hard goals — the pins; the stiffness belongs to the
    // trunk and the unpinned remainder.
    for (const IkEffector& pin : m_pins) {
        for (int cur = pin.node; cur >= 0 && cur != m_pelvis;
             cur = m_parents[static_cast<std::size_t>(cur)]) {
            m_priorWeights[static_cast<std::size_t>(cur)] =
                std::min(m_priorWeights[static_cast<std::size_t>(cur)], kPosePriorWeight * 0.3f);
        }
    }
    // The ROOT is the stiffest of all for limb drags: a person pulling a hand barely moves their
    // pelvis, but the forward pass drags the floating root a little toward the target every tick
    // — the pelvis slid toward every pull and the spine arched around it (a swayback "hip
    // thrust" on gentle drags). Explicit pelvis drags are unaffected (the root is a pinned
    // effector then, and effectors are prior-exempt), as are balance corrections (soft pelvis
    // effector) — this only resists incidental drift.
    if (m_pelvis >= 0) {
        m_priorWeights[static_cast<std::size_t>(m_pelvis)] = kPosePriorWeight * 5.0f;
    }
    // The TRUNK segment of the dragged limb's path (see m_trunkChain in the header): from the
    // node where the limb joins the AXIAL skeleton down to the root. Stiffened per solve under
    // UPWARD drag intent.
    m_trunkChain.clear();
    {
        // The junction is identified by MASS, not by branching: the first ancestor whose
        // OFF-PATH descendants carry real body mass (the head/neck and the other arm hang off
        // the upper chest; the other leg off the pelvis). Branch-counting failed both ways —
        // a FINGER effector's first branching ancestor is the HAND (freezing from there locked
        // the entire arm, and grabbing near a hand almost always picks a finger), while the
        // LAST branching ancestor landed on the lower chest (the token-mass pectoral bones
        // branch there), leaving the upper chest free to pitch the head down again.
        // (subtreeMass was computed at the top of beginDrag.)
        const int trunkStart = limbJunction(m_effector, subtreeMass);
        for (int cur = trunkStart; cur >= 0; cur = m_parents[static_cast<std::size_t>(cur)]) {
            m_trunkChain.push_back(cur);
            if (cur == m_pelvis) {
                break;
            }
        }
        static const bool kRigTrace = std::getenv("POSESTUDIO_IK_TRACE") != nullptr;
        if (kRigTrace) {
            std::fprintf(stderr, "[ikrig] effector=%d trunkStart=%d chainLen=%zu\n", m_effector,
                         trunkStart, m_trunkChain.size());
        }
    }
    return true;
}

int IkRig::limbJunction(int node, const std::vector<float>& subtreeMass) const {
    constexpr float kTrunkJunctionMass = 0.05f; // ~5% of body mass hanging off the path
    const int n = static_cast<int>(m_parents.size());
    int prevOnPath = node;
    for (int cur = m_parents[static_cast<std::size_t>(node)];
         cur >= 0 && cur != m_pelvis; cur = m_parents[static_cast<std::size_t>(cur)]) {
        float offPathMass = 0.0f;
        for (int c = 0; c < n; ++c) {
            if (m_parents[static_cast<std::size_t>(c)] == cur && c != prevOnPath &&
                m_bodyNode[static_cast<std::size_t>(c)]) {
                offPathMass += subtreeMass[static_cast<std::size_t>(c)];
            }
        }
        if (offPathMass > kTrunkJunctionMass) {
            return cur; // first real junction walking up = where the limb attaches
        }
        prevOnPath = cur;
    }
    return m_pelvis;
}

void IkRig::beginSettle(const std::vector<glm::vec3>& positions) {
    const int n = m_graph.nodeCount();
    if (!dragActive() || static_cast<int>(positions.size()) != n) {
        return;
    }
    // A step still in flight at mouse-up completes instantly in TARGET terms: the pin lands at
    // its balanced spot and the settle's capped monotone rounds animate the foot down onto it —
    // leaving the swing target mid-air would settle the foot hanging there.
    if (m_stepPin >= 0) {
        landStep(positions);
    }
    // The settle's prior is the pose at MOUSE-UP — the pose the user just made and expects to
    // keep. (The drag-start pose would be wrong here: a prior toward it stood the figure back up
    // out of its crouch and yanked the placed hand toward where the drag began — the round-5
    // failure that originally led to running the settle prior-free. Prior-free was wrong too:
    // with only the pins guarded, the solver paid for millimeters of pin progress with
    // centimeters of unrelated drift, and a lifted foot visibly sagged after release.)
    m_settlePrior = positions;
    if (m_effector >= 0) {
        m_settleEffectorTarget = positions[static_cast<std::size_t>(m_effector)];
    }
    // Stiffness: everything not serving a pin holds the release pose firmly; pin-serving chains
    // (and the effector's own chain — it must articulate to keep the effector fixed while the
    // root shifts to plant the feet) stay nearly prior-free. Effectors are prior-exempt in the
    // solver, so the held joints themselves are unaffected.
    m_settlePriorWeights.assign(static_cast<std::size_t>(n), kPosePriorWeight * 2.0f);
    for (const IkEffector& pin : m_pins) {
        for (int cur = pin.node; cur >= 0 && cur != m_pelvis;
             cur = m_parents[static_cast<std::size_t>(cur)]) {
            m_settlePriorWeights[static_cast<std::size_t>(cur)] = kPosePriorWeight * 0.3f;
        }
    }
    for (int cur = m_effector; cur >= 0 && cur != m_pelvis;
         cur = m_parents[static_cast<std::size_t>(cur)]) {
        m_settlePriorWeights[static_cast<std::size_t>(cur)] = kPosePriorWeight * 0.3f;
    }
    // The ROOT stays supple too: planting flexibility comes almost entirely from the pelvis
    // giving a couple of centimeters, and the pose CONTRACT the settle protects is the held
    // effector plus the limbs' shapes (their own priors), not the root's absolute position — a
    // stiff root prior here left strained releases stalled with the feet hovering just off
    // their pins.
    if (m_pelvis >= 0) {
        m_settlePriorWeights[static_cast<std::size_t>(m_pelvis)] = kPosePriorWeight * 0.3f;
    }
}

bool IkRig::settleToPins(std::vector<glm::vec3>& positions,
                         const std::vector<glm::quat>& frameSeed) {
    const int n = m_graph.nodeCount();
    if (!dragActive() || m_pins.empty() || static_cast<int>(positions.size()) != n ||
        static_cast<int>(frameSeed.size()) != n ||
        static_cast<int>(m_settlePrior.size()) != n) {
        return false;
    }
    // The pins land; the RELEASED EFFECTOR is held as a pin at its mouse-up position ("the pose
    // must hold when letting go" — the user placed that joint there deliberately); everything
    // else eases toward the release-pose prior. The drag's active set (paths joining effector,
    // pins, and root) is exactly the settle's — same targets — so it is reused rather than
    // re-marked every tick; the rest of the body keeps its local pose and rides along.
    std::vector<IkEffector> effectors = m_pins;
    bool effectorPinned = m_effector < 0;
    for (const IkEffector& pin : m_pins) {
        effectorPinned = effectorPinned || pin.node == m_effector;
    }
    // The released joint is held only for LIMB gestures (a placed hand or foot). After a TRUNK
    // drag (chest, hip, head — effectorIsTrunk) the body must be free to drop onto its feet:
    // with the chest pinned the settle's first round could only make the worst foot pin WORSE
    // (the body cannot descend 4cm around a fixed chest), was reverted, and the figure stayed
    // standing 4cm in the air after every stepping chest drag. The release pose stays the soft
    // prior, so the trunk follows the landing rather than being yanked anywhere else.
    if (!effectorPinned && !effectorIsTrunk()) {
        effectors.push_back({m_effector, m_settleEffectorTarget, true, 1.0f});
    }
    FabrikSolver::Settings settings;
    settings.maxIterations = kIterationsPerTick;
    settings.priorIterationNorm = kPriorIterationNorm; // settle stiffness tuned at 6 iterations
    settings.floorY = m_groundOffsetY;
    settings.floorClearance = &m_floorClearance;
    FabrikSolver::solve(m_graph, m_active, effectors, m_edgeRestDir, m_edgeRestLen,
                        m_edgeConstraint, frameSeed, positions, settings, &m_settlePrior,
                        &m_settlePriorWeights);
    return true;
}

void IkRig::rebuildSupportHull(int excludePin) {
    std::vector<glm::vec2> pts;
    for (std::size_t p = 0; p < m_pins.size(); ++p) {
        if (static_cast<int>(p) == excludePin) {
            continue; // the foot in flight supports nothing
        }
        const glm::vec2 t(m_pins[p].target.x, m_pins[p].target.z);
        for (const glm::vec2& off : m_pinFootprint[p]) {
            pts.push_back(t + off);
        }
    }
    m_supportHull = BalanceController::supportPolygon(std::move(pts));
}

void IkRig::landStep(const std::vector<glm::vec3>& positions) {
    IkEffector& pin = m_pins[static_cast<std::size_t>(m_stepPin)];
    pin.target = m_stepTo;
    pin.leashRadius = socketLeash(pin.node, m_stepTo, positions, pin.leashOffset);
    rebuildSupportHull(-1);
    m_stepPin = -1;
    m_imbalanceTicks = 0;
    ++m_stepsTaken;
}

void IkRig::updateStepping(const std::vector<glm::vec3>& positions, bool allowTrigger,
                           float entryPinErr, const glm::vec2* anchorXZ) {
    if (m_stepPin >= 0) {
        // A step in flight ALWAYS advances (drag intent may change mid-swing; a foot must never
        // hang mid-air waiting for it to change back): ease the pin target along the glide with
        // a sine lift arc, then land — re-pin at the balanced spot, restore a recomputed leash,
        // and rebuild the full support polygon.
        ++m_stepTick;
        const float t =
            glm::clamp(static_cast<float>(m_stepTick) / static_cast<float>(m_stepTotal), 0.0f,
                       1.0f);
        const float ease = t * t * (3.0f - 2.0f * t);
        IkEffector& pin = m_pins[static_cast<std::size_t>(m_stepPin)];
        pin.target = glm::mix(m_stepFrom, m_stepTo, ease);
        pin.target.y += m_stepHeight * std::sin(3.14159265f * t);
        if (m_stepTick >= m_stepTotal) {
            landStep(positions);
        }
        return;
    }
    if (!allowTrigger) {
        m_imbalanceTicks = 0;
        return;
    }
    // The stance ANCHOR the balanced-stance estimate centers on: a pelvis drag's TARGET when
    // explicit (where the user is TAKING the body — steps land where it is going, and the end
    // stance is the drag-start stance shape centered on where the hip stops), else the CoM
    // (upper-body drags: the balance point is the intent signal). Anchoring steps on the
    // current CoM during a pelvis drag placed every landing behind the moving hip and the walk
    // ended in a collapsed, machinery-looking stance.
    // The explicit anchor is offset by the stance's drag-start relationship to the hip (see
    // m_stanceRootOffset); the CoM anchor is the balance point itself.
    glm::vec2 anchor(0.0f);
    if (anchorXZ != nullptr) {
        anchor = *anchorXZ + m_stanceRootOffset;
    } else {
        const glm::vec3 com = BalanceController::centerOfMass(positions, m_parents, m_masses);
        anchor = glm::vec2(com.x, com.z);
    }
    // Worst steppable stance error vs the anchor — both the explicit-anchor trigger and the
    // step-choice ranking read it.
    const float scale = m_sizeScale;
    int best = -1;
    float bestErr = kStepMinDistance * scale;
    for (std::size_t p = 0; p < m_pins.size(); ++p) {
        if (!m_pinSteppable[p]) {
            continue;
        }
        const glm::vec2 desired = anchor + m_pinStanceOffset[p];
        const float e =
            glm::length(glm::vec2(m_pins[p].target.x, m_pins[p].target.z) - desired);
        if (e > bestErr) {
            bestErr = e;
            best = static_cast<int>(p);
        }
    }
    // Trigger, sustained for kStepConfirmTicks (a transient lean recovers by itself): the
    // smoothed balance EFFORT stays high — the drag keeps imposing a lean the pelvis correction
    // must continuously fight (the post-correction residual is useless here: the engagement
    // blend succeeds each tick, so the residual reads balanced right up until the figure is
    // grotesquely stretched) — or a standing foot's ENTRY pin error says the pose can no longer
    // physically hold its plant (the FK pose is being dragged off the pin — the moment a person
    // must step) — or, with an EXPLICIT anchor only, a foot is simply far from its
    // target-centered stance spot: this is what makes a pelvis-walk step PROACTIVELY (before
    // the foot gets visibly dragged) and take its final GATHERING step once the hip arrives
    // (the trailing foot's pin holds fine there, so neither other signal would ever fire).
    const bool strained = entryPinErr > kStepPinErrThreshold * scale;
    const bool misplaced =
        anchorXZ != nullptr && best >= 0 && bestErr > kStepStanceThreshold * scale;
    static const bool kStepTrace = std::getenv("IK_STEP_TRACE") != nullptr;
    if (kStepTrace) {
        std::fprintf(stderr,
                     "[step] entryPinErr=%.4f corr=%.4f stanceErr=%.4f s=%d m=%d ticks=%d\n",
                     entryPinErr, glm::length(m_balanceCorrection), best >= 0 ? bestErr : 0.0f,
                     strained ? 1 : 0, misplaced ? 1 : 0, m_imbalanceTicks);
    }
    if (glm::length(m_balanceCorrection) <= kStepNeedThreshold * scale && !strained &&
        !misplaced) {
        m_imbalanceTicks = std::max(0, m_imbalanceTicks - 1);
        return;
    }
    if (++m_imbalanceTicks < kStepConfirmTicks) {
        return;
    }
    if (best < 0) {
        m_imbalanceTicks = 0;
        return;
    }
    IkEffector& pin = m_pins[static_cast<std::size_t>(best)];
    const float bindY = m_bindPos[static_cast<std::size_t>(pin.node)].y + m_groundOffsetY;
    glm::vec2 landXZ = anchor + m_pinStanceOffset[static_cast<std::size_t>(best)];
    // Reachability: the landing must stay within the leg's reach of where the ROOT is going —
    // the anchor (measuring from the CURRENT root systematically pulled landings in under the
    // pelvis: by landing time the root has moved on, and the tight ring NARROWED every stance).
    {
        const float pathLen = pathLenToRoot(pin.node);
        const float dy = positions[static_cast<std::size_t>(m_pelvis)].y - bindY;
        // 0.98 of the leg path: a STANDING leg already uses ~0.95 of its root-to-foot path
        // (the pelvis-to-socket offset is lateral, not collinear), so a conservative factor
        // here read every standing stance as "unreachable" and no step could ever initiate.
        const float maxHoriz2 = 0.9604f * pathLen * pathLen - dy * dy;
        if (maxHoriz2 <= 1e-6f) {
            m_imbalanceTicks = 0;
            return; // the root sits too high/low for this leg to plant anywhere useful
        }
        const glm::vec2 toLand = landXZ - anchor;
        const float horiz = glm::length(toLand);
        const float maxHoriz = std::sqrt(maxHoriz2);
        if (horiz > maxHoriz) {
            landXZ = anchor + toLand * (maxHoriz / horiz);
        }
    }
    // Clearance, SIDE-PRESERVING: never land on another pin, and never CROSS it — a landing on
    // the far side of the standing foot (a long lateral move whose trailing foot's desired spot
    // lies past the leading foot) is clamped to the NEAR side at clearance distance instead.
    // The legs never scissor, and on a long move the feet alternate naturally: the clamped foot
    // stops at the standing one, which then carries the larger stance error and leapfrogs.
    for (std::size_t p = 0; p < m_pins.size(); ++p) {
        if (static_cast<int>(p) == best) {
            continue;
        }
        const glm::vec2 other(m_pins[p].target.x, m_pins[p].target.z);
        const glm::vec2 curSide =
            glm::vec2(pin.target.x, pin.target.z) - other; // which side this foot is on now
        const glm::vec2 away = landXZ - other;
        const float sideLen = glm::length(curSide);
        if (sideLen > 1e-5f && glm::dot(away, curSide) < 0.0f) {
            landXZ = other + curSide * (kStepClearance * scale / sideLen); // would cross
            continue;
        }
        const float d = glm::length(away);
        if (d < kStepClearance * scale) {
            if (d < 1e-5f) {
                m_imbalanceTicks = 0;
                return;
            }
            landXZ = other + away * (kStepClearance * scale / d);
        }
    }
    m_stepFrom = pin.target;
    m_stepTo = glm::vec3(landXZ.x, bindY, landXZ.y);
    const float dist = glm::length(m_stepTo - m_stepFrom);
    if (dist < kStepMinDistance * scale) {
        m_imbalanceTicks = 0;
        return;
    }
    m_stepPin = best;
    m_stepTick = 0;
    m_stepTotal = glm::clamp(static_cast<int>(dist / (kStepSpeed * scale)), kStepMinTicks,
                             kStepMaxTicks);
    m_stepHeight = glm::clamp(kStepHeightFactor * dist, kStepHeightMin * scale,
                              kStepHeightMax * scale);
    // A moving pin's leash would yank the root after the swinging target; the standing limbs'
    // leashes keep binding the root, and the landing recomputes this one. The sentinel must be
    // effectively INFINITE, not merely unset: the solver treats a non-positive radius as
    // "compute the path-length fallback", which quietly re-leashed the root to the moving
    // swing target.
    pin.leashRadius = 1e9f;
    rebuildSupportHull(best);
    static const bool kRigStepTrace = std::getenv("POSESTUDIO_IK_TRACE") != nullptr;
    if (kRigStepTrace) {
        std::fprintf(stderr, "[ikrig] STEP pin=%d from(%.3f %.3f) to(%.3f %.3f) ticks=%d\n",
                     pin.node, m_stepFrom.x, m_stepFrom.z, m_stepTo.x, m_stepTo.z, m_stepTotal);
    }
}

bool IkRig::solveDrag(const glm::vec3& target, std::vector<glm::vec3>& positions,
                      const std::vector<glm::quat>& frameSeed) {
    const int n = m_graph.nodeCount();
    if (!dragActive() || static_cast<int>(positions.size()) != n ||
        static_cast<int>(frameSeed.size()) != n) {
        return false;
    }

    // Deadband: nothing meaningfully to do (cursor holding still, pins already planted).
    bool satisfied =
        glm::length(positions[static_cast<std::size_t>(m_effector)] - target) <= kGoalDeadband;
    for (const IkEffector& pin : m_pins) {
        satisfied = satisfied &&
                    glm::length(positions[static_cast<std::size_t>(pin.node)] - pin.target) <=
                        kGoalDeadband;
    }
    if (satisfied) {
        // A satisfied EXPLICIT-ANCHOR (pelvis) drag may still owe a GATHERING step: the hip is
        // on its target and every pin holds, but a trailing foot can be far from its
        // target-centered stance spot — and only updateStepping can see that. Its confirm
        // counter keeps counting here; the tick a step initiates, the swing target moves and
        // the deadband stops being satisfied, so the normal solve path takes over. And for ANY
        // drag, a step already IN FLIGHT must keep advancing: the deadband can become satisfied
        // mid-swing (the foot converged onto the frozen mid-arc target while the cursor held
        // still), and returning without the advance left the foot hanging mid-air forever.
        if (!m_pins.empty() && !m_suspended) {
            if (m_stepPin >= 0) {
                updateStepping(positions, false, 0.0f, nullptr); // advance only
                return true;
            }
            if (m_effector == m_graph.root()) {
                const bool crouchIntent =
                    target.y < m_startPose[static_cast<std::size_t>(m_pelvis)].y - 0.08f;
                if (!crouchIntent) {
                    const glm::vec2 anchor(target.x, target.z);
                    updateStepping(positions, true, 0.0f, &anchor);
                    return m_stepPin >= 0;
                }
            }
        }
        return false;
    }

    // Worst steppable pin error at SOLVE ENTRY (the FK pose, before any solving) — the strain
    // half of the step trigger: an extreme drag reaches poses whose joint limits can no longer
    // hold a foot on its pin, and the foot gets dragged along the floor.
    float entrySteppablePinErr = 0.0f;
    for (std::size_t p = 0; p < m_pins.size(); ++p) {
        if (p < m_pinSteppable.size() && m_pinSteppable[p]) {
            entrySteppablePinErr = std::max(
                entrySteppablePinErr,
                glm::length(positions[static_cast<std::size_t>(m_pins[p].node)] -
                            m_pins[p].target));
        }
    }

    // Dragging the pelvis itself: the root is the one joint FBIK can place directly (root
    // translation is free), so it becomes a PINNED effector on the drag target and the solve
    // re-plants the planted feet around it — a crouch. Balance is deliberately skipped here: the
    // user is driving the pelvis explicitly.
    FabrikSolver::Settings settings;
    settings.maxIterations = kIterationsPerTick;
    settings.priorIterationNorm = kPriorIterationNorm;
    settings.minStepDisplacement = kOutputDeadband;
    settings.floorY = m_groundOffsetY;
    settings.floorClearance = &m_floorClearance;

    if (m_effector == m_graph.root()) {
        // An explicit pelvis drag is a deliberate whole-body gesture (a crouch): both legs must
        // fold in coordination every tick, which needs more solver iterations than a limb drag.
        settings.maxIterations = kIterationsPerTick * 3;
        std::vector<IkEffector> effectors = m_pins;
        // USER pins bound an explicit pelvis drag: the root target is projected into each user
        // pin's reach ball (a pinned hand holds — the hip drop stops where the arm runs out),
        // unlike ground contacts, which a pelvis drag may lift by design (the leashes are off
        // for a pinned root in the solver).
        glm::vec3 rootTarget = target;
        for (std::size_t p = 0; p < m_pins.size(); ++p) {
            const IkEffector& pin = m_pins[p];
            if (!m_pinUser[p] || pin.leashRadius <= 0.0f || pin.leashRadius > 1e8f) {
                continue;
            }
            const glm::vec3 center = pin.target - pin.leashOffset;
            const glm::vec3 d = rootTarget - center;
            const float len = glm::length(d);
            if (len > pin.leashRadius) {
                rootTarget = center + d * (pin.leashRadius / len);
            }
        }
        effectors.push_back({m_effector, rootTarget, true, 1.0f});
        FabrikSolver::solve(m_graph, m_active, effectors, m_edgeRestDir, m_edgeRestLen,
                            m_edgeConstraint, frameSeed, positions, settings, &m_startPose,
                            &m_priorWeights);
        // A LATERAL pelvis drag walks the figure: moving the hip sideways/forward drags the
        // planted feet off their pins (the strain trigger — there is no balance signal here,
        // the pelvis is user-driven), and the feet re-plant under the moved body exactly like
        // an upper-body lean. A DOWNWARD pelvis drag is a crouch and keeps its feet planted —
        // measured against the drag-START pelvis height, not per tick, so a diagonal
        // down-and-across gesture stays a crouch throughout.
        if (!m_pins.empty()) {
            const bool crouchIntent =
                target.y < m_startPose[static_cast<std::size_t>(m_pelvis)].y - 0.08f;
            const glm::vec2 anchor(target.x, target.z);
            updateStepping(positions, /*allowTrigger=*/!crouchIntent,
                           crouchIntent ? 0.0f : entrySteppablePinErr, &anchor);
        }
        return true;
    }

    // Drag intent shapes the body's compliance:
    //  - DOWNWARD intent (target below the grabbed joint): the root prior YIELDS vertically —
    //    pushing the chest (or pulling a hand) down folds the body into a crouch over the
    //    pinned, weight-bearing feet instead of fighting the standing-height anchor.
    //  - A sustained, mostly-VERTICAL pull beyond the leashed body's reach is a deliberate
    //    LIFT: enter SUSPENSION — pins release, the root rises with the drag, and the whole
    //    body (fully active, gravity-biased, prior-free) settles hanging below the grab point,
    //    limbs dangling within their joint limits.
    const glm::vec3& effPos = positions[static_cast<std::size_t>(m_effector)];
    // Decisive gestures only (-0.15): a hand pulled a few cm downward must not sag the pelvis.
    const bool downIntent = target.y < effPos.y - 0.15f;
    // The crouch DRIVE: how far below its current height the pelvis is pulled this solve — the
    // remaining downward error, capped per solve so the descent unfolds smoothly. Applied
    // through the pelvis effector below; self-limiting (error -> 0 as the push lands).
    const float rootDown =
        downIntent ? glm::clamp(effPos.y - target.y, 0.0f, 0.05f * m_sizeScale) : 0.0f;
    if (downIntent) {
        settings.rootDownYield = kRootDownYield;
    }
    // UPWARD intent: the trunk is EXCLUDED from the solve — the arm and shoulder girdle do the
    // raising against a fixed chest, and the spine simply holds its posture. Stiffening the
    // trunk's prior was not enough: the raising limb's residual error (its joints at their
    // limits) re-recruited the spine every tick and the solve found a bow/lean the user never
    // asked for — "pulling straight up bent her over". With the trunk fixed, an up-pull raises
    // the arm along its natural arc, predictably; pulls the arc cannot serve leave the hand
    // short until SUSPENSION takes over. Forward/lateral/downward intents keep full trunk
    // recruitment (leaning into a reach, crouching under a push).
    // A TRUNK effector (the head, the chest — effectorIsTrunk) is exempt: its "limb" is the
    // neck or nothing, and excluding the trunk left a 12cm head pull moving the head 7mm.
    // Pulling the head up should straighten the spine — that is the trunk's own gesture, not
    // an arm's residual recruiting it.
    const bool upIntent = target.y > effPos.y + 0.10f && !effectorIsTrunk();
    const std::vector<float>* solveWeights = &m_priorWeights;
    const std::vector<char>* solveActive = &m_active;
    if (upIntent && !m_trunkChain.empty()) {
        m_scratchWeights = m_priorWeights;
        for (const int t : m_trunkChain) {
            m_scratchWeights[static_cast<std::size_t>(t)] = std::max(
                m_scratchWeights[static_cast<std::size_t>(t)], kPosePriorWeight * kUpTrunkStiffen);
        }
        solveWeights = &m_scratchWeights;
        m_scratchActive = m_active;
        for (const int t : m_trunkChain) {
            m_scratchActive[static_cast<std::size_t>(t)] = 0; // fixed base: the trunk stays put
        }
        solveActive = &m_scratchActive;
    }
    if (!m_suspended && !m_pins.empty()) {
        // Chain length from the grab point to the root: the body's maximum reach from the
        // (leash-bound) pelvis, and later the hang distance the root dangles at.
        const float chainLen = pathLenToRoot(m_effector);
        const glm::vec3 strain = target - effPos;
        const float strainLen = glm::length(strain);
        bool leashBound = false;
        const glm::vec3& rootPos = positions[static_cast<std::size_t>(m_pelvis)];
        for (const IkEffector& pin : m_pins) {
            if (pin.leashRadius > 0.0f &&
                glm::length(rootPos - pin.target) > 0.96f * pin.leashRadius) {
                leashBound = true;
                break;
            }
        }
        // The lift gate is GEOMETRIC unreachability — the target farther from the leash-bound
        // root than the fully-extended chain — NOT the instantaneous drag error: the hand lags
        // the cursor by design (damped catch-up), so error-based strain read a brisk upward
        // pull as "beyond reach" while the arm was still down and hoisted the figure instead of
        // simply RAISING THE ARM overhead (which is what a reachable up-pull must do first).
        const bool beyondReach =
            glm::length(target - rootPos) > chainLen + kSuspendStrain * m_sizeScale;
        if (leashBound && beyondReach && strainLen > kSuspendStrain * m_sizeScale &&
            strain.y > kSuspendUpFraction * strainLen) {
            if (++m_suspendTicks >= kSuspendConfirmTicks) {
                m_suspended = true;
                // Ground contacts release; USER pins stay (they are explicit intent — a body
                // hanging from a pinned hand is exactly what a lift against one produces).
                {
                    std::vector<IkEffector> kept;
                    std::vector<char> keptUser;
                    for (std::size_t p = 0; p < m_pins.size(); ++p) {
                        if (m_pinUser[p]) {
                            kept.push_back(m_pins[p]);
                            keptUser.push_back(1);
                        }
                    }
                    m_pins = std::move(kept);
                    m_pinUser = std::move(keptUser);
                    m_pinFootprint.assign(m_pins.size(), {});
                    m_pinSteppable.assign(m_pins.size(), 0);
                    m_pinStanceOffset.assign(m_pins.size(), glm::vec2(0.0f));
                }
                m_supportHull.clear();
                m_stepPin = -1; // an in-flight step's foot is released with the rest
                m_suspendHang = chainLen;
                // Only real body-mass segments dangle (trunk, limbs — the balance mass model
                // already classifies them); token-mass bones (face, fingers, helpers) keep
                // their local pose and ride. Without this the JAW — a hinged joint — was
                // gravity-pulled OPEN: the figure hung with its mouth agape. The effector's own
                // chain is always active regardless of mass.
                m_active.assign(m_bodyNode.size(), 0);
                for (std::size_t i = 0; i < m_bodyNode.size(); ++i) {
                    if (m_bodyNode[i] && m_masses[i] > kRealMassThreshold) {
                        m_active[i] = 1;
                    }
                }
                for (int cur = m_effector; cur >= 0;
                     cur = m_parents[static_cast<std::size_t>(cur)]) {
                    if (!m_bodyNode[static_cast<std::size_t>(cur)]) {
                        break;
                    }
                    m_active[static_cast<std::size_t>(cur)] = 1;
                    if (cur == m_pelvis) {
                        break;
                    }
                }
            }
        } else {
            m_suspendTicks = 0;
        }
    }

    if (m_suspended) {
        settings.gravityBias = kSuspendGravity * m_sizeScale;
        settings.rootDownYield = 1.0f;
        // The root HANGS below the grab point (a soft effector at chain-length under the
        // target): without it, gravity dragged the whole body to the floor while the hand
        // reached up alone — the root must rise with the drag for the body to dangle.
        std::vector<IkEffector> effectors{
            {m_effector, target, false, 1.0f},
            {m_pelvis, glm::vec3(target.x, target.y - m_suspendHang, target.z), false, 0.5f}};
        effectors.insert(effectors.end(), m_pins.begin(), m_pins.end()); // surviving user pins
        // No pose prior: the hanging body is shaped by gravity + the joint limits alone.
        FabrikSolver::solve(m_graph, m_active, effectors, m_edgeRestDir, m_edgeRestLen,
                            m_edgeConstraint, frameSeed, positions, settings, nullptr, nullptr);
        return true;
    }

    std::vector<IkEffector> effectors;
    effectors.reserve(m_pins.size() + 2);
    effectors.push_back({m_effector, target, false, 1.0f});
    effectors.insert(effectors.end(), m_pins.begin(), m_pins.end());

    FabrikSolver::solve(m_graph, *solveActive, effectors, m_edgeRestDir, m_edgeRestLen,
                        m_edgeConstraint, frameSeed, positions, settings, &m_startPose,
                        solveWeights);

    // Auto-balance: when the solved CoM's ground projection leaves the (margin-inset) support
    // polygon, a soft pelvis effector pulls it back. The engagement machinery exists to make
    // that influence CONTINUOUS — the original design skipped the balanced re-solve whenever
    // the correction was negligible, and under a sustained pull the loop crossed that on/off
    // boundary every few ticks, toggling between two visibly different solutions: with every
    // idle limb riding on the trunk, the user saw it as TREMBLING while pulling. Three rules,
    // each verified against the tremble metrics (every cheaper variant — skip-when-quiet,
    // iteration-scaled, trust-region-scaled — put some boundary back and measurably churned):
    //  1. The re-solve runs EVERY tick, at full depth, on a COPY of the solution; the result is
    //     blended in by ENGAGEMENT (positions = mix(plain, balanced, engage)) — exactly the
    //     plain solve at engage 0, exactly the old engaged re-solve at engage 1, a plain lerp
    //     between (the transient mid-ramp blend bends bone lengths a hair; extraction re-imposes
    //     them through FK immediately).
    //  2. Engagement follows a LOW-PASSED correction (m_balanceCorrection), so tick-scale
    //     flicker of the raw need (the CoM dancing on the inset boundary) never reaches the
    //     blend.
    //  3. The effector's correction target is scaled by engagement, so the correction
    //     DECELERATES as the CoM approaches the polygon instead of overshooting deep inside —
    //     where the need reads zero, the correction would release, and the loop would swing
    //     back out (the relaxation oscillation the low-pass alone could not remove).
    // Not under UPWARD intent: the trunk (pelvis included) is excluded from the solve there, so
    // the pelvis effector is inert and the re-solve would only DOUBLE the solve depth in a
    // regime whose damped-motion behavior is tuned without it (measurably worsening the
    // up-sweep's transient reversal at the suspension boundary). The up-intent flip is already
    // a binary structure change in the design (the active set restructures), so this skip adds
    // no new boundary class.
    if (!m_supportHull.empty() && m_pelvis >= 0 && m_pelvis != m_effector && !upIntent) {
        glm::vec2 need(0.0f);
        BalanceController::balanceCorrection(positions, m_parents, m_masses, m_supportHull,
                                             kBalanceMargin * m_sizeScale,
                                             need); // false leaves need at 0
        m_balanceCorrection = glm::mix(m_balanceCorrection, need, kBalanceSmoothing);
        const float engage =
            (rootDown > 0.005f || m_healing)
                ? 1.0f
                : glm::clamp(glm::length(m_balanceCorrection) / (kBalanceRamp * m_sizeScale),
                             0.0f, 1.0f);
        static const bool kBalTrace = std::getenv("IK_BAL_TRACE") != nullptr;
        if (kBalTrace) {
            std::fprintf(stderr, "[bal] need=%.4f corr=%.4f engage=%.3f down=%.3f heal=%d\n",
                         glm::length(need), glm::length(m_balanceCorrection), engage, rootDown,
                         m_healing ? 1 : 0);
        }
        if (engage > 0.0f) {
            // XZ from the engagement-scaled smoothed correction; Y from the PRIOR height —
            // EXCEPT under downward intent, where the effector actively DRIVES the pelvis down
            // by the remaining descent (capped per solve): pushing the chest down folds the
            // body into a crouch over the pinned feet (the passive yield alone was not enough —
            // the backward pass kept re-imposing the standing configuration from the un-driven
            // root). The prior-height default is what lets a hovering figure keep descending
            // while balance fires (the engaged effector makes the pelvis prior-exempt, so a
            // current-height Y silently disabled the root prior).
            const float pelvisY =
                downIntent ? positions[static_cast<std::size_t>(m_pelvis)].y - rootDown
                           : m_startPose[static_cast<std::size_t>(m_pelvis)].y;
            const glm::vec2 corr = m_balanceCorrection * engage;
            const glm::vec3 pelvisTarget =
                glm::vec3(positions[static_cast<std::size_t>(m_pelvis)].x + corr.x, pelvisY,
                          positions[static_cast<std::size_t>(m_pelvis)].z + corr.y);
            std::vector<IkEffector> balanced = effectors;
            balanced.push_back({m_pelvis, pelvisTarget, false, kPelvisWeight});
            m_balancedScratch = positions;
            FabrikSolver::solve(m_graph, *solveActive, balanced, m_edgeRestDir, m_edgeRestLen,
                                m_edgeConstraint, frameSeed, m_balancedScratch, settings,
                                &m_startPose, solveWeights);
            if (engage >= 1.0f) {
                positions = m_balancedScratch;
            } else {
                for (std::size_t i = 0; i < positions.size(); ++i) {
                    positions[i] = glm::mix(positions[i], m_balancedScratch[i], engage);
                }
            }
        }
    }
    // Balance-driven foot re-planting: advance an in-flight step (always — a swing must land
    // whatever the drag does meanwhile), or evaluate the step trigger. New steps only while
    // grounded and outside the special regimes: upward intent barely moves the CoM, a healing
    // drag is already descending onto its pins, and a downward crouch briefly drags feet along
    // the floor by design (its transient pin error is not a step signal).
    if (!m_suspended && !m_pins.empty()) {
        updateStepping(positions, /*allowTrigger=*/!upIntent && !m_healing,
                       downIntent ? 0.0f : entrySteppablePinErr, nullptr);
    }
    return true;
}

} // namespace pose
