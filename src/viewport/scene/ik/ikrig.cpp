/**
 * @file ikrig.cpp
 * @brief IkRig construction: the per-figure FBIK state built once from the bind skeleton, plus
 *        the metric helpers every drag phase shares (path lengths, socket leashes, limb
 *        junctions).
 *
 * The rig's implementation spans four translation units so the phases can evolve independently:
 * this file (build + shared helpers), ikrigdrag.cpp (beginDrag — contacts, pins, user pins, the
 * stiffness model, the trunk chain), ikrigsolve.cpp (solveDrag with its intent policies and the
 * balance blend, and the release settle) and ikrigstepping.cpp (balance-driven foot
 * re-planting); the tuning constants they share live in ikrig_constants.h. build() hosts the
 * load-bearing rooting decisions: the FBIK root is the first multi-child DESCENDANT of the
 * anatomical root (real figures root at an origin-level figure node whose chain is excluded
 * from the rig wholesale — no graph edge, no contact eligibility, no balance mass), per-edge
 * constraints are derived on the OWNING (parent) side, near-hinge bend joints inherit their
 * parent twist bone's authored range as fold-plane freedom, and every world-space threshold
 * scales with the figure's pelvis height (sizeScale). Qt-free (std + GLM).
 */

#include "ikrig.h"

#include "balancecontroller.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace pose {

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
    // Anatomical children per bone, in ASCENDING index order: limbJunction sums off-path subtree
    // masses over these, and ascending order is the summation order the former whole-skeleton
    // scan used — the sums stay bit-identical.
    m_children.assign(n, {});
    for (std::size_t i = 0; i < n; ++i) {
        if (bones[i].parent >= 0 && static_cast<std::size_t>(bones[i].parent) < n) {
            m_children[static_cast<std::size_t>(bones[i].parent)].push_back(static_cast<int>(i));
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
    // the figure-node chain contribute none. Realized by the Armature's twist witness.
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

int IkRig::limbJunction(int node) const {
    constexpr float kTrunkJunctionMass = 0.05f; // ~5% of body mass hanging off the path
    int prevOnPath = node;
    for (int cur = m_parents[static_cast<std::size_t>(node)];
         cur >= 0 && cur != m_pelvis; cur = m_parents[static_cast<std::size_t>(cur)]) {
        float offPathMass = 0.0f;
        for (const int c : m_children[static_cast<std::size_t>(cur)]) {
            if (c != prevOnPath && m_bodyNode[static_cast<std::size_t>(c)]) {
                offPathMass += m_subtreeMass[static_cast<std::size_t>(c)];
            }
        }
        if (offPathMass > kTrunkJunctionMass) {
            return cur; // first real junction walking up = where the limb attaches
        }
        prevOnPath = cur;
    }
    return m_pelvis;
}

} // namespace pose
