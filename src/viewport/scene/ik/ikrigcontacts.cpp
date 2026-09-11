/**
 * @file ikrigcontacts.cpp
 * @brief LIVE contact re-detection during a drag (IkRig::updateContacts) and the pin-set
 *        maintenance around it: adding a contact pin mid-drag, removing one, and rebuilding the
 *        active subgraph and the stiffness model (rebuildPriorWeights — beginDrag's too) for
 *        the current pin set.
 *
 * The drag-start contacts (ikrigdrag.cpp) are what the figure stands on when the drag begins;
 * a LIVE contact is a joint that reaches the floor while the drag lowers the body — the hands
 * in a deep crouch, a knee coming down — and would otherwise sink straight through it: an
 * inactive subtree rides the trunk rigidly, and the solver's floor rule (placeChild) covers
 * active joints only, so a 55cm hip crouch from a forward bend buried both hands 8cm deep. A
 * live contact is a UNILATERAL support — it holds the joint where it touched and lets the body
 * lean on it, but standing back up lifts it off again — so it carries no reach leash, never
 * steps, and releases once the body pulls it off its spot. See ikrig.cpp for the TU layout;
 * the constants come from ikrig_constants.h. Qt-free (std + GLM).
 */

#include "ikrig.h"

#include "ikrig_constants.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace pose {

namespace {

// A joint has REACHED the floor when its contact height (its lowest riding part measured
// against that part's own clearance) is within this of it: the solver holds an ACTIVE free
// joint exactly at its clearance, so a joint resting on the floor rule reads 0 ± the governor's
// residual, and an inactive one crossing the floor between two ticks reads negative.
constexpr float kLiveContactBand = 0.005f;
// A live contact is UNILATERAL, like a hand on a floor: it resists being pushed through
// (the pin height never yields), it resists being dragged along up to a friction-like strain,
// beyond which it SLIDES, and it lets go when the body LIFTS it. All three read the SOLVER's
// output for the pin, not the applied pose: a freshly planted hand's arm must fold from
// straight, and the extraction delivers that fold at its capped rate while the solver already
// has it — the applied hand trails its pin by several centimetres for a few ticks (an
// all-fours descent measured 5.5cm), the governor's transient, not a pull.
//  - LIFT-OFF: the limb TAUT (the solved junction farther from the pin than kLiveContactTaut
//    of the chain's rest span — the pelvis rising out of a crouch with the hands planted pulls
//    the arms straight) AND the solved joint above its pin height by kLiveContactLift, for
//    kLiveContactReleaseTicks consecutive ticks, releases the pin; the joint then re-arms only
//    after clearing the floor by kLiveContactRearm. Both halves are needed: an arm that cannot
//    FOLD (a descent the joint limits refuse) also leaves the solver's hand above the pin —
//    the clamped straight arm points beside it — but with the shoulder CLOSER than the span,
//    and that hand must slide, not let go.
//  - SLIDE: a horizontal solved error beyond kLiveContactSlip moves the pin along the floor
//    by the excess, so the pin resists up to that strain and then creeps with the body — a
//    descent the arms cannot absorb (joint limits) slides the hands forward on the floor
//    instead of dropping them (a released hand sank through the floor as an inactive subtree
//    again, and a joint AT the floor could never re-plant).
constexpr float kLiveContactLift = 0.02f;
constexpr float kLiveContactTaut = 0.95f;
constexpr float kLiveContactSlip = 0.025f;
constexpr int   kLiveContactReleaseTicks = 3;
// A released joint re-arms only once it has cleared the floor by this: released AT the floor
// with the arm taut, it would otherwise re-plant on the very next tick.
constexpr float kLiveContactRearm = 0.03f;
// Riding parts within this of the floor join the new pin's footprint (the fingertips, not the
// wrist, are what a hand stands on).
constexpr float kLiveFootprintBand = 0.03f;

} // namespace

float IkRig::pathLenToEffector(int node) const {
    float acc = 0.0f;
    for (int cur = node; cur >= 0; cur = m_parents[static_cast<std::size_t>(cur)]) {
        for (std::size_t a = 0; a < m_effAncestor.size(); ++a) {
            if (m_effAncestor[a] == cur) {
                return acc + m_effAncestorLen[a]; // met the effector's chain: cur is the LCA
            }
        }
        acc += m_edgeRestLen[static_cast<std::size_t>(cur)];
    }
    return 1e30f; // disconnected (the excluded figure-node chain)
}

bool IkRig::descendsFromPelvis(int node) const {
    if (node < 0 || node >= static_cast<int>(m_parents.size())) {
        return false;
    }
    for (int cur = m_parents[static_cast<std::size_t>(node)]; cur >= 0;
         cur = m_parents[static_cast<std::size_t>(cur)]) {
        if (cur == m_pelvis) {
            return true;
        }
    }
    return false;
}

float IkRig::contactHeight(int node, const std::vector<glm::vec3>& positions) const {
    const std::size_t i = static_cast<std::size_t>(node);
    float h = positions[i].y - m_floorClearance[i];
    for (const int d : m_ridingDescendants[i]) {
        const std::size_t j = static_cast<std::size_t>(d);
        h = std::min(h, positions[j].y - m_floorClearance[j]);
    }
    return h - m_groundOffsetY;
}

void IkRig::addLivePin(int node, const glm::vec3& target,
                       const std::vector<glm::vec3>& positions) {
    // No leash: the sentinel must be effectively INFINITE, not merely unset (a non-positive
    // radius makes the solver compute its path-length fallback, which would leash the root to
    // a hand on the floor — and standing up must be able to lift that hand off).
    IkEffector pin{node, target, true, 1.0f, 1e9f};
    m_pins.push_back(pin);
    m_pinUser.push_back(0);
    m_pinLive.push_back(1);
    m_liveErrTicks.push_back(0);
    m_pinSteppable.push_back(0);
    m_pinStanceOffset.push_back(glm::vec2(0.0f));
    // The limb's rest span (see m_liveSpan): its chain's rest offsets summed as vectors.
    {
        const int junction = limbJunction(node);
        glm::vec3 span(0.0f);
        for (int cur = node; cur >= 0 && cur != junction;
             cur = m_parents[static_cast<std::size_t>(cur)]) {
            span += m_edgeRestDir[static_cast<std::size_t>(cur)] *
                    m_edgeRestLen[static_cast<std::size_t>(cur)];
        }
        m_liveSpan.push_back(glm::length(span));
    }
    std::vector<glm::vec2> footprint{glm::vec2(0.0f)};
    for (const int d : m_ridingDescendants[static_cast<std::size_t>(node)]) {
        const std::size_t j = static_cast<std::size_t>(d);
        if (positions[j].y - m_floorClearance[j] - m_groundOffsetY <
            kLiveFootprintBand * m_sizeScale) {
            footprint.emplace_back(positions[j].x - target.x, positions[j].z - target.z);
        }
    }
    m_pinFootprint.push_back(std::move(footprint));
}

void IkRig::removePin(std::size_t index) {
    const auto erase = [index](auto& v) {
        if (index < v.size()) {
            v.erase(v.begin() + static_cast<std::ptrdiff_t>(index));
        }
    };
    erase(m_pins);
    erase(m_pinUser);
    erase(m_pinLive);
    erase(m_liveErrTicks);
    erase(m_liveSpan);
    erase(m_pinSteppable);
    erase(m_pinStanceOffset);
    erase(m_pinFootprint);
    if (m_stepPin == static_cast<int>(index)) {
        m_stepPin = -1;
    } else if (m_stepPin > static_cast<int>(index)) {
        --m_stepPin;
    }
}

void IkRig::rebuildActiveSet() {
    // Active subgraph: the paths joining effector and pins to the root. Everything else —
    // fingers during an arm drag, the face — rides along rigidly.
    std::vector<int> targets{m_effector};
    for (const IkEffector& pin : m_pins) {
        targets.push_back(pin.node);
    }
    m_active = m_graph.markActivePaths(targets);
}

void IkRig::rebuildPriorWeights() {
    const int n = m_graph.nodeCount();
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
}

bool IkRig::updateContacts(const std::vector<glm::vec3>& applied,
                           const std::vector<glm::vec3>& solved) {
    const int n = m_graph.nodeCount();
    if (!dragActive() || m_suspended || static_cast<int>(applied.size()) != n ||
        static_cast<int>(solved.size()) != n || m_pinLive.size() != m_pins.size() ||
        m_liveSpan.size() != m_pins.size() || m_contactBlocked.size() != static_cast<std::size_t>(n)) {
        return false;
    }
    static const bool kTrace = std::getenv("POSESTUDIO_IK_TRACE") != nullptr ||
                               std::getenv("IK_CONTACT_TRACE") != nullptr;
    // IK_CONTACT_TRACE=2: every candidate joint within 5cm of the floor, per tick.
    static const bool kTraceNear = std::getenv("IK_CONTACT_TRACE") != nullptr &&
                                   std::getenv("IK_CONTACT_TRACE")[0] == '2';
    const float scale = m_sizeScale;
    bool changed = false;   // pins added or removed: the active set and stiffness follow
    bool hullDirty = false; // a pin slid: the support polygon follows
    // 1. LIFT-OFF and SLIDE (see the constants): a live contact the solver holds above its pin
    //    (sustained) is let go, and its joint must clear the floor before it can plant again;
    //    one the solver holds beyond the slip strain along the floor creeps with the body.
    for (std::size_t p = 0; p < m_pins.size();) {
        if (!m_pinLive[p]) {
            ++p;
            continue;
        }
        const int node = m_pins[p].node;
        const glm::vec3& at = solved[static_cast<std::size_t>(node)];
        IkEffector& pin = m_pins[p];
        const int junction = limbJunction(node);
        const float reach = glm::length(solved[static_cast<std::size_t>(junction)] - pin.target);
        const bool taut = p < m_liveSpan.size() && reach > kLiveContactTaut * m_liveSpan[p];
        if (taut && at.y - pin.target.y > kLiveContactLift * scale) {
            if (++m_liveErrTicks[p] >= kLiveContactReleaseTicks) {
                if (kTrace) {
                    std::fprintf(stderr, "[ikrig] CONTACT lift-off node=%d dy=%.4f reach=%.3f span=%.3f\n",
                                 node, at.y - pin.target.y, reach, m_liveSpan[p]);
                }
                m_contactBlocked[static_cast<std::size_t>(node)] = 1;
                removePin(p);
                changed = true;
                continue;
            }
        } else {
            m_liveErrTicks[p] = 0;
        }
        const glm::vec2 slip(at.x - pin.target.x, at.z - pin.target.z);
        const float slipLen = glm::length(slip);
        if (slipLen > kLiveContactSlip * scale) {
            const glm::vec2 move = slip * (1.0f - kLiveContactSlip * scale / slipLen);
            pin.target.x += move.x;
            pin.target.z += move.y;
            hullDirty = true;
            if (kTrace) {
                std::fprintf(stderr, "[ikrig] CONTACT slide node=%d by=%.4f to(%.3f %.3f)\n", node,
                             glm::length(move), pin.target.x, pin.target.z);
            }
        }
        ++p;
    }
    // 2. DETECT: a real-mass body joint (token-mass parts — fingers, toes, face bones — count
    //    through the joint they ride, never on their own) that is not the effector, not within
    //    the drag's own lift reach of it (the same rule that frees a dragged foot's toes at
    //    drag start), not already served by a pin at or above it (a planted foot's toes), and
    //    whose contact height has reached the floor. A pin BELOW it does not exclude it: a knee
    //    coming down onto the floor with its foot still planted is a second support on that
    //    leg (a kneel), and the solver restores the foot's chain through the knee pin.
    for (int c = 0; c < n; ++c) {
        const std::size_t ci = static_cast<std::size_t>(c);
        // A real-mass joint by its own segment OR by what hangs off it (a hand whose class
        // share a helper bone halved still carries its fingers) — the promotion rule's pair.
        const bool realMass =
            m_masses[ci] >= kTokenBoneMass || m_subtreeMass[ci] >= kTokenLimbMass;
        if (!m_bodyNode[ci] || c == m_pelvis || c == m_effector || !realMass ||
            !descendsFromPelvis(c)) {
            continue;
        }
        const float h = contactHeight(c, applied);
        if (kTraceNear && h < 0.05f * scale) {
            std::fprintf(stderr, "[ikrig]   near node=%d h=%.4f blocked=%d reach=%.3f\n", c, h,
                         m_contactBlocked[ci] ? 1 : 0, pathLenToEffector(c));
        }
        if (m_contactBlocked[ci]) {
            if (h > kLiveContactRearm * scale) {
                m_contactBlocked[ci] = 0;
            }
            continue;
        }
        if (h > kLiveContactBand * scale) {
            continue;
        }
        // The drag's own limb never plants itself (the lift-reach rule of beginDrag) — except
        // under a ROOT drag, whose "limb" is the whole body: the knees are within that reach
        // of the pelvis, and a kneel is exactly a hip drag bringing them down.
        if (m_effector != m_pelvis && pathLenToEffector(c) <= kEffectorLiftReach * scale) {
            continue;
        }
        bool served = false;
        for (const IkEffector& pin : m_pins) {
            for (int cur = c; cur >= 0 && !served; cur = m_parents[static_cast<std::size_t>(cur)]) {
                served = cur == pin.node; // a pin at or above c
            }
            if (served) {
                break;
            }
        }
        if (served) {
            continue;
        }
        glm::vec3 target = applied[ci];
        target.y -= h; // the lowest riding part exactly at its clearance (h <= band: a lift)
        if (kTrace) {
            std::fprintf(stderr, "[ikrig] CONTACT plant node=%d h=%.4f at(%.3f %.3f %.3f)\n", c,
                         h, target.x, target.y, target.z);
        }
        addLivePin(c, target, applied);
        changed = true;
    }
    if (changed) {
        rebuildActiveSet();
        rebuildPriorWeights();
    }
    if (changed || hullDirty) {
        rebuildSupportHull(m_stepPin);
    }
    return changed;
}

} // namespace pose
