/**
 * @file ikrig.h
 * @brief The full-body-IK orchestrator: binds the skeleton graph (step 1), the FABRIK solver
 *        (step 2), the derived joint constraints (step 3), and the balance controller (step 4)
 *        into one drag-driven solve the Armature can call per mouse move.
 *
 * Built once per figure from its bind skeleton (graph topology, per-edge rest directions,
 * hinge/cone constraints derived from the authored joint limits, segment masses). The solve is
 * rooted at the PELVIS — rootNode(), the first multi-child descendant of the anatomical root;
 * real figures root at an origin-level figure node whose chain of virtual ancestors is excluded
 * from the rig wholesale (no graph edge, no contact eligibility, no balance mass) — so traversal
 * order equals hierarchy order and every constraint is evaluated in its owning parent-side
 * frame. The root FLOATS: ground anchoring is the PINS' job.
 *
 * Per drag: beginDrag() detects which joints are planted on the ground (minus contacts within a
 * limb's length of the effector, by tree path — dragging a foot must lift the foot AND its whole
 * toe subtree, not fight its own pins), pins each limb's most proximal planted contact at its
 * ground-healed height, gives each pin a
 * reach LEASH (the root may never leave the intersection of the planted limbs' reach balls),
 * captures the drag-start pose as the solver's soft prior with per-node stiffness weights, and
 * builds the balance support polygon over all contacts. solveDrag() then runs the constrained
 * multi-chain FABRIK toward the drag target, measures the solved center of mass against the
 * support polygon, and — when the pose has gone off balance — re-solves with a soft pelvis
 * effector shifted back over the polygon (the auto-balance loop). At mouse-up, beginSettle() +
 * settleToPins() run the animated release settle: the pins land while the released joint is held
 * at its mouse-up position and the body eases toward the release pose. Positions in/out are
 * model space; the Armature converts them back to its per-joint Euler pose. Qt-free (std + GLM).
 *
 * The implementation is split across five translation units by drag phase: ikrig.cpp (build +
 * the shared metric helpers), ikrigdrag.cpp (beginDrag), ikrigsolve.cpp (solveDrag and the
 * release settle), ikrigstepping.cpp (balance-driven stepping) and ikrigcontacts.cpp (live
 * contact re-detection mid-drag); their shared tuning constants live in the private
 * ikrig_constants.h.
 */

#ifndef IKRIG_H
#define IKRIG_H

#include "fabriksolver.h"
#include "ikconstraints.h"
#include "skeletongraph.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <string>
#include <vector>

namespace pose {

/// Bind-skeleton description of one bone, as the Armature hands it to IkRig::build().
struct IkRigBone {
    std::string name;
    int         parent = -1;         ///< Anatomical parent index (-1 = skeleton root).
    glm::vec3   bindPos{0.0f};       ///< Bind joint position (model space).
    glm::mat3   orientAxes{1.0f};    ///< Oriented rotation axes (columns; the `orient` rotation).
    glm::vec3   rotMinDeg{0.0f};     ///< Authored per-axis limits (degrees) ...
    glm::vec3   rotMaxDeg{0.0f};
    glm::bvec3  rotLimited{false, false, false}; ///< ... and which axes carry them.
};

/**
 * @class IkRig
 * @brief Per-figure FBIK state: build once, then per interactive gesture beginDrag ->
 *        solveDrag per tick -> (mouse-up) beginSettle -> settleToPins per tick -> endDrag.
 */
class IkRig {
public:
    /// Builds the graph, per-edge constraints, and segment masses from the bind skeleton.
    void build(const std::vector<IkRigBone>& bones);
    bool built() const { return !m_parents.empty(); }

    /// Starts an interactive drag of @p effectorNode. @p positions = current model-space joint
    /// positions; @p contactNodes = joints currently planted on the ground (the Armature detects
    /// them by world height). @p groundOffsetY is the MODEL-space height of the world floor
    /// (the Armature's transform may translate the figure vertically — the Ground button does —
    /// and every bind-height ground reference here must shift with it, or pins heal to a floor
    /// that no longer matches the visible one). Picks the anchor root, pins, active subgraph,
    /// and support polygon. @p userPins (optional) are joints the USER pinned: each is held at
    /// its current position as a hard pin — never ground-healed, never released by the drag's
    /// lift reach, never moved by the balance stepper, and kept through suspension (an explicit
    /// pin is intent). A user pin on the effector itself or in its subtree is ignored for that
    /// drag (dragging a pinned hand moves the pin). Returns false if the rig isn't built or the
    /// effector is invalid.
    bool beginDrag(int effectorNode, const std::vector<glm::vec3>& positions,
                   const std::vector<int>& contactNodes, float groundOffsetY = 0.0f,
                   const std::vector<int>* userPins = nullptr);

    /// True if pin @p index (into pins()) is a USER pin rather than a ground contact.
    bool pinIsUser(std::size_t index) const {
        return index < m_pinUser.size() && m_pinUser[index] != 0;
    }

    /// LIVE CONTACT RE-DETECTION — call once per drag tick with the APPLIED model-space pose
    /// (after the governors and the pin refinement: what the user sees is what touches the
    /// floor). A real-mass joint whose lowest riding part — itself or a token-mass descendant
    /// that rides it rigidly: the fingers under a hand, the face under the head, the toes under
    /// a foot — has come down to the floor becomes a contact PIN at that height: the hands in a
    /// deep crouch, a knee coming down. Its limb chain goes active and holds it, the support
    /// polygon grows around it, and the body can lean on it — instead of the subtree sinking
    /// through the floor as it rides the trunk down (the solver's floor rule covers ACTIVE
    /// joints only, and the drag-start contacts are the only pins there were). Unlike those
    /// contacts a live one is a UNILATERAL support: no reach leash (standing back up must be
    /// able to lift a hand off the floor), never stepped, and RELEASED again once the body
    /// pulls it off its spot — the SOLVER's own pin error (@p solved, this tick's solve
    /// output) sustained above a few centimetres: the applied pose lags a freshly planted
    /// hand by design while the extraction folds the arm at its capped rate, and reading that
    /// transient as a pull let go of every hand three ticks after it planted — after which the
    /// joint must clear the floor by a margin before it can re-plant. Returns true when the pin
    /// set changed (pins() is re-read by the caller's per-pin state).
    bool updateContacts(const std::vector<glm::vec3>& applied,
                        const std::vector<glm::vec3>& solved);

    /// True if pin @p index (into pins()) is a LIVE contact (see updateContacts).
    bool pinIsLive(std::size_t index) const {
        return index < m_pinLive.size() && m_pinLive[index] != 0;
    }

    /// Per node (parallel to the skeleton), 1 = on a USER pin's LIMB chain for the current drag:
    /// the pin up to (excluding) the junction where its limb joins the axial skeleton — the same
    /// mass rule as the trunk chain, so a pinned hand's chain runs through the fingers' branching
    /// up to the collar. The Armature exempts these joints from its drag-tick rotational prior: the
    /// pin's restoration determines them every tick, and a prior decaying toward the drag-START
    /// pose could only pull the limb off its pin.
    const std::vector<char>& userPinLimbNodes() const { return m_userPinLimb; }

    /// One drag update: solves @p positions (model space, in place) toward @p target with the
    /// pins held and auto-balancing active. @p frameSeed = per-bone current rest->posed rotation
    /// (model space). Returns false when no drag is in progress.
    bool solveDrag(const glm::vec3& target, std::vector<glm::vec3>& positions,
                   const std::vector<glm::quat>& frameSeed);

    /// Starts the RELEASE settle: captures @p positions (the pose at mouse-up) as the settle's
    /// pose prior AND pins the drag effector at its release position. The settle must plant the
    /// ground pins while HOLDING the pose the user just made — the user placed that joint there
    /// deliberately, and an unguarded pins-only solve pays for millimeters of pin progress with
    /// centimeters of unrelated body drift (a lifted foot visibly sagged after release).
    void beginSettle(const std::vector<glm::vec3>& positions);

    /// One settle step (no drag goal): relaxes the body onto its planted contacts with the
    /// effector held at its release position and the release pose as a soft prior (pin-serving
    /// chains stay nearly prior-free so limbs can articulate to land). Call beginSettle() first.
    /// Returns false without pins or an active drag.
    bool settleToPins(std::vector<glm::vec3>& positions, const std::vector<glm::quat>& frameSeed);

    /// The effector's release position captured by beginSettle() — the position the settle must
    /// hold. The Armature's per-tick drift bound measures against this.
    const glm::vec3& settleEffectorTarget() const { return m_settleEffectorTarget; }

    void endDrag() {
        m_effector = -1;
        m_imbalanceTicks = 0; // a mid-confirm step trigger dies with the drag (stepPending())
        m_stepPin = -1;       // steppingPin() stays callable after the drag: no stale index
        m_suspended = false;
    }
    bool dragActive() const { return m_effector >= 0; }

    /// True when the current drag's effector carries real body mass below it (its subtree over
    /// ~5% of the body: the hip, the spine, the chest, the head; a thigh) — a TRUNK gesture that
    /// moves the body, as opposed to a LIMB gesture placing a hand or a foot. The release settle's
    /// pose-hold bound (the released joint stays within 2cm of where it was let go) applies to
    /// limb effectors only: planting the feet after a trunk drag necessarily moves the trunk,
    /// and the absolute bound left the figure standing 4cm in the air after a stepping chest drag.
    bool effectorIsTrunk() const {
        return m_effector >= 0 && static_cast<std::size_t>(m_effector) < m_subtreeMass.size() &&
               m_subtreeMass[static_cast<std::size_t>(m_effector)] > 0.05f;
    }

    /// The figure's size relative to the adult reference the rig's world-space tuning constants
    /// were calibrated on (pelvis bind height / ~1m, clamped). Every GEOMETRIC threshold —
    /// contact release reach, ground-heal bands, step geometry, suspension strain — scales by
    /// it, so a 0.54-scale child crouches and walks instead of losing its pins to adult-sized
    /// thresholds. Perceptual per-tick speed caps (the Armature's governors) deliberately do not.
    float sizeScale() const { return m_sizeScale; }

    /// Which nodes the current drag's solve moves (paths between effector, pins, pelvis, and the
    /// anchor root); everything else keeps its local pose and rides along.
    const std::vector<char>& activeNodes() const { return m_active; }

    /// The current drag's pins (planted contacts / the airborne root fallback) — the Armature's
    /// progress-based governor measures goal error against these.
    const std::vector<IkEffector>& pins() const { return m_pins; }

    /// A joint's floor clearance (see FabrikSettings::floorClearance): its rest height above
    /// the floor for a ground contact, a flesh radius otherwise. The Armature clamps the drag
    /// target with it so the cursor cannot ask for a joint below the floor either.
    float floorClearance(int node) const {
        return (node >= 0 && node < static_cast<int>(m_floorClearance.size()))
                   ? m_floorClearance[static_cast<std::size_t>(node)]
                   : 0.0f;
    }

    /// The FBIK solve root (the "pelvis": first multi-child descendant of the anatomical root —
    /// real figures root at an origin-level FIGURE NODE with the hip as its lone child). The
    /// Armature absorbs the solved root displacement into THIS bone's pose translation.
    int rootNode() const { return m_pelvis; }

    /// True when the edge from @p child's anatomical parent to @p child is a RIGID link: the
    /// parent joint's swing axes are locked (a mid-limb twist bone), so the child's position
    /// follows the chain rigidly — rotation about the bone axis can't move an along-axis child.
    /// The Armature's rotation extraction aims THROUGH such links (at the first joint whose position
    /// actually responds to the fitted rotation): aiming a thigh at its twist joint — a point ON
    /// the thigh line — constrains only 2 of 3 rotation DoF, and the unconstrained one is exactly
    /// what places the shin.
    bool edgeRigid(int child) const {
        if (child < 0 || child >= static_cast<int>(m_edgeConstraint.size())) {
            return false;
        }
        const JointConstraint& c = m_edgeConstraint[static_cast<std::size_t>(child)];
        return c.type == JointConstraint::Type::Cone && !c.perAxis && c.coneHalfAngle <= 1e-5f;
    }

    /// The positional constraint of the edge from @p child's anatomical parent to @p child (a
    /// default Free constraint for an invalid index). The Armature's extraction reads it for the
    /// TWIST WITNESS (dominantBendTangent): a twist bone's rotation about its own segment is
    /// fitted from the plane the solve bent its grandchild in.
    const JointConstraint& edgeConstraint(int child) const {
        static const JointConstraint kFree{};
        return (child >= 0 && child < static_cast<int>(m_edgeConstraint.size()))
                   ? m_edgeConstraint[static_cast<std::size_t>(child)]
                   : kFree;
    }

    /// True when the last solveDrag ran under UPWARD intent — the trunk excluded from the solve
    /// so a raised limb works against a fixed chest. The Armature's drag finisher must not
    /// recruit the trunk then either (it would bow the spine the solve deliberately held).
    bool lastSolveExcludedTrunk() const { return m_lastUpIntent; }

    /// The node the current drag actually solves for. Usually the grabbed joint passed to
    /// beginDrag(), but a token-mass grab (a finger, a toe, a face bone) is PROMOTED to the
    /// limb's first real-mass joint (the hand, the foot, the head) — a finger pull is an ARM
    /// gesture. The Armature offsets its drag targets by the grab offset when these differ.
    int dragEffector() const { return m_effector; }

    /// The node of the foot currently mid-STEP (balance-driven re-plant), or -1. While a step
    /// is in flight the Armature must not settle-freeze (the swing needs the solve ticks) and must
    /// not enforce that foot's flat-sole orientation (it is swinging).
    int steppingPin() const {
        return m_stepPin >= 0 ? m_pins[static_cast<std::size_t>(m_stepPin)].node : -1;
    }

    /// True while a step is in flight OR its trigger is confirming. The Armature must not
    /// settle-freeze while this holds: a pelvis-walk's final GATHERING step (the trailing foot
    /// coming to the target-centered stance) fires only from live solve ticks, and a freeze
    /// racing the confirm counter left the figure planted in a mid-stride stance.
    bool stepPending() const { return m_stepPin >= 0 || m_imbalanceTicks > 0; }

    /// True while a LIFT (suspension) is confirming. The Armature must not settle-freeze then:
    /// a target that became unreachable only as the cursor came to rest — the older figure
    /// generations' lower-hanging hands — had its fifteen confirm ticks cut short by the freeze
    /// and the figure never lifted.
    bool suspendPending() const { return !m_suspended && m_suspendTicks > 0; }

    /// Balance steps completed during the current drag (diagnostics / tests).
    int stepsTaken() const { return m_stepsTaken; }

    /// The node where @p node's limb joins the AXIAL skeleton — the first ancestor whose
    /// off-path descendants carry real body mass (the upper chest for anything on an arm, the
    /// pelvis for a leg or a spine bone). The Armature's pin refinement corrects a pin through
    /// exactly this chain (the pin up to, excluding, the junction): the limb serves its own pin
    /// and the trunk is never recruited for it. Mass (m_subtreeMass), not branching, identifies
    /// the junction — see m_trunkChain.
    int limbJunction(int node) const;

private:
    // Built once per figure:
    SkeletonGraph                m_graph;
    std::vector<int>             m_parents;        ///< Anatomical hierarchy.
    std::vector<std::vector<int>> m_children;      ///< Per bone: anatomical children, ascending.
    std::vector<glm::vec3>       m_bindPos;        ///< Bind joint positions (model space).
    std::vector<glm::vec3>       m_edgeRestDir;    ///< Per bone: rest direction parent -> bone.
    std::vector<float>           m_edgeRestLen;    ///< Per bone: rest length of that edge.
    std::vector<JointConstraint> m_edgeConstraint; ///< Per bone: constraint of that edge.
    std::vector<float>           m_masses;         ///< Per bone: segment mass (balance).
    std::vector<float>           m_subtreeMass;    ///< Per bone: own + descendants' mass.
    std::vector<float>           m_floorClearance; ///< Per bone: see floorClearance().
    std::vector<char>            m_bodyNode;       ///< False for the figure-node chain above the
                                                   ///< pelvis: no edges, contacts, or mass.
    /// Per bone: its token-mass descendants — the parts that ride it rigidly whenever it is not
    /// itself on an effector path (a hand's fingers, the head's face bones, a foot's toes). The
    /// live contact detection reads a joint's height through them: a hand has "reached the
    /// floor" when its lowest fingertip has.
    std::vector<std::vector<int>> m_ridingDescendants;
    int                          m_pelvis = -1;    ///< The FBIK root (see rootNode()).
    float                        m_sizeScale = 1.0f; ///< See sizeScale().
    float                        m_groundOffsetY = 0.0f; ///< World-floor height in model space (per drag).

    // Per-drag state. Pin targets AND the pose prior are captured at DRAG START and never re-read
    // from the pose: re-reading each move would let the per-move extraction residual (Euler
    // clamps make the FK pose differ slightly from the solved positions) relocate the stance,
    // ratcheting the figure across the floor while the cursor holds still.
    int                     m_effector = -1; ///< The solved effector — see dragEffector().
    float                   m_effectorChainLen = 0.0f; ///< pathLenToRoot(m_effector): drag-invariant.
    /// The effector's ancestor chain (effector first) with the cumulative rest length to each:
    /// the tree-path metric of pathLenToEffector(), captured once per drag after the promotion.
    std::vector<int>        m_effAncestor;
    std::vector<float>      m_effAncestorLen;
    std::vector<IkEffector> m_pins;        ///< Planted contacts (or the airborne root fallback).
    std::vector<char>       m_active;
    std::vector<glm::vec2>  m_supportHull; ///< XZ support polygon of the planted contacts.
    std::vector<glm::vec3>  m_startPose;   ///< Drag-start positions: the solver's soft pose prior.
    // Release-settle state (beginSettle): the pose at mouse-up is the settle's prior — NOT the
    // drag-start pose, which would yank the achieved pose back toward where the drag began.
    std::vector<glm::vec3>  m_settlePrior;
    std::vector<float>      m_settlePriorWeights;
    glm::vec3               m_settleEffectorTarget{0.0f};
    // Per-node prior weights = the stiffness model: weight grows with graph distance from the
    // dragged effector, so the grabbed limb moves freely while the trunk and far limbs resist —
    // the dragged limb moves first and the body follows reluctantly, like real recruitment.
    std::vector<float>      m_priorWeights;
    // Smoothed balance correction (XZ), low-passed across ticks; its magnitude drives the
    // balance ENGAGEMENT — the continuous blend factor of the balanced re-solve (see the
    // balance block in solveDrag). Applying/dropping the pelvis effector the instant the CoM
    // crossed the hull edge restructured the solve per tick, and the trunk (with every idle
    // limb riding on it) JERKED between the two solutions.
    glm::vec2               m_balanceCorrection{0.0f};
    std::vector<glm::vec3>  m_balancedScratch; ///< The balanced re-solve's output (blend source).
    // SUSPENSION (lift-off): a sustained, mostly-vertical pull beyond the leashed body's reach
    // means the user is deliberately lifting the figure — the pins release, the root rises with
    // the drag, and a per-iteration gravity bias makes the whole (fully active) body settle
    // hanging below the grab point, limbs dangling within their joint limits.
    bool                    m_suspended = false;
    int                     m_suspendTicks = 0;
    bool                    m_lastUpIntent = false; ///< See lastSolveExcludedTrunk().
    float                   m_suspendHang = 0.0f; ///< Grab-point→root chain length (root hangs here).
    // True when this drag began with ground-healed (snapped) pins — a hovering figure being
    // pulled back onto the floor. The pelvis effector then fires every tick (Y at the healed
    // prior height) instead of waiting for a balance need: that steady downward driver is what
    // completes the descent.
    bool                    m_healing = false;
    // BALANCE-DRIVEN STEPPING (foot re-planting): when a sustained drag holds the CoM outside
    // the support polygon beyond what leaning can absorb, the figure re-plants a foot at its
    // BALANCED stance position — the drag-start stance shape re-centered on the current CoM —
    // with an animated swing (eased glide + lift arc on the pin target). Captured at drag
    // start: which pins are steppable standing feet, each pin's XZ offset from the stance
    // center (the stance SHAPE), and each pin's footprint (its limb's planted-contact offsets,
    // so the support polygon can be rebuilt around a moved pin).
    std::vector<std::vector<glm::vec2>> m_pinFootprint;
    std::vector<glm::vec2>              m_pinStanceOffset;
    /// The stance center's drag-start XZ offset from the ROOT: an explicit (pelvis-drag)
    /// anchor is offset by it, so the walk re-creates the stance the figure actually held
    /// under its hip (the feet naturally sit a few centimetres behind it) rather than one
    /// centered under the hip. Balance-driven steps stay centered on the CoM itself — that IS
    /// the re-centering they exist for (offsetting them cost two of a chest drag's four steps).
    glm::vec2                           m_stanceRootOffset{0.0f};
    std::vector<char>                   m_pinSteppable;
    std::vector<char>                   m_pinUser; ///< Parallel to m_pins: 1 = user pin.
    std::vector<char>                   m_pinLive; ///< Parallel to m_pins: 1 = live contact.
    std::vector<int>                    m_liveErrTicks; ///< Parallel: ticks over the release error.
    /// Parallel: a live pin's limb REST SPAN — the straight-line length of its chain's rest
    /// offsets from the pin up to its junction (a hanging arm's shoulder-to-hand distance):
    /// the socket farther from the pin than this is a limb being pulled taut.
    std::vector<float>                  m_liveSpan;
    /// Per node: 1 while a released live contact must clear the floor before re-planting.
    std::vector<char>                   m_contactBlocked;
    std::vector<char>                   m_userPinLimb; ///< See userPinLimbNodes().
    int       m_stepPin = -1;      ///< Index into m_pins of the foot in flight (-1 = none).
    glm::vec3 m_stepFrom{0.0f};
    glm::vec3 m_stepTo{0.0f};
    float     m_stepHeight = 0.0f;
    int       m_stepTick = 0;
    int       m_stepTotal = 0;
    int       m_imbalanceTicks = 0; ///< Consecutive ticks of above-threshold balance need.
    int       m_stepsTaken = 0;

    /// Rebuilds m_supportHull from the pins' footprints at their CURRENT targets, excluding
    /// @p excludePin's contribution (the foot in flight supports nothing).
    void rebuildSupportHull(int excludePin);

    /// Summed bone rest lengths from @p node up to the pelvis (the one metric distance used by
    /// leashes, reach clamps, and suspension alike).
    float pathLenToRoot(int node) const;

    /// Summed bone rest lengths along the tree path from @p node to the drag effector (up one
    /// side to their lowest common ancestor, down the other — see m_effAncestor); a huge value
    /// for a node outside the body tree. The contact-release reach is measured with it.
    float pathLenToEffector(int node) const;

    /// True when @p node hangs below the pelvis in the anatomical tree: what a ground contact
    /// must be (a merged follower's scene node sits beside the pelvis at the origin and once
    /// read as "planted").
    bool descendsFromPelvis(int node) const;

    /// A joint's height above the floor through its lowest riding part (see
    /// m_ridingDescendants), each part measured against its own floor clearance: 0 = touching.
    float contactHeight(int node, const std::vector<glm::vec3>& positions) const;

    /// Adds a LIVE contact pin at @p node held at @p target (see updateContacts) with its
    /// footprint from the riding parts near the floor; the caller rebuilds the derived state.
    void addLivePin(int node, const glm::vec3& target, const std::vector<glm::vec3>& positions);

    /// Removes pin @p index from m_pins and every array parallel to it.
    void removePin(std::size_t index);

    /// m_active = the paths joining the effector and every pin to the root.
    void rebuildActiveSet();

    /// m_priorWeights = the stiffness model (see the member note) for the current effector and
    /// pin set: weight by metric distance from the effector, pin-serving chains nearly free,
    /// the root stiffest of all.
    void rebuildPriorWeights();

    /// Socket-centered reach leash for a pin at @p node held at @p target, in the pose
    /// @p positions: the limb hangs from its SOCKET (the root's child on the pin's chain), so
    /// the ball confines the socket — radius = the socket-to-pin distance in this pose (a
    /// stance the figure provably holds) with fractional headroom for a bent start — and is
    /// expressed on the root through the root->socket offset written to @p offsetOut (see
    /// IkEffector::leashOffset). A root-centered ball let the root sit on its far side with the
    /// socket genuinely out of reach: a planted foot 8cm short under a hard lean.
    float socketLeash(int node, const glm::vec3& target, const std::vector<glm::vec3>& positions,
                      glm::vec3& offsetOut) const;

    /// Lands the in-flight step: pin onto m_stepTo, leash recomputed against the current root,
    /// support hull rebuilt, step + trigger state cleared. Shared by the swing's final tick and
    /// beginSettle's instant completion at mouse-up.
    void landStep(const std::vector<glm::vec3>& positions);

    /// Advances an in-flight step / evaluates the step trigger (see the .cpp). @p entryPinErr =
    /// the worst steppable pin's distance from its target at SOLVE ENTRY (the FK pose): a foot
    /// being dragged off its plant is the strain half of the step trigger. @p anchorXZ, when
    /// non-null, is the EXPLICIT stance anchor — a pelvis drag's target, where the user is
    /// taking the body — that steps aim at and are reach-clamped against; null anchors on the
    /// CoM (upper-body drags, where balance is the intent signal).
    void updateStepping(const std::vector<glm::vec3>& positions, bool allowTrigger,
                        float entryPinErr, const glm::vec2* anchorXZ);

    // The TRUNK segment of the dragged limb's path: from the node where the limb joins the
    // AXIAL skeleton (the first ancestor whose off-path descendants carry real body mass — the
    // upper chest for anything on an arm, fingers included; the pelvis for a leg) down to the
    // root. Under UPWARD drag intent these
    // nodes are stiffened to near-rigid (m_scratchWeights): pulling a hand up is arm +
    // shoulder-girdle work — a spine cannot lengthen — and letting the solve recruit the spine
    // pitched the chest and swung the head down (the "straight-up pull bends her over" repro).
    // Forward/downward pulls keep the normal weights (trunk flexion is how those recruit).
    std::vector<int>        m_trunkChain;
    std::vector<float>      m_scratchWeights;
    std::vector<char>       m_scratchActive; ///< m_active minus the trunk (upward-intent solves).
};

} // namespace pose

#endif // IKRIG_H
