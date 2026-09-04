/**
 * @file ikrig.h
 * @brief The full-body-IK orchestrator: binds the skeleton graph (step 1), the FABRIK solver
 *        (step 2), the derived joint constraints (step 3), and the balance controller (step 4)
 *        into one drag-driven solve the Model can call per mouse move.
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
 * model space; the Model converts them back to its per-joint Euler pose. Qt-free (std + GLM).
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

/// Bind-skeleton description of one bone, as the Model hands it to IkRig::build().
struct IkRigBone {
    std::string name;
    int         parent = -1;         ///< Anatomical parent index (-1 = skeleton root).
    glm::vec3   bindPos{0.0f};       ///< Model-space bind joint position.
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
    /// positions; @p contactNodes = joints currently planted on the ground (the Model detects
    /// them by world height). @p groundOffsetY is the MODEL-space height of the world floor
    /// (the Model's transform may translate the figure vertically — the Ground button does —
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

    /// Per node (parallel to the skeleton), 1 = on a USER pin's LIMB chain for the current drag:
    /// the pin up to (excluding) the junction where its limb joins the axial skeleton — the same
    /// mass rule as the trunk chain, so a pinned hand's chain runs through the fingers' branching
    /// up to the collar. The Model exempts these joints from its drag-tick rotational prior: the
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
    /// hold. The Model's per-tick drift bound measures against this.
    const glm::vec3& settleEffectorTarget() const { return m_settleEffectorTarget; }

    void endDrag() {
        m_effector = -1;
        m_imbalanceTicks = 0; // a mid-confirm step trigger dies with the drag (stepPending())
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
    /// thresholds. Perceptual per-tick speed caps (the Model's governors) deliberately do not.
    float sizeScale() const { return m_sizeScale; }

    /// Which nodes the current drag's solve moves (paths between effector, pins, pelvis, and the
    /// anchor root); everything else keeps its local pose and rides along.
    const std::vector<char>& activeNodes() const { return m_active; }

    /// The current drag's pins (planted contacts / the airborne root fallback) — the Model's
    /// progress-based governor measures goal error against these.
    const std::vector<IkEffector>& pins() const { return m_pins; }

    /// A joint's floor clearance (see FabrikSettings::floorClearance): its rest height above
    /// the floor for a ground contact, a flesh radius otherwise. The Model clamps the drag
    /// target with it so the cursor cannot ask for a joint below the floor either.
    float floorClearance(int node) const {
        return (node >= 0 && node < static_cast<int>(m_floorClearance.size()))
                   ? m_floorClearance[static_cast<std::size_t>(node)]
                   : 0.0f;
    }

    /// The balance SUPPORT POLYGON of the current drag (model-space XZ, convex hull of the
    /// planted contacts; empty outside a drag or while suspended) — the overlay draws it.
    const std::vector<glm::vec2>& supportHull() const { return m_supportHull; }

    /// The FBIK solve root (the "pelvis": first multi-child descendant of the anatomical root —
    /// real figures root at an origin-level FIGURE NODE with the hip as its lone child). The
    /// Model absorbs the solved root displacement into THIS bone's pose translation.
    int rootNode() const { return m_pelvis; }

    /// True when the edge from @p child's anatomical parent to @p child is a RIGID link: the
    /// parent joint's swing axes are locked (a mid-limb twist bone), so the child's position
    /// follows the chain rigidly — rotation about the bone axis can't move an along-axis child.
    /// The Model's rotation extraction aims THROUGH such links (at the first joint whose position
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
    /// default Free constraint for an invalid index). The Model's extraction reads it for the
    /// TWIST WITNESS (dominantBendTangent): a twist bone's rotation about its own segment is
    /// fitted from the plane the solve bent its grandchild in.
    const JointConstraint& edgeConstraint(int child) const {
        static const JointConstraint kFree{};
        return (child >= 0 && child < static_cast<int>(m_edgeConstraint.size()))
                   ? m_edgeConstraint[static_cast<std::size_t>(child)]
                   : kFree;
    }

    /// The node the current drag actually solves for. Usually the grabbed joint passed to
    /// beginDrag(), but a token-mass grab (a finger, a toe, a face bone) is PROMOTED to the
    /// limb's first real-mass joint (the hand, the foot, the head) — a finger pull is an ARM
    /// gesture. The Model offsets its drag targets by the grab offset when these differ.
    int dragEffector() const { return m_effector; }

    /// The node of the foot currently mid-STEP (balance-driven re-plant), or -1. While a step
    /// is in flight the Model must not settle-freeze (the swing needs the solve ticks) and must
    /// not enforce that foot's flat-sole orientation (it is swinging).
    int steppingPin() const {
        return m_stepPin >= 0 ? m_pins[static_cast<std::size_t>(m_stepPin)].node : -1;
    }

    /// True while a step is in flight OR its trigger is confirming. The Model must not
    /// settle-freeze while this holds: a pelvis-walk's final GATHERING step (the trailing foot
    /// coming to the target-centered stance) fires only from live solve ticks, and a freeze
    /// racing the confirm counter left the figure planted in a mid-stride stance.
    bool stepPending() const { return m_stepPin >= 0 || m_imbalanceTicks > 0; }

    /// Balance steps completed during the current drag (diagnostics / tests).
    int stepsTaken() const { return m_stepsTaken; }

    /// The node where @p node's limb joins the AXIAL skeleton — the first ancestor whose
    /// off-path descendants carry real body mass (the upper chest for anything on an arm, the
    /// pelvis for a leg or a spine bone). The Model's pin refinement corrects a pin through
    /// exactly this chain (the pin up to, excluding, the junction): the limb serves its own pin
    /// and the trunk is never recruited for it.
    int limbJunction(int node) const { return limbJunction(node, m_subtreeMass); }

private:
    // Built once per figure:
    SkeletonGraph                m_graph;
    std::vector<int>             m_parents;        ///< Anatomical hierarchy.
    std::vector<glm::vec3>       m_bindPos;        ///< Model-space bind joint positions.
    std::vector<glm::vec3>       m_edgeRestDir;    ///< Per bone: rest direction parent -> bone.
    std::vector<float>           m_edgeRestLen;    ///< Per bone: rest length of that edge.
    std::vector<JointConstraint> m_edgeConstraint; ///< Per bone: constraint of that edge.
    // Per bone: the built twist range of a NEAR-HINGE cone edge (a knee), re-applied per drag:
    // the knee's fold-plane freedom serves gestures that drive the legs (a trunk drag's crouch
    // or walk, a foot drag) and is switched off for other limb drags — under a strained hand
    // pull the legs used it to give laterally, the feet slipped 8cm off their pins, and the
    // stepper fired mid-pull. True hinges (elbows) keep theirs always.
    std::vector<glm::vec2>       m_edgeTwistBuilt;
    std::vector<float>           m_masses;         ///< Per bone: segment mass (balance).
    std::vector<float>           m_subtreeMass;    ///< Per bone: own + descendants' mass.
    std::vector<float>           m_floorClearance; ///< Per bone: see floorClearance().
    std::vector<char>            m_bodyNode;       ///< False for the figure-node chain above the
                                                   ///< pelvis: no edges, contacts, or mass.
    int                          m_pelvis = -1;    ///< The FBIK root (see rootNode()).
    float                        m_sizeScale = 1.0f; ///< See sizeScale().
    float                        m_groundOffsetY = 0.0f; ///< Model-space world-floor height (per drag).

    // Per-drag state. Pin targets AND the pose prior are captured at DRAG START and never re-read
    // from the pose: re-reading each move would let the per-move extraction residual (Euler
    // clamps make the FK pose differ slightly from the solved positions) relocate the stance,
    // ratcheting the figure across the floor while the cursor holds still.
    int                     m_effector = -1; ///< The solved effector — see dragEffector().
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

    /// The node where @p node's limb joins the AXIAL skeleton: the first ancestor whose OFF-PATH
    /// descendants carry real body mass (per @p subtreeMass, own + descendants); the pelvis when
    /// none does. Mass, not branching, identifies the junction — see m_trunkChain.
    int limbJunction(int node, const std::vector<float>& subtreeMass) const;

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
