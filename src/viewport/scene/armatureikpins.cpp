/**
 * @file armatureikpins.cpp
 * @brief Armature::refinePins — EXACT enforcement of the joint pins in the applied pose: a
 *        damped-least-squares fit in joint space (numeric Jacobian, limits respected, iterated to
 *        0.1mm) of each pin's own limb chain onto its pin, with a second root-translation stage
 *        for user pins the joints cannot serve, plus the drag-tick FLOOR LIFT and the exact
 *        drag-target finisher.
 *
 * The solver holds a pin in POSITION space, but the applied pose is what the user sees, and the
 * extraction, the angular caps and above all the governor's joint-space under-relaxation land a
 * pinned joint millimetres off every tick. This pass closes that residual: user pins every tick;
 * the drag's contact pins and the released joint only in the release settle (band-gated and
 * per-tick capped so the landing stays animated — exact contacts DURING the drag erased the FK
 * slip that is the balance stepper's strain signal); and the grabbed joint onto the cursor as a
 * FINISHER (full within 2cm, fading to nothing by 6cm). Two lessons recorded inline: the Jacobian
 * is the RESIDUAL's derivative (J·dθ = −r), and a pin's orientation must be served by the whole
 * limb, not the pin's own channels. Vulkan-free, Qt-free.
 */

#include "armature.h"

#include "ikrig.h" // pins / pinIsUser / limbJunction / steppingPin / effectorIsTrunk

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstddef>
#include <cstdlib>
#include <utility>
#include <vector>

namespace pose {

namespace {

// Solves the small dense system A·y = b in place (Gaussian elimination with partial pivoting);
// @p A is n×n row-major, @p b becomes y. Returns false on a singular system.
bool solveDense(std::vector<float>& A, std::vector<float>& b, int n) {
    for (int col = 0; col < n; ++col) {
        int pivot = col;
        for (int row = col + 1; row < n; ++row) {
            if (std::abs(A[static_cast<std::size_t>(row * n + col)]) >
                std::abs(A[static_cast<std::size_t>(pivot * n + col)])) {
                pivot = row;
            }
        }
        const float pv = A[static_cast<std::size_t>(pivot * n + col)];
        if (std::abs(pv) < 1e-12f) {
            return false;
        }
        if (pivot != col) {
            for (int k = 0; k < n; ++k) {
                std::swap(A[static_cast<std::size_t>(col * n + k)],
                          A[static_cast<std::size_t>(pivot * n + k)]);
            }
            std::swap(b[static_cast<std::size_t>(col)], b[static_cast<std::size_t>(pivot)]);
        }
        for (int row = col + 1; row < n; ++row) {
            const float f = A[static_cast<std::size_t>(row * n + col)] / pv;
            if (f == 0.0f) {
                continue;
            }
            for (int k = col; k < n; ++k) {
                A[static_cast<std::size_t>(row * n + k)] -= f * A[static_cast<std::size_t>(col * n + k)];
            }
            b[static_cast<std::size_t>(row)] -= f * b[static_cast<std::size_t>(col)];
        }
    }
    for (int row = n - 1; row >= 0; --row) {
        float s = b[static_cast<std::size_t>(row)];
        for (int k = row + 1; k < n; ++k) {
            s -= A[static_cast<std::size_t>(row * n + k)] * b[static_cast<std::size_t>(k)];
        }
        b[static_cast<std::size_t>(row)] = s / A[static_cast<std::size_t>(row * n + row)];
    }
    return true;
}

// Exact user-pin refinement (Armature::refinePins): iterations, convergence tolerance (m), the
// numeric-Jacobian probe (degrees), the per-iteration channel step cap (degrees — residuals are
// millimetres, so steps are fractions of a degree; the cap only guards a near-singular chain),
// the damped-least-squares damping (m/deg — the Jacobian entries are lever × π/180, ~0.01 for
// a shin, so this damps ~5%), the orientation residual's scale (metres per radian: 1° of pin
// rotation counts like 3.5mm — position wins when a limit forces a compromise, so a pinned
// joint holds its PLACE first and its orientation as far as the limb allows), and the root
// translation's unit (metres per DoF unit: a 2mm pelvis shift costs what a 1° rotation does,
// i.e. ~5× per centimetre of effect — the joints absorb the residual first).
constexpr int   kPinRefineIters = 10;
constexpr float kPinRefineTol = 1e-4f;
constexpr float kPinJacobianDeltaDeg = 0.2f;
constexpr float kPinRefineMaxStepDeg = 3.0f;
constexpr float kPinRefineDamping = 0.002f;
constexpr float kPinOrientScale = 0.2f;
constexpr float kPinRootUnit = 0.002f;
// CONTACT-pin refinement (the drag's ground contacts and the settle's held effector, which are
// soft in the solver; RELEASE SETTLE only — see refinePins): the residual band — a contact
// farther off than this is genuine catch-up (the settle's animated landing), NOT extraction
// residual, and is left alone; the per-tick correction cap, so the last centimetre of a landing
// stays eased instead of snapping; the orientation band, inside which the sole's drag-start
// orientation is re-imposed exactly (outside it the extraction's capped flat-sole hold brings
// it in); and the row weight that makes a USER pin outrank a contact sharing its chain.
// 2cm (was 1cm): the solver may leave the pelvis bent between its two hip sockets, and the
// extraction's rigid pelvis (bisector fit) then lands each foot up to ~1.3cm off after a
// stepping lean on the newest and oldest rigs — extraction residual, exactly what this closes.
constexpr float kContactRefineBand = 0.020f;
constexpr float kContactRefineStep = 0.003f;
constexpr float kContactRefineRotBand = 0.05235988f; // 3 degrees
constexpr float kPinUserRowWeight = 4.0f;
// FLOOR LIFT (drag ticks — see refinePins): a contact pin whose APPLIED position has sunk below
// its pin height by more than the tolerance is lifted back up to that height through its own
// limb, vertically only, at most this far per tick. The solver keeps the feet on their pins to
// a millimetre; it is the governor's joint-space under-relaxation that buries them: a crouch
// lowers the hip's translation LINEARLY with the applied fraction, but a near-straight leg
// shortens only QUADRATICALLY with its knee angle, so applying half of each leaves the foot
// below the floor every tick, and the sink compounded to 4-6cm in a 12cm hip crouch. Lateral
// slip is deliberately left alone: that is the balance stepper's strain signal.
// Position only, deliberately: re-flattening the lifted SOLE here too (its drag-start
// orientation, by a capped 3 deg/tick step) was built and measured — it halved the crouch's toe
// dip (the sole pitches a few degrees through a crouch and the toe joints, 18cm ahead of the
// ankle, dip ~2cm below their rest height, about a centimetre through the floor) but made the
// figure laterally STIFF: a person leaning sideways rolls onto the foot's edge, and with the
// soles held exactly flat the 45cm lateral chest drag leaned 4cm less, never reached the
// balance-effort step trigger, and ended 26cm short instead of 13cm. The extraction's capped,
// relaxed sole hold is the right softness for that roll; the toe dip stays a known soft spot
// (gated in the harness).
constexpr float kFloorLiftTol = 0.001f;
constexpr float kFloorLiftStep = 0.010f;
// LIVE contacts (a hand or knee that reached the floor mid-drag — IkRig::updateContacts) are
// held in FULL position through their limb every drag tick, capped per tick and band-limited,
// unlike the drag-start feet: they carry no stepper strain signal to preserve, and the solver
// alone cannot be trusted to keep them — an arm's FABRIK chain hangs off the upper chest and
// the clavicle (15° and 27° cones on 10cm bones) through a kinked rigid wrist link, and under
// a descending trunk its restoration rounds diverge from a correct two-bone seed, leaving the
// solved hand 6-9cm above a pin the joints can plainly serve (the all-fours harness phase).
// The joint-space fit folds the elbow instead. Beyond the band the gap is the solver's.
constexpr float kLiveRefineStep = 0.020f;
constexpr float kLiveRefineBand = 0.10f;
// The DRAG target's correction (the grabbed joint is closed onto the cursor every drag tick, see
// refinePins) is a FINISHER: full within kDragRefineFull of the target, fading linearly to
// nothing at kDragRefineFade. Per-tick residuals of an ordinary drag (the relaxation's leftover,
// ~1-2cm) close exactly; a large gap (a fast flick, a 15cm foot lift) stays with the solve's own
// posture choice until the limb is nearly there.
constexpr float kDragRefineFull = 0.02f;
constexpr float kDragRefineFade = 0.06f;
// The drag finisher also gets the TRUNK — the joints from the limb's junction with the axial
// skeleton up to (excluding) the solve root — as extra degrees of freedom at this cost per
// unit rotation relative to a limb channel (a weighted least-squares fit: the limb takes
// ~16/17 of a residual it can serve, the trunk only what the limb cannot). A hand pulled 10cm
// sideways at constant height sits at the edge of the arm's reach; the limb-only fit blocked
// at the collar and left the hand 14mm short of the cursor — where the solve's own trunk
// stiffness had settled — for good ("the hand never quite arrives"). A degree or two of spine
// closes it exactly, as a person leans a hair to reach. Not under UPWARD intent (the solve
// held the trunk fixed on purpose: a raised limb works against a fixed chest).
constexpr float kDragTrunkWeight = 4.0f;

} // namespace

void Armature::refinePins(bool settling, const glm::vec3* dragTarget) {
    if (!m_ikRig || !m_ikRig->dragActive() || m_bones.empty()) {
        return;
    }
    const int n = static_cast<int>(m_bones.size());
    const std::vector<IkEffector>& pins = m_ikRig->pins();
    const int stepping = m_ikRig->steppingPin();
    struct PinRef {
        int              node = -1;
        glm::vec3        target{0.0f};
        std::vector<int> chain; // the pin's parent first, up to (excluding) the limb junction
        std::vector<int> trunk; // the drag target only: the junction up to (excluding) the root
        int              flat = -1; // index into m_ikFlatNodes/m_ikFlatRot (orientation), or -1
        int              row = 0;   // first residual row
        int              rows = 3;  // 3 = position only, 6 = position + orientation
        float            weight = 1.0f; // residual row weight (user pins outrank contacts)
        bool             drag = false;   // the drag target: never served by the root stage
    };
    std::vector<PinRef> refs;
    int m = 0; // total residual rows
    bool anyUser = false; // the root stage serves USER pins only (see below)
    // Angle (radians) between @p node's current world rotation and its drag-start one.
    const auto flatRotError = [&](int node, int flat) {
        const glm::mat3 rel = glm::transpose(m_ikFlatRot[static_cast<std::size_t>(flat)]) *
                              glm::mat3(m_poseGlobal[static_cast<std::size_t>(node)]);
        const float c = glm::clamp((rel[0][0] + rel[1][1] + rel[2][2] - 1.0f) * 0.5f, -1.0f, 1.0f);
        return std::acos(c);
    };

    // kind: 0 = contact pin (exact in the settle; floor-lifted during the drag), 1 = user pin,
    // 2 = the drag target, 3 = a LIVE contact (a contact pin held in full during the drag too).
    const auto addRef = [&](int node, const glm::vec3& target, int kind) {
        const bool user = kind == 1;
        if (node < 0 || node >= n || node == stepping) {
            return; // a foot mid-STEP is swinging: nothing to hold it to yet
        }
        for (const PinRef& r : refs) {
            if (r.node == node) {
                return;
            }
        }
        PinRef ref;
        ref.node = node;
        int flat = -1;
        for (std::size_t f = 0; f < m_ikFlatNodes.size(); ++f) {
            if (m_ikFlatNodes[f] == node) {
                flat = static_cast<int>(f);
                break;
            }
        }
        if (user) {
            ref.target = target;
            ref.weight = kPinUserRowWeight;
            ref.flat = flat;
            anyUser = true;
        } else if (kind == 2) {
            // The DRAG TARGET itself (see armature.h): the grabbed joint is closed onto the
            // filtered cursor EXACTLY every drag tick through its own limb — the pin machinery
            // with the cursor as the pin. Whatever part of the target the limb can reach from
            // the current trunk it reaches now; the trunk, legs, and balance keep following on
            // the damped FABRIK dynamics underneath. Never band-gated (closing the gap IS the
            // point), no orientation rows, capped only against warps.
            const glm::vec3 pos(m_poseGlobal[static_cast<std::size_t>(node)][3]);
            const glm::vec3 delta = target - pos;
            const float dist = glm::length(delta);
            const float w = glm::clamp((kDragRefineFade - dist) / (kDragRefineFade - kDragRefineFull),
                                       0.0f, 1.0f);
            if (w <= 0.0f) {
                return; // far off: the solve's own posture choice drives this gap
            }
            ref.target = pos + delta * w;
            ref.flat = -1;
            ref.drag = true;
        } else {
            // CONTACT pins (the drag's ground contacts, the settle's held effector): held
            // exactly too, but ONLY IN THE RELEASE SETTLE, and only as RESIDUAL cleanup. During
            // the drag their PLACE is left to the solver: an exact per-tick hold there erased
            // the FK slip that IS the balance stepper's strain signal (the foot held until the
            // strain exceeded the band, then jumped 2cm in one tick and a strained release
            // landed 2cm off instead of 5mm), and the pelvis stage fought the solver's root
            // (fast-drag catch-up never completed, walk landings fell 2cm short) — the drag
            // ticks only apply the vertical FLOOR LIFT below. At rest — the settle — exactness
            // is the contract. A contact farther off than the band is genuine catch-up (the
            // settle's own animated landing) and is left alone; the correction is capped per
            // tick so the last centimetre lands eased (a 6-8mm cap measurably worsened the
            // settle's per-tick residual); and the sole's orientation is re-imposed only once
            // the extraction's flat-sole hold has brought it within the rotation band.
            const glm::vec3 pos(m_poseGlobal[static_cast<std::size_t>(node)][3]);
            if (kind == 3) {
                // A LIVE contact (see kLiveRefineStep): full position, capped, band-limited —
                // in the release settle too: the settle's solve lets a planted hand's diverging
                // arm chain lift it several centimetres while the feet land (the monotone
                // guard watches only the WORST pin), and the contact band below is too narrow
                // to bring it back (the oldest rigs' hands rose 5cm off the floor on release).
                const glm::vec3 delta = target - pos;
                const float dist = glm::length(delta);
                if (dist <= kFloorLiftTol || dist > kLiveRefineBand) {
                    return;
                }
                // The settle keeps its per-tick landing pace (the 2cm settle cap is a contract
                // — a 2cm refinement step on top of it moved joints 3.2cm in one settle tick).
                const float step = settling ? kContactRefineStep : kLiveRefineStep;
                ref.target = dist > step ? pos + delta * (step / dist) : target;
                ref.flat = -1;
            } else if (!settling) {
                // DRAG ticks: the FLOOR LIFT only (see kFloorLiftTol). A planted contact that
                // the applied pose has pushed below its pin height is raised back to that
                // height — through its own leg, vertically, keeping whatever lateral slip the
                // strain produced — so a crouch never buries a foot, tick after tick.
                const float sink = target.y - pos.y;
                if (sink <= kFloorLiftTol) {
                    return;
                }
                ref.target = glm::vec3(pos.x, pos.y + std::min(sink, kFloorLiftStep), pos.z);
                ref.flat = -1;
            } else {
                const glm::vec3 delta = target - pos;
                const float dist = glm::length(delta);
                if (dist > kContactRefineBand) {
                    return;
                }
                ref.target = dist > kContactRefineStep
                                 ? pos + delta * (kContactRefineStep / dist)
                                 : target;
                ref.flat = (flat >= 0 && flatRotError(node, flat) < kContactRefineRotBand)
                               ? flat
                               : -1;
            }
        }
        // The pin's limb chain (see IkRig::limbJunction): its parent up to, excluding, the
        // junction where the limb joins the axial skeleton. A pin ON the solve root has no
        // chain (the root's virtual ancestors — the figure node — must never be rotated).
        const int junction = m_ikRig->limbJunction(node);
        const int rigRoot = m_ikRig->rootNode();
        for (int cur = m_bones[static_cast<std::size_t>(node)].parent;
             cur >= 0 && cur != junction && cur != rigRoot;
             cur = m_bones[static_cast<std::size_t>(cur)].parent) {
            ref.chain.push_back(cur);
        }
        if (kind == 2 && !m_ikRig->lastSolveExcludedTrunk()) {
            // The trunk, weighted (see kDragTrunkWeight): what the limb cannot close, the spine
            // does — never under an upward-intent solve, whose fixed trunk is deliberate, and
            // never a joint that carries a PIN: a foot drag's "trunk" is the pelvis bone, whose
            // rotation swings the standing leg off its plant while that leg has no row in this
            // fit (contacts are only floor-lifted during a drag); the spine above the hip
            // carries no foot, and a pinned hand on it has its rows here.
            for (int cur = junction; cur >= 0 && cur != rigRoot;
                 cur = m_bones[static_cast<std::size_t>(cur)].parent) {
                bool carriesPin = false;
                for (const IkEffector& pin : pins) {
                    for (int up = pin.node; up >= 0 && !carriesPin;
                         up = m_bones[static_cast<std::size_t>(up)].parent) {
                        carriesPin = up == cur;
                    }
                }
                if (!carriesPin) {
                    ref.trunk.push_back(cur);
                }
            }
        }
        ref.rows = ref.flat >= 0 ? 6 : 3;
        ref.row = m;
        m += ref.rows;
        refs.push_back(std::move(ref));
    };
    for (std::size_t p = 0; p < pins.size(); ++p) {
        addRef(pins[p].node, pins[p].target,
               m_ikRig->pinIsUser(p) ? 1 : (m_ikRig->pinIsLive(p) ? 3 : 0));
    }
    if (settling && m_ikRig->dragEffector() >= 0 && !m_ikRig->effectorIsTrunk()) {
        // The release settle holds the let-go joint at its mouse-up position (the pose
        // contract): exact, through its own limb, like a contact. (Holding it with the drag
        // finisher instead was measured and rejected: the wider range let the settle snap the
        // arm 11cm in one tick.)
        addRef(m_ikRig->dragEffector(), m_ikRig->settleEffectorTarget(), 0);
    }
    if (!settling && dragTarget != nullptr && m_ikRig->dragEffector() >= 0 &&
        m_ikRig->dragEffector() != m_ikRig->rootNode()) {
        // A pelvis drag places the root directly in the solve; every other effector is closed
        // onto the cursor here.
        addRef(m_ikRig->dragEffector(), *dragTarget, 2);
    }
    if (refs.empty()) {
        return;
    }

    // Degrees of freedom, in two stages. Stage 1: every unlocked Euler channel of every chain
    // joint (union over pins — overlapping chains, e.g. a pinned hand and a pinned finger, are
    // solved jointly), plus the pin's OWN channels for an oriented pin (they steer its
    // orientation; position rows read 0). Stage 2, only if the joints cannot close the residual
    // (a limit-blocked chain, or a straight leg that cannot lengthen when the pelvis landed a
    // few millimetres too far from the planted foot): the solve root's pose TRANSLATION — the
    // pelvis shifts by the millimetres the anchored limb demands, as a real body's does. Its
    // unit (kPinRootUnit) prices a pelvis shift well above a joint rotation of equal effect.
    // The root stage exists for USER pins only: a pelvis shift on behalf of a CONTACT moved
    // the settle's other landings (a walk's trailing foot ended 2.6cm from where it had been
    // landing) — contacts get the joints, and what the joints cannot close stays.
    struct Dof {
        int  joint;
        int  axis;
        bool translation;
    };
    std::vector<Dof>   dofs;
    std::vector<float> dofWeight; // per dof: the least-squares cost per unit rotation (>= 1)
    std::vector<int>   dofFirst(m_bones.size(), -1); // per joint: its first dof index, or -1
    const auto addJointDofs = [&](int j, float weight) {
        const int first = dofFirst[static_cast<std::size_t>(j)];
        if (first >= 0) {
            // Already a dof (a joint on one pin's limb and another's trunk): the lighter cost.
            for (std::size_t d = static_cast<std::size_t>(first); d < dofs.size() && dofs[d].joint == j; ++d) {
                dofWeight[d] = std::min(dofWeight[d], weight);
            }
            return;
        }
        dofFirst[static_cast<std::size_t>(j)] = static_cast<int>(dofs.size());
        const Bone& b = m_bones[static_cast<std::size_t>(j)];
        for (int a = 0; a < 3; ++a) {
            if (b.rotLimited[a] && b.rotMax[a] - b.rotMin[a] < 1e-3f) {
                continue; // locked channel
            }
            dofs.push_back({j, a, false});
            dofWeight.push_back(weight);
        }
    };
    for (const PinRef& ref : refs) {
        for (const int j : ref.chain) {
            addJointDofs(j, 1.0f);
        }
        if (ref.rows == 6) {
            addJointDofs(ref.node, 1.0f);
        }
    }
    for (const PinRef& ref : refs) {
        for (const int j : ref.trunk) {
            addJointDofs(j, kDragTrunkWeight);
        }
    }
    const std::size_t rotDofs = dofs.size();
    const int rigRoot = m_ikRig->rootNode();
    if (anyUser && rigRoot >= 0 && rigRoot < n) {
        for (int a = 0; a < 3; ++a) {
            dofs.push_back({rigRoot, a, true});
            dofWeight.push_back(1.0f);
        }
    }
    const std::size_t D = dofs.size();
    // True when rotating @p joint moves @p ref's pin: the joint is the pin itself or one of
    // its ancestors. Kinematic truth, not chain membership — the drag target's trunk joints
    // move a hand pinned on the same trunk, and the fit must know it to keep that pin.
    const auto influences = [&](const PinRef& ref, const Dof& dof) {
        if (dof.translation) {
            // The root stage serves USER pins only — never the drag target: the pelvis is the
            // FABRIK layer's decision, and letting the cursor recruit it here dragged a pinned
            // foot 6mm off its pin under a chest drag (the two rows competed for the root).
            return !ref.drag;
        }
        for (int cur = ref.node; cur >= 0; cur = m_bones[static_cast<std::size_t>(cur)].parent) {
            if (cur == dof.joint) {
                return true;
            }
        }
        return false;
    };
    // The pin's world matrix with every poseLocal from joint @p top (an ancestor of the pin,
    // or the pin itself) down to the pin re-composed; m_poseGlobal above @p top is read as-is.
    const auto pinPoseFrom = [&](const PinRef& ref, int top) {
        int path[64];
        int len = 0;
        int cur = ref.node;
        while (cur >= 0 && cur != top && len < 64) {
            path[len++] = cur;
            cur = m_bones[static_cast<std::size_t>(cur)].parent;
        }
        if (cur != top) {
            return m_poseGlobal[static_cast<std::size_t>(ref.node)]; // not an ancestor: unmoved
        }
        const int parentOfTop = m_bones[static_cast<std::size_t>(top)].parent;
        glm::mat4 g = parentOfTop >= 0 ? m_poseGlobal[static_cast<std::size_t>(parentOfTop)]
                                       : glm::mat4(1.0f);
        g = g * m_bones[static_cast<std::size_t>(top)].poseLocal;
        for (int k = len - 1; k >= 0; --k) {
            g = g * m_bones[static_cast<std::size_t>(path[k])].poseLocal;
        }
        return g;
    };
    // Residual rows of @p ref for world matrix @p g: the position error, then (oriented pins)
    // the rotation vector taking g's rotation onto the drag-start rotation, scaled to metres.
    const auto residual = [&](const PinRef& ref, const glm::mat4& g, float* out) {
        const glm::vec3 pos(g[3]);
        for (int c = 0; c < 3; ++c) {
            out[c] = ref.target[c] - pos[c];
        }
        if (ref.rows == 6) {
            const glm::mat3 R(g);
            const glm::mat3& F = m_ikFlatRot[static_cast<std::size_t>(ref.flat)];
            glm::quat q = glm::quat_cast(F * glm::transpose(R));
            if (q.w < 0.0f) {
                q = -q;
            }
            const glm::vec3 v(q.x, q.y, q.z);
            const float vl = glm::length(v);
            const glm::vec3 omega =
                vl > 1e-9f ? v * (2.0f * std::atan2(vl, q.w) / vl) : glm::vec3(0.0f);
            for (int c = 0; c < 3; ++c) {
                out[3 + c] = omega[c] * kPinOrientScale;
            }
        }
    };
    // Full pose refresh (bones are parent-first): cheap, and the root stage moves everything.
    const auto refreshAll = [&]() {
        for (int i = 0; i < n; ++i) {
            const int p = m_bones[static_cast<std::size_t>(i)].parent;
            m_poseGlobal[static_cast<std::size_t>(i)] =
                (p >= 0 ? m_poseGlobal[static_cast<std::size_t>(p)] : glm::mat4(1.0f)) *
                m_bones[static_cast<std::size_t>(i)].poseLocal;
        }
    };
    const auto worstResidual = [&](std::vector<float>& res) {
        float worst = 0.0f;
        for (const PinRef& ref : refs) {
            residual(ref, m_poseGlobal[static_cast<std::size_t>(ref.node)],
                     &res[static_cast<std::size_t>(ref.row)]);
            for (int k = 0; k < ref.rows; ++k) {
                worst = std::max(worst, std::abs(res[static_cast<std::size_t>(ref.row + k)]));
            }
        }
        return worst;
    };
    // Row weights: where a user pin and a contact share a chain (a pinned knee over a planted
    // foot), the least-squares compromise favours the user's pin — it is the declared intent.
    std::vector<float> rowWeight(static_cast<std::size_t>(m), 1.0f);
    for (const PinRef& ref : refs) {
        for (int k = 0; k < ref.rows; ++k) {
            rowWeight[static_cast<std::size_t>(ref.row + k)] = ref.weight;
        }
    }

    // Three stages, each a FALLBACK for the one before: the limbs; then the drag target's
    // trunk; then the root translation (user pins). A later stage opens only once the earlier
    // one has stalled — its channels blocked at their limits, or its iterations spent above
    // tolerance. The trunk in particular must never share an ordinary tick's correction: a
    // weighted fit gave it 1/17 of every residual, and over a sustained tracking drag (the hand
    // 8mm behind a rising cursor, closed every tick) that share accumulated into a 7cm head
    // drift the solve's own trunk prior could not take back.
    std::vector<char> blocked(D, 0); // channels the limit clamp stopped: dropped from the fit
    bool anyTrunk = false;
    for (std::size_t d = 0; d < rotDofs; ++d) {
        if (dofWeight[d] > 1.0f) {
            blocked[d] = 1; // the trunk waits for the limb stage to stall
            anyTrunk = true;
        }
    }
    for (std::size_t d = rotDofs; d < D; ++d) {
        blocked[d] = 1; // stage 1: joints only
    }
    bool trunkStage = !anyTrunk;
    bool rootStage = false;
    const auto enableTrunk = [&]() {
        trunkStage = true;
        for (std::size_t d = 0; d < rotDofs; ++d) {
            if (dofWeight[d] > 1.0f) {
                blocked[d] = 0;
            }
        }
    };
    const auto enableRoot = [&]() {
        rootStage = true;
        for (std::size_t d = rotDofs; d < D; ++d) {
            blocked[d] = 0;
        }
    };
    std::vector<float> res(static_cast<std::size_t>(m), 0.0f);
    std::vector<float> J;
    std::vector<float> A;
    std::vector<float> y;
    float tmp[6];
    int   itersUsed = 0;
    int   stageStart = 0; // the iteration the current stage opened at
    for (int iter = 0; iter < 3 * kPinRefineIters && D > 0; ++iter) {
        if (worstResidual(res) < kPinRefineTol) {
            break;
        }
        if (iter - stageStart >= kPinRefineIters) {
            if (!trunkStage) {
                enableTrunk();
                stageStart = iter;
            } else if (!rootStage) {
                enableRoot();
                stageStart = iter;
            } else {
                break;
            }
        }
        ++itersUsed;
        // Jacobian of the residual, rows = residual components, columns = channels: numeric
        // for the rotation channels; analytic for the root translation (a unit shift along
        // the root's parent-frame axis moves every pin by exactly that, orientations untouched).
        // Rows are scaled by their weight (a weighted least-squares fit).
        J.assign(static_cast<std::size_t>(m) * D, 0.0f);
        for (std::size_t d = 0; d < D; ++d) {
            if (blocked[d]) {
                continue;
            }
            const int j = dofs[d].joint;
            const int a = dofs[d].axis;
            if (dofs[d].translation) {
                const int rp = m_bones[static_cast<std::size_t>(j)].parent;
                const glm::mat3 parentRot =
                    rp >= 0 ? glm::mat3(m_poseGlobal[static_cast<std::size_t>(rp)]) : glm::mat3(1.0f);
                const glm::vec3 shift = parentRot[a] * kPinRootUnit;
                for (const PinRef& ref : refs) {
                    if (ref.drag) {
                        continue; // see influences(): the drag target never moves the root
                    }
                    for (int c = 0; c < 3; ++c) {
                        J[static_cast<std::size_t>(ref.row + c) * D + d] =
                            -shift[c] * rowWeight[static_cast<std::size_t>(ref.row + c)];
                    }
                }
                continue;
            }
            const float save = m_boneEuler[static_cast<std::size_t>(j)][a];
            m_boneEuler[static_cast<std::size_t>(j)][a] = save + kPinJacobianDeltaDeg;
            recomposePoseLocal(static_cast<std::size_t>(j));
            // Column scaled by the dof's cost (a weighted least-squares fit: the step below is
            // divided by it again, so an expensive channel moves 1/cost^2 as much per unit of
            // residual as a free one).
            const float colScale = 1.0f / (kPinJacobianDeltaDeg * dofWeight[d]);
            for (const PinRef& ref : refs) {
                if (!influences(ref, dofs[d])) {
                    continue;
                }
                residual(ref, pinPoseFrom(ref, j), tmp);
                for (int k = 0; k < ref.rows; ++k) {
                    J[static_cast<std::size_t>(ref.row + k) * D + d] =
                        (tmp[k] - res[static_cast<std::size_t>(ref.row + k)]) * colScale *
                        rowWeight[static_cast<std::size_t>(ref.row + k)];
                }
            }
            m_boneEuler[static_cast<std::size_t>(j)][a] = save;
            recomposePoseLocal(static_cast<std::size_t>(j));
        }
        // Damped least squares: dTheta = -J^T (J J^T + lambda^2 I)^-1 r (J is the residual's
        // derivative).
        A.assign(static_cast<std::size_t>(m * m), 0.0f);
        y.assign(static_cast<std::size_t>(m), 0.0f);
        for (int i = 0; i < m; ++i) {
            for (int k = 0; k < m; ++k) {
                float s = 0.0f;
                for (std::size_t d = 0; d < D; ++d) {
                    s += J[static_cast<std::size_t>(i) * D + d] * J[static_cast<std::size_t>(k) * D + d];
                }
                A[static_cast<std::size_t>(i * m + k)] = s;
            }
            A[static_cast<std::size_t>(i * m + i)] += kPinRefineDamping * kPinRefineDamping;
            y[static_cast<std::size_t>(i)] =
                -res[static_cast<std::size_t>(i)] * rowWeight[static_cast<std::size_t>(i)];
        }
        if (!solveDense(A, y, m)) {
            break;
        }
        bool moved = false;
        for (std::size_t d = 0; d < D; ++d) {
            if (blocked[d]) {
                continue;
            }
            float dTheta = 0.0f;
            for (int k = 0; k < m; ++k) {
                dTheta += J[static_cast<std::size_t>(k) * D + d] * y[static_cast<std::size_t>(k)];
            }
            dTheta /= dofWeight[d]; // the weighted fit's second division (see colScale)
            dTheta = glm::clamp(dTheta, -kPinRefineMaxStepDeg, kPinRefineMaxStepDeg);
            if (std::abs(dTheta) < 1e-6f) {
                continue;
            }
            const int j = dofs[d].joint;
            const int a = dofs[d].axis;
            if (dofs[d].translation) {
                m_boneTranslation[static_cast<std::size_t>(j)][a] += dTheta * kPinRootUnit;
                recomposePoseLocal(static_cast<std::size_t>(j));
                moved = true;
                continue;
            }
            const float requested = m_boneEuler[static_cast<std::size_t>(j)][a] + dTheta;
            m_boneEuler[static_cast<std::size_t>(j)][a] = requested;
            clampBoneEuler(j); // the authoritative anatomical limits
            if (std::abs(m_boneEuler[static_cast<std::size_t>(j)][a] - requested) > 1e-4f) {
                blocked[d] = 1; // at its limit in the needed direction: let the others absorb it
            }
            recomposePoseLocal(static_cast<std::size_t>(j));
            moved = true;
        }
        if (!moved) {
            if (!trunkStage) {
                enableTrunk(); // the limb has nothing left to give: the trunk stage
                stageStart = iter;
                continue;
            }
            if (!rootStage) {
                enableRoot(); // the joints have nothing left to give: the pelvis stage
                stageStart = iter;
                continue;
            }
            break;
        }
        refreshAll();
    }
    static const bool kPinTrace = std::getenv("POSESTUDIO_IK_PIN_TRACE") != nullptr;
    if (kPinTrace) {
        const float worst = worstResidual(res);
        if (worst > 5e-4f) {
            std::fprintf(stderr, "[pin] residual %.5f after %d iters (root stage %d), blocked:",
                         worst, itersUsed, rootStage ? 1 : 0);
            for (std::size_t d = 0; d < rotDofs; ++d) {
                if (blocked[d]) {
                    std::fprintf(stderr, " %s.%c",
                                 m_boneNames[static_cast<std::size_t>(dofs[d].joint)].c_str(),
                                 "xyz"[dofs[d].axis]);
                }
            }
            std::fprintf(stderr, "\n");
        }
    }
    computeSkinMatrices();
}

} // namespace pose
