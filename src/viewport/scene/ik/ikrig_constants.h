/**
 * @file ikrig_constants.h
 * @brief The IkRig's tuning constants, shared by its four translation units (ikrig.cpp,
 *        ikrigdrag.cpp, ikrigsolve.cpp, ikrigstepping.cpp). PRIVATE to the rig — not part of the
 *        scene/ik interface, so nothing outside those files may include it.
 *
 * Every value here was set by a measured failure in the IK harness (tools/ikharness) and the
 * comment at each definition records which; change one only with the harness re-run. The
 * constants sit in an anonymous namespace so each TU gets its own internal copy, exactly as it
 * did when the rig was one file — the rig's numerics must stay bit-identical to the harness
 * baseline, and the function-local trace probes (std::getenv) stay where they are used.
 */

#ifndef IKRIG_CONSTANTS_H
#define IKRIG_CONSTANTS_H

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

} // namespace pose

#endif // IKRIG_CONSTANTS_H
