/**
 * @file armatureik_detail.h
 * @brief The Armature's FBIK governor constants and the wrapped-angle helper, shared by
 *        armatureik.cpp (the drag/settle governors) and armatureikextract.cpp (the rotation
 *        extraction). PRIVATE to those two files — not part of the Armature's interface.
 *
 * Each constant records the user-visible failure that set it; change one only with the IK
 * harness (tools/ikharness) re-run. They sit in an anonymous namespace so each TU gets its own
 * internal copy, exactly as when the integration was one file — the numerics must stay
 * bit-identical to the harness baseline.
 */

#ifndef ARMATUREIK_DETAIL_H
#define ARMATUREIK_DETAIL_H

namespace pose {

namespace {

// A joint whose world height is below this is a ground contact for FBIK anchoring/balance (feet
// and toes standing, knees kneeling; a figure held above the floor has none and anchors at the
// skeleton root). World units (a figure imports at real scale; standing ankles sit ~0.09).
constexpr float kIkContactHeight = 0.15f;

// FBIK angular rate limit: no joint's Euler pose may change by more than this per solve (per
// mouse event). The solver's positional trust region bounds SOLVED displacement, but extraction
// turns positions into rotations, and a small capped motion at the chest is a large swing at the
// end of a long lever (the head, the far hand) — capping at the rotation level is what actually
// bounds visible per-event pose change, and it preserves bone lengths by construction. Mouse
// events are dense (60+/s), so 12 deg/event still allows very fast posing.
constexpr float kIkMaxJointDeltaDeg = 12.0f;

// Shortest signed angular difference a-b in degrees, wrapped to (-180, 180].
inline float wrappedAngleDelta(float a, float b) {
    float d = a - b;
    while (d > 180.0f) d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return d;
}

// FBIK world-space governor: if ANY joint's world position moved farther than this in one solve,
// the whole pose delta is scaled down to fit. Per-joint angular caps alone still COMPOUND down a
// chain (four capped joints swing a hand far); this is the final authority on how fast the pose
// may visibly change per mouse event, so IK can never "jump". Catch-up continues across events
// (the viewport re-issues the drag target on a timer while the button is held).
constexpr float kIkMaxWorldStep = 0.10f;

// Rotational-prior decay per drag tick (see Armature::m_ikStartEuler): how fast fit-undetermined
// rotation components ease back toward the drag-start pose. Strong enough to keep unwitnessed
// twist from ratcheting into the joints' permissive clamp side (the spine bow), weak enough
// that the per-tick lag it adds to determined components is invisible.
constexpr float kIkRotationPrior = 0.05f;

// FBIK under-relaxation: the fraction of each solve's pose delta actually applied per event.
// 0.5 exactly cancels period-2 oscillation (a solver alternating between two near-equal
// configurations tick to tick — visible as trembling); convergence still completes across the
// 60 Hz drag ticks.
constexpr float kIkRelaxation = 0.5f;

// Micro-motion damping (the governor's per-joint soft deadzone), gated on REVERSAL: a joint's
// applied delta is scaled by e / (e + kIkMicroDamp*) — but only when it OPPOSES the previous
// tick's applied delta. The solve -> extract -> FK loop carries a residual tick-to-tick limit
// cycle (near-equal configurations alternating) that the uniform under-relaxation attenuates
// but sustains — at 60 Hz the user sees idle joints SHIMMER while pulling; a reversal IS that
// oscillation, and the soft factor kills a sub-0.1-degree alternation below perception while a
// genuine course change just eases through one damped tick. Gating on magnitude alone was
// wrong: it also shaved a raising arm's steady climb, and the rotational prior's take-back
// then overcame the climb — the hand visibly SANK mid-raise while the cursor rose. Angular in
// degrees, translation in world units.
constexpr float kIkMicroDampDeg = 0.15f;
constexpr float kIkMicroDampTrans = 5e-4f;

// Motion shaping (velocity-limited governor): the per-tick movement allowance may grow at most
// kIkAccel (world units / tick^2) over the PREVIOUS tick's applied speed — a brisk flick ramps
// up over ~5 ticks (~80 ms, a real limb's spin-up) instead of jumping to full rate in one —
// and never exceeds the braking curve sqrt(2 * kIkDecel * goalError) + the tracking allowance,
// so catch-up after the cursor stops DECELERATES smoothly into the pose instead of running at
// a constant rate and stopping flat (the profile harness showed a one-tick 19mm/tick jump and
// a 14mm hard stop on fast gestures — the "mechanical" look). The tracking term (3x target
// motion) keeps the braking curve from throttling pursuit of a MOVING cursor: the lag behind a
// live drag is meant to be consumed at speed.
constexpr float kIkAccel = 0.010f;
constexpr float kIkDecel = 0.004f;
// Symmetric braking limit: the allowance may also SHRINK by at most this per tick, so a
// stopping cursor ramps the pose down instead of collapsing its budget in one tick (the
// adaptive target filter converges fast on a stop, and the tracking term vanishing with it
// measured as a 23mm one-tick speed drop — a visible hitch). Purely an allowance: a pose that
// reaches its goals stops through vanishing solver proposals regardless.
constexpr float kIkBrake = 0.006f;

} // namespace

} // namespace pose

#endif // ARMATUREIK_DETAIL_H
