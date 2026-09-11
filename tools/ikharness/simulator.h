/**
 * @file simulator.h
 * @brief The drag-loop simulator: the viewport's IK tick, without the viewport. A Scenario
 *        scripts a cursor path over a grabbed joint (plus optional user pins, an FK pre-pose, a
 *        hover, a figure scale and cursor noise); runScenario drives the REAL Armature loop
 *        (beginIkDrag -> dragIkTo per tick through the shared IkCursorFilter -> the animated
 *        release settle -> endIkDrag) and measures everything the real-skeleton phases gate.
 *
 * Metric semantics worth knowing before gating on them: contactMinY / contactBindMinY are mins
 * over DIFFERENT joints (the lowest contact pin reached vs the lowest contact pin's rest height),
 * so their difference only detects sinking below the LOWEST pin's bind height — exact for two
 * same-height feet, weaker for knee + foot pins; penetrationMax is the per-joint check against
 * the rig's own floor clearance. `suspended` reads "every pin released while contacts existed",
 * which cannot detect a suspension that a user pin survives. runIdleMetrics is a second,
 * identical pass that keeps per-joint step histories for the idle-joint tremble metrics, so the
 * main loop stays a plain mirror of the viewport's tick.
 */

#ifndef IKHARNESS_SIMULATOR_H
#define IKHARNESS_SIMULATOR_H

#include "armature.h"

#include <glm/glm.hpp>

#include <string>
#include <utility>
#include <vector>

namespace ikharness {

struct Waypoint {
    int         tick;   // reached at this tick (tick 0 = the grab)
    glm::vec3   offset; // cursor offset from the grab point (world, metres)
    const char* label;  // the segment ending here
};

struct Scenario {
    std::string           grab;                 // the joint under the cursor
    std::vector<Waypoint> path;                 // starts with {0, 0, "start"}
    std::vector<std::string> userPins;          // pinned before the drag
    std::vector<std::pair<std::string, glm::vec3>> prePose; // FK pre-pose (bone, euler)
    float                 hover = 0.0f;         // root pose translation Y before the drag (a hovering figure)
    float                 scale = 1.0f;         // skeleton scale (0.54 = a child)
    float                 cursorNoise = 0.0f;   // +-metres of per-tick cursor noise
    bool                  release = true;       // run the release settle
    std::vector<std::string> idleJoints;        // oscillation is measured on these
    // The X/Y/Z wheel during the drag: at each listed tick the GRABBED bone's Euler pose is
    // nudged by the delta (degrees), exactly as the window's wheel handler does.
    std::vector<std::pair<int, glm::vec3>> nudges;
};

struct PhaseStats {
    std::string label;
    int         ticks = 0;
    double      lagSum = 0.0, lagMax = 0.0;
    double      effOsc = 0.0;          // grabbed joint's reversal overlap (the app's osc)
    double      maxJump = 0.0;         // worst single-tick joint displacement
    double      effSpeedUpMax = 0.0;   // worst single-tick increase of the grabbed joint's speed
    double      effSpeedDownMax = 0.0; // worst single-tick decrease
    double      cursorSpeedUpMax = 0.0, cursorSpeedDownMax = 0.0;
    int         movedTicks = 0;        // ticks the solve changed the pose
    double      lagMean() const { return ticks > 0 ? lagSum / ticks : 0.0; }
};

struct RunResult {
    bool                     ok = false;
    std::string              error;
    std::vector<PhaseStats>  phases;
    int                      dragTicks = 0;
    // Contact pins (ground contacts the rig pinned at drag start) — drift from their start.
    std::vector<int>         contactPins;
    double                   contactDriftMax = 0.0;   // after the settle
    double                   contactDriftMaxDrag = 0.0; // worst during the drag
    // Mins over DIFFERENT joints (the lowest pin reached vs the lowest pin's rest height): their
    // difference only detects sinking below the LOWEST pin's bind height — exact for two
    // same-height feet, weaker for knee + foot pins; penetrationMax is the per-joint check.
    double                   contactMinY = 1e9;       // lowest a contact pin went (world)
    double                   contactBindMinY = 1e9;   // its rest height (the floor reference)
    // Displacements of named joints (start -> end).
    double                   hipDisp = 0.0, hipDispY = 0.0;
    double                   headDisp = 0.0, chestDisp = 0.0;
    // Effector.
    int                      grabIndex = -1;
    glm::vec3                grabStart{0.0f}, grabEnd{0.0f}, cursorEnd{0.0f};
    glm::vec3                grabEulerStart{0.0f}, grabEulerEnd{0.0f}; // the grabbed bone's pose (deg)
    double                   restingMiss = 0.0;       // |grab - cursor| at the end of the drag
    double                   holdDrift = 0.0;         // effector motion through the settle
    // Settle.
    int                      settleTicks = 0;
    double                   settleMaxStep = 0.0;
    double                   settleWorstPinErr = 0.0; // contact pins after the settle
    // Pins over the drag: the most at once, the count at mouse-up, and the LIVE contacts
    // (joints the rig planted mid-drag when they reached the floor — see
    // IkRig::updateContacts): the most at once and the worst horizontal slip of a live-pinned
    // joint from where it planted (a hand on the floor must not skate).
    int                      pinsMax = 0;
    int                      pinsAtRelease = 0;
    int                      livePinsMax = 0;
    double                   livePinSlideMax = 0.0;
    // The lower hand's world height: its minimum over the drag + settle (a hand planted on
    // the floor sits ~9cm up with its fingers hanging to the floor; lower = fingers through
    // it) and its value at the end.
    double                   handsMinY = 1e9;
    double                   handsEndY = 1e9;
    // The lower knee's world height at the end (a kneel puts it on the floor).
    double                   kneesEndY = 1e9;
    // Stepping / suspension.
    int                      stepsTaken = 0;
    bool                     suspended = false;       // every pin released while contacts existed
                                                      // (misses a lift a user pin survives)
    double                   feetMinY = 1e9;          // both feet at the end (world)
    // User pins.
    double                   userPinStepMax = 0.0;    // worst per-tick displacement of a pinned joint
    double                   userPinDistMax = 0.0;    // worst distance from its pin position
    double                   userPinRotMaxDeg = 0.0;  // worst world-rotation deviation
    // Everything, for ad-hoc checks: at drag start, at mouse-up (before the settle), after it.
    std::vector<glm::vec3>   startPos, dragEndPos, endPos;
    double                   hipDropAtRelease = 0.0;   // hip descent at mouse-up (a crouch's depth)
    // FLOOR PENETRATION: the deepest any body joint went below (floor + its rig clearance) —
    // the solver's own floor rule, so 0 means no joint ever crossed it. Drag and settle.
    double                   penetrationMax = 0.0;
    int                      penetrationNode = -1;
    double                   minJointY = 1e9;         // lowest ANY joint went vs its bind height (drag; --custom info)
    // LEG diagnostics (the "knees twist inward / feet mangled" complaints), per leg over both:
    // kneeTwist* read the thigh twist bone's one free channel — the fold-plane twist the
    // extraction's witness sets — as |twist - drag start|; kneeInward* is the knee's offset from
    // its hip-socket->ankle line TOWARD the midline (valgus, m; negative = bowed outward);
    // kneeOscMax / kneeStepMax are the knees' reversal-overlap tremble and worst single-tick
    // step during the drag; footRot* is the worst world-rotation deviation of a contact-pinned
    // foot from its drag-start orientation (drag + settle) — the flat-sole hold's fidelity.
    double                   kneeTwistMaxDeg = 0.0;
    double                   kneeTwistEndDeg = 0.0;
    double                   kneeInwardMax = -1e9;
    double                   kneeInwardEnd = 0.0;
    double                   kneeOscMax = 0.0;
    double                   kneeStepMax = 0.0;
    double                   footRotMaxDeg = 0.0;
    double                   footRotEndDeg = 0.0;
};

// A second, lighter simulator pass for the IDLE-joint metrics (per-tick steps of joints that
// should not move during a gesture): tremble = accumulated reversal overlap, and the worst
// single-tick step. Kept separate so the main loop stays a faithful mirror of the viewport.
struct IdleMetrics {
    double oscMax = 0.0;   // worst accumulated reversal overlap over the idle joints
    double stepMax = 0.0;  // worst single-tick step of an idle joint
};

/// The bone list with every local bind translation scaled by @p scale (a 0.54 child).
std::vector<pose::ArmatureBone> scaledBones(const std::vector<pose::ArmatureBone>& bones,
                                            float scale);

/// The index of the bone the phases call @p canonical — a G8-generation name (`lHand`,
/// `chest_2`, `lThighTwist`) — on THIS rig, or -1. Every figure generation names the same
/// joints differently (`l_hand` / `spine4` / `l_thightwist1` on the newest one, `chestUpper`
/// on the previous), and the phases are rig-agnostic gestures: the alias table in simulator.cpp
/// maps each canonical name to its equivalents, tried in order after the name itself.
int resolveBone(const pose::Armature& arm, const std::string& canonical);
/// The rig's own name for @p canonical (see resolveBone), or the canonical name when absent.
std::string resolveBoneName(const pose::Armature& arm, const std::string& canonical);

/// The angle (degrees) between two rotation matrices.
double rotationAngleDeg(const glm::mat3& a, const glm::mat3& b);

/// Runs @p sc on a fresh Armature built from @p baseBones and returns every measurement; with
/// @p verbose the per-phase motion profile is printed as it goes.
RunResult runScenario(const std::vector<pose::ArmatureBone>& baseBones, const Scenario& sc,
                      bool verbose);

/// The idle-joint tremble pass (see the header note): the same drag, measuring only the joints
/// named in sc.idleJoints.
IdleMetrics runIdleMetrics(const std::vector<pose::ArmatureBone>& baseBones, const Scenario& sc);

/// start -> @p offset over @p moveTicks, then held for @p holdTicks (labels "move" and "hold").
std::vector<Waypoint> pullPath(const glm::vec3& offset, int moveTicks, int holdTicks);

} // namespace ikharness

#endif // IKHARNESS_SIMULATOR_H
