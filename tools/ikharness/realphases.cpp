/**
 * @file realphases.cpp
 * @brief The real-skeleton phases (see realphases.h). Each reproduces a user-reported behaviour
 *        and gates the measurements that once failed — see CLAUDE.md's FBIK section for the
 *        history behind every number. Gates are regression guards, not specs.
 */

#include "realphases.h"

#include "armature.h"
#include "report.h"
#include "simulator.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

using namespace pose;

namespace ikharness {

namespace {

const std::vector<std::string> kIdleForHandDrag{"head", "rHand", "rFoot", "lFoot", "hip"};
const std::vector<std::string> kIdleForHeadDrag{"lHand", "rHand", "rFoot", "lFoot"};

// The leg diagnostics every real-figure phase reports under --verbose (see RunResult): the
// knees' fold-plane twist, their inward (valgus) offset, their tremble, and the planted feet's
// orientation — the measurements behind the "knees twist inward / feet mangled" complaints.
// Limits for the leg diagnostics a phase GATES (negative = an info row only). The trunk phases
// gate all three of twist / inward / foot rotation: before the knee-and-feet round every
// crouch, walk and chest lean ran the thigh twist to its 75° limit, knocked the knees 10-20cm
// inward and rotated the planted feet 45°, and the position-only gates never saw it.
struct LegGates {
    double twistDeg = -1.0;   // knee twist max
    double inwardMm = -1.0;   // knee inward max
    double footRotDeg = -1.0; // planted foot rotation at the end
    double trembleMm = -1.0;  // knee tremble
};

std::vector<Gate> legInfo(const RunResult& r, const LegGates& g) {
    const auto row = [](const char* name, double value, double limit) {
        return limit >= 0.0 ? gateMax(name, value, limit) : info(name, value);
    };
    return {row("knee twist max (deg)", r.kneeTwistMaxDeg, g.twistDeg),
            info("knee twist at end (deg)", r.kneeTwistEndDeg),
            row("knee inward max (mm)", r.kneeInwardMax * 1000.0, g.inwardMm),
            info("knee inward at end (mm)", r.kneeInwardEnd * 1000.0),
            row("knee tremble (mm)", r.kneeOscMax * 1000.0, g.trembleMm),
            info("knee worst step (mm)", r.kneeStepMax * 1000.0),
            info("planted foot rotation max (deg)", r.footRotMaxDeg),
            row("planted foot rotation at end (deg)", r.footRotEndDeg, g.footRotDeg)};
}

// The trunk-phase limits: crouches (feet planted under the hips) and leans/walks (a lean rolls
// the knee a few centimetres; a walk's landing foot re-flattens within a tick or two).
constexpr LegGates kCrouchLegs{10.0, 30.0, 8.0, -1.0};
constexpr LegGates kLeanLegs{12.0, 40.0, 8.0, -1.0}; // 10.2° on the previous generation's pinned lean
constexpr LegGates kHandLegs{5.0, 20.0, 5.0, -1.0}; // a hand drag never moves the legs

} // namespace

void realPhases(Report& report, const std::vector<ArmatureBone>& bones) {
    const bool v = report.verbose;
    // REST-POSE normalization: the older generations rest in a T-POSE (arms horizontal, the
    // hand above the shoulder), and every scenario here is a gesture from the base rig's A-pose
    // (arms hanging) — "+60cm up" from a horizontal arm is beyond any reach, and "+25cm out"
    // from one is a full-body strain. Such a rig gets its arms lowered by a shoulder pre-pose
    // before every scenario, so the phases measure the same gestures on every generation.
    std::vector<std::pair<std::string, glm::vec3>> restPose;
    {
        Armature probe;
        probe.build(bones);
        const int hand = resolveBone(probe, "lHand");
        const int collar = resolveBone(probe, "lCollar");
        if (hand >= 0 && collar >= 0 &&
            probe.boneWorldPosition(static_cast<std::size_t>(hand)).y >
                probe.boneWorldPosition(static_cast<std::size_t>(collar)).y - 0.10f) {
            // The shoulder's third channel lowers the arm on these rigs (its authored range
            // reaches -75/-85 on the left); the right mirrors as (x, -y, -z).
            restPose = {{"lShldr", glm::vec3(0.0f, 0.0f, -65.0f)},
                        {"rShldr", glm::vec3(0.0f, 0.0f, 65.0f)}};
            std::printf("[info] T-pose rest: arms lowered by a shoulder pre-pose in every phase\n");
        }
    }
    auto run = [&](Scenario sc) {
        sc.prePose.insert(sc.prePose.begin(), restPose.begin(), restPose.end());
        return runScenario(bones, sc, v);
    };
    auto runIdle = [&](Scenario sc) {
        sc.prePose.insert(sc.prePose.begin(), restPose.begin(), restPose.end());
        return runIdleMetrics(bones, sc);
    };
    // A phase report with the leg diagnostics appended (info rows: verbose only).
    auto phaseLegs = [&](const std::string& name, std::vector<Gate> gates, const RunResult& r,
                         const LegGates& lg = LegGates{}) {
        for (const Gate& g : legInfo(r, lg)) {
            gates.push_back(g);
        }
        report.phase(name, gates);
    };

    // --- The in-app benchmark path (POSESTUDIO_IK_BENCH=lHand): the same waypoints, so the
    // per-phase lag here must match the app's [ikperf] report tick for tick.
    if (report.wants("bench")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.path = {{0, glm::vec3(0.0f), "start"},
                   {60, glm::vec3(0.25f, 0.15f, 0.05f), "move-med"},
                   {120, glm::vec3(0.25f, 0.15f, 0.05f), "hold-1"},
                   {180, glm::vec3(0.0f), "return"},
                   {240, glm::vec3(0.0f), "hold-2"},
                   {270, glm::vec3(-0.30f, 0.20f, 0.0f), "move-fast"},
                   {330, glm::vec3(-0.30f, 0.20f, 0.0f), "hold-3"}};
        sc.idleJoints = kIdleForHandDrag;
        const RunResult r = run(sc);
        const IdleMetrics idle = runIdle(sc);
        std::vector<Gate> gates;
        if (!r.ok) {
            gates.push_back(gateMin(("error: " + r.error).c_str(), 0.0, 1.0));
        } else {
            gates = {gateMax("move-med lag mean (mm)", r.phases[0].lagMean() * 1000.0, 5.0),
                     gateMax("hold-1 resting lag (mm)", r.phases[1].lagMax * 1000.0, 5.0),
                     gateMax("return lag mean (mm)", r.phases[2].lagMean() * 1000.0, 8.0),
                     gateMax("hold-2 return-to-rest miss (mm)", r.phases[3].lagMax * 1000.0, 3.0),
                     gateMax("move-fast lag mean (mm)", r.phases[4].lagMean() * 1000.0, 30.0),
                     gateMax("hold-3 flick resting miss (mm)", r.phases[5].lagMax * 1000.0, 5.0),
                     // 70: an elbow re-orienting its fold plane at the extraction's 12°/tick
                     // twist cap swings ~6cm in a tick on the oldest rigs (no twist bones).
                     gateMax("worst single-tick jump (mm)", std::max({r.phases[0].maxJump, r.phases[2].maxJump, r.phases[4].maxJump}) * 1000.0, 70.0),
                     gateMax("grabbed-joint accel excess (mm/tick^2)", std::max(0.0, r.phases[4].effSpeedUpMax - r.phases[4].cursorSpeedUpMax) * 1000.0, 12.0),
                     gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0),
                     gateMax("idle-joint tremble (mm)", idle.oscMax * 1000.0, 35.0),
                     gateMax("settle ticks", r.settleTicks, 45.0),
                     gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0)};
            for (const PhaseStats& ph : r.phases) {
                gates.push_back(info((ph.label + " lag mean (mm)").c_str(), ph.lagMean() * 1000.0));
            }
        }
        phaseLegs("[real] bench path: lHand (the app's POSESTUDIO_IK_BENCH)", gates, r, kHandLegs);
    }

    // --- A single gentle side pull holds the feet and the hip; the hand lands near the cursor.
    if (report.wants("gentle")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.path = pullPath(glm::vec3(0.10f, 0.0f, 0.0f), 30, 60);
        sc.idleJoints = kIdleForHandDrag;
        const RunResult r = run(sc);
        phaseLegs("[real] gentle side pull (lHand +10cm x)",
                     {gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0),
                      gateMax("hip displacement (mm)", r.hipDisp * 1000.0, 10.0),
                      gateMax("hand-to-cursor at rest (mm)", r.restingMiss * 1000.0, 5.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0),
                      info("steps", r.stepsTaken)}, r, kHandLegs);
    }

    // --- Wheel DEPTH steps: the app moves the drag plane in discrete notches while the cursor
    // rests (a ~5cm target jump per notch at the default framing — VulkanWindow::wheelEvent), so
    // the raw target arrives as a staircase. Each step must read as a smooth push (the accel
    // shaping ramps the hand over a few ticks), the hand must land on the moved cursor, and the
    // feet must stay planted — a forward hand pull recruits the arm and trunk, not the legs.
    if (report.wants("depth")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.path.push_back({0, glm::vec3(0.0f), "start"});
        int t = 0;
        for (int k = 1; k <= 6; ++k) {
            const glm::vec3 off(0.0f, 0.0f, 0.05f * static_cast<float>(k)); // toward a front camera
            sc.path.push_back({t + 1, off, "step"});
            sc.path.push_back({t + 11, off, "hold"});
            t += 11;
        }
        sc.path.push_back({t + 60, sc.path.back().offset, "rest"});
        sc.idleJoints = kIdleForHandDrag;
        const RunResult r = run(sc);
        const IdleMetrics idle = runIdle(sc);
        double stepJumpMax = 0.0;
        double stepSpeedUpMax = 0.0;
        for (const PhaseStats& ph : r.phases) {
            stepJumpMax = std::max(stepJumpMax, ph.maxJump);
            stepSpeedUpMax = std::max(stepSpeedUpMax, ph.effSpeedUpMax);
        }
        phaseLegs("[real] wheel depth steps (lHand 6 x 5cm z notches)",
                     {gateMax("hand-to-cursor at rest (mm)", r.restingMiss * 1000.0, 5.0),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0),
                      gateMax("hip displacement (mm)", r.hipDisp * 1000.0, 40.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0),
                      gateMax("idle-joint tremble (mm)", idle.oscMax * 1000.0, 35.0),
                      info("worst single-tick joint jump (mm)", stepJumpMax * 1000.0),
                      info("worst hand speed-up per tick (mm)", stepSpeedUpMax * 1000.0),
                      info("steps", r.stepsTaken)}, r);
    }

    // --- Pose utilities (Armature::mirrorPose / mirrorSubtreeToOpposite / resetBone /
    // resetPose). The mirror rule — (x, -y, -z) per Euler channel, a negated x translation — is
    // checked GEOMETRICALLY: after mirroring, the right hand and foot must sit exactly where the
    // reflection (x -> -x) of the left ones was, which only holds if the rule matches the rig's
    // real left/right orientation frames. Mirroring twice must be the identity, a limb copy must
    // be one-way, and the resets must return exactly to rest without touching pins.
    if (report.wants("utilities")) {
        Armature arm;
        arm.build(bones);
        const int lShin = resolveBone(arm, "lShin"), rShin = resolveBone(arm, "rShin");
        const int lFoot = resolveBone(arm, "lFoot"), rFoot = resolveBone(arm, "rFoot");
        const int lForeArm = resolveBone(arm, "lForeArm"), rForeArm = resolveBone(arm, "rForeArm");
        const int lHand = resolveBone(arm, "lHand"), rHand = resolveBone(arm, "rHand");
        const int abdomen = resolveBone(arm, "abdomenLower"), hip = resolveBone(arm, "hip");
        const int head = resolveBone(arm, "head");
        const bool named = lShin >= 0 && rShin >= 0 && lFoot >= 0 && rFoot >= 0 && lForeArm >= 0 &&
                           rForeArm >= 0 && lHand >= 0 && rHand >= 0 && abdomen >= 0 && hip >= 0 &&
                           head >= 0;
        double pairing = 0.0, geomErr = 1e9, ruleErr = 1e9, involutionErr = 1e9, copyErr = 1e9;
        double resetJointErr = 1e9, resetLimbErr = 1e9, resetPoseErr = 1e9;
        double pinKept = 0.0;
        auto err = [](const glm::vec3& a, const glm::vec3& b) {
            return static_cast<double>(glm::length(a - b));
        };
        if (named) {
            pairing = (arm.mirrorBone(static_cast<std::size_t>(lShin)) == rShin &&
                       arm.mirrorBone(static_cast<std::size_t>(rShin)) == lShin &&
                       arm.mirrorBone(static_cast<std::size_t>(lHand)) == rHand &&
                       arm.mirrorBone(static_cast<std::size_t>(hip)) == hip &&
                       arm.mirrorBone(static_cast<std::size_t>(head)) == head)
                          ? 1.0
                          : 0.0;
            // A one-sided pose well inside every limit on both sides (see the base rig's ranges).
            const std::vector<std::pair<std::string, glm::vec3>> pose = {
                {resolveBoneName(arm, "lShin"), glm::vec3(120.0f, -8.0f, 3.0f)},
                {resolveBoneName(arm, "lForeArm"), glm::vec3(0.0f, -100.0f, 0.0f)},
                {resolveBoneName(arm, "lHand"), glm::vec3(5.0f, -20.0f, 30.0f)},
                {resolveBoneName(arm, "abdomenLower"), glm::vec3(10.0f, 5.0f, -5.0f)},
                {resolveBoneName(arm, "head"), glm::vec3(-10.0f, 10.0f, 5.0f)},
                {"@trans:" + resolveBoneName(arm, "hip"), glm::vec3(0.05f, -0.10f, 0.02f)},
            };
            arm.applyPose(pose);
            const glm::vec3 lFootBefore = arm.boneWorldPosition(static_cast<std::size_t>(lFoot));
            const glm::vec3 lHandBefore = arm.boneWorldPosition(static_cast<std::size_t>(lHand));
            std::vector<glm::vec3> eulerBefore, transBefore;
            for (std::size_t i = 0; i < arm.boneCount(); ++i) {
                eulerBefore.push_back(arm.boneEuler(i));
                transBefore.push_back(arm.boneTranslation(i));
            }

            arm.mirrorPose();
            const glm::vec3 reflFoot(-lFootBefore.x, lFootBefore.y, lFootBefore.z);
            const glm::vec3 reflHand(-lHandBefore.x, lHandBefore.y, lHandBefore.z);
            geomErr = std::max(err(arm.boneWorldPosition(static_cast<std::size_t>(rFoot)), reflFoot),
                               err(arm.boneWorldPosition(static_cast<std::size_t>(rHand)), reflHand));
            ruleErr = 0.0;
            ruleErr = std::max(ruleErr, err(arm.boneEuler(static_cast<std::size_t>(rShin)), glm::vec3(120.0f, 8.0f, -3.0f)));
            ruleErr = std::max(ruleErr, err(arm.boneEuler(static_cast<std::size_t>(rForeArm)), glm::vec3(0.0f, 100.0f, 0.0f)));
            ruleErr = std::max(ruleErr, err(arm.boneEuler(static_cast<std::size_t>(rHand)), glm::vec3(5.0f, 20.0f, -30.0f)));
            ruleErr = std::max(ruleErr, err(arm.boneEuler(static_cast<std::size_t>(abdomen)), glm::vec3(10.0f, -5.0f, 5.0f)));
            ruleErr = std::max(ruleErr, err(arm.boneEuler(static_cast<std::size_t>(head)), glm::vec3(-10.0f, -10.0f, -5.0f)));
            ruleErr = std::max(ruleErr, err(arm.boneEuler(static_cast<std::size_t>(lShin)), glm::vec3(0.0f)));
            ruleErr = std::max(ruleErr, err(arm.boneTranslation(static_cast<std::size_t>(hip)), glm::vec3(-0.05f, -0.10f, 0.02f)));

            arm.mirrorPose(); // an involution: back to the original pose exactly
            involutionErr = 0.0;
            for (std::size_t i = 0; i < arm.boneCount(); ++i) {
                involutionErr = std::max(involutionErr, err(arm.boneEuler(i), eulerBefore[i]));
                involutionErr = std::max(involutionErr, err(arm.boneTranslation(i), transBefore[i]));
            }

            // One-way limb copy: the right arm takes the left's pose; the left keeps its own.
            arm.mirrorSubtreeToOpposite(lForeArm);
            copyErr = 0.0;
            copyErr = std::max(copyErr, err(arm.boneEuler(static_cast<std::size_t>(rForeArm)), glm::vec3(0.0f, 100.0f, 0.0f)));
            copyErr = std::max(copyErr, err(arm.boneEuler(static_cast<std::size_t>(rHand)), glm::vec3(5.0f, 20.0f, -30.0f)));
            copyErr = std::max(copyErr, err(arm.boneEuler(static_cast<std::size_t>(lForeArm)), glm::vec3(0.0f, -100.0f, 0.0f)));
            copyErr = std::max(copyErr, err(arm.boneEuler(static_cast<std::size_t>(lHand)), glm::vec3(5.0f, -20.0f, 30.0f)));
            copyErr = std::max(copyErr, err(arm.boneEuler(static_cast<std::size_t>(rShin)), glm::vec3(0.0f))); // outside the subtree: untouched

            // Reset one joint: the hand only, its parent keeps its bend.
            arm.resetBone(lHand, false);
            resetJointErr = std::max(err(arm.boneEuler(static_cast<std::size_t>(lHand)), glm::vec3(0.0f)),
                                     err(arm.boneEuler(static_cast<std::size_t>(lForeArm)), glm::vec3(0.0f, -100.0f, 0.0f)));
            // Reset a limb: the forearm and everything below it; the other arm is untouched.
            arm.resetBone(lForeArm, true);
            resetLimbErr = std::max(err(arm.boneEuler(static_cast<std::size_t>(lForeArm)), glm::vec3(0.0f)),
                                    err(arm.boneEuler(static_cast<std::size_t>(rForeArm)), glm::vec3(0.0f, 100.0f, 0.0f)));
            // Reset the pose: everything to rest, a pin survives.
            arm.setSelectedBone(rHand);
            arm.togglePinSelectedBone();
            arm.resetPose();
            resetPoseErr = 0.0;
            for (std::size_t i = 0; i < arm.boneCount(); ++i) {
                resetPoseErr = std::max(resetPoseErr, err(arm.boneEuler(i), glm::vec3(0.0f)));
                resetPoseErr = std::max(resetPoseErr, err(arm.boneTranslation(i), glm::vec3(0.0f)));
            }
            pinKept = arm.isBonePinned(static_cast<std::size_t>(rHand)) ? 1.0 : 0.0;
        }
        report.phase("[real] pose utilities: mirror + reset",
                     {gateMin("bone pairing (l<->r, centre = self)", pairing, 1.0),
                      gateMax("mirrored hand/foot vs reflected original (mm)", geomErr * 1000.0, 1.0),
                      gateMax("mirror rule error (deg / m)", ruleErr, 1e-3),
                      gateMax("mirror twice = identity (max error)", involutionErr, 1e-3),
                      gateMax("limb copy error (deg)", copyErr, 1e-3),
                      gateMax("reset joint error (deg)", resetJointErr, 1e-3),
                      gateMax("reset limb error (deg)", resetLimbErr, 1e-3),
                      gateMax("reset pose error", resetPoseErr, 1e-3),
                      gateMin("pin survives reset pose", pinKept, 1.0)});
    }

    // --- A reachable overhead pull RAISES THE ARM with the head and chest still, the feet
    // planted, no suspension (the "pulling straight up bends her over" repro).
    if (report.wants("arm-raise")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.path = pullPath(glm::vec3(0.0f, 0.60f, 0.0f), 60, 90);
        sc.idleJoints = kIdleForHandDrag;
        const RunResult r = run(sc);
        phaseLegs("[real] arm raise (lHand +60cm y, reachable)",
                     {gateMin("hand rise (m)", r.grabEnd.y - r.grabStart.y, 0.59),
                      gateMax("head displacement (mm)", r.headDisp * 1000.0, 20.0), // 16.5 on the newest rig
                      gateMax("chest displacement (mm)", r.chestDisp * 1000.0, 15.0),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0),
                      gateMax("suspended", r.suspended ? 1.0 : 0.0, 0.0),
                      info("hand-to-cursor at rest (mm)", r.restingMiss * 1000.0)}, r);
    }

    // --- A sustained pull far beyond reach LIFTS the figure: the feet leave the floor and the
    // body dangles below the grab point.
    if (report.wants("suspension")) {
        Scenario sc;
        sc.grab = "lHand";
        // +1.35m: the lift gate is GEOMETRIC (the target farther from the root than the
        // hand-to-root chain plus 15cm — arm and spine, ~1.1m), and +1.20 cleared it by a
        // hair on the base rig only; a rig whose hand rests 5cm lower never lifted at all.
        sc.path = pullPath(glm::vec3(0.0f, 1.35f, 0.0f), 60, 120);
        sc.release = false;
        const RunResult r = run(sc);
        phaseLegs("[real] suspension (lHand +135cm y, beyond reach)",
                     {gateMin("suspended", r.suspended ? 1.0 : 0.0, 1.0),
                      gateMin("feet height at the end (m)", r.feetMinY, 0.20),
                      gateMin("hip rise (m)", r.hipDispY, 0.10)}, r);
    }

    // --- Pushing the chest down folds a crouch over the planted feet; no foot buries itself.
    if (report.wants("crouch-push")) {
        Scenario sc;
        sc.grab = "chest_2";
        sc.path = pullPath(glm::vec3(0.0f, -0.25f, 0.0f), 60, 90);
        const RunResult r = run(sc);
        phaseLegs("[real] chest push-down (chest_2 -25cm y)",
                     {gateMin("hip drop at release (m)", r.hipDropAtRelease, 0.05),
                      info("hip drop after settle (m)", -r.hipDispY),
                      gateMax("lowest contact vs floor, drag (mm)", (r.contactBindMinY - r.contactMinY) * 1000.0, 10.0),
                      gateMax("floor penetration, any joint (mm)", r.penetrationMax * 1000.0, 30.0),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 80.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0)}, r, kCrouchLegs);
    }

    // --- Dragging the hip down is an explicit crouch: deep, feet planted.
    if (report.wants("hip-crouch")) {
        Scenario sc;
        sc.grab = "hip";
        sc.path = pullPath(glm::vec3(0.0f, -0.12f, 0.0f), 40, 80);
        const RunResult r = run(sc);
        phaseLegs("[real] hip crouch (hip -12cm y)",
                     {gateMin("hip drop at release (m)", r.hipDropAtRelease, 0.10),
                      info("hip drop after settle (m)", -r.hipDispY),
                      gateMax("lowest contact vs floor, drag (mm)", (r.contactBindMinY - r.contactMinY) * 1000.0, 5.0),
                      // The toe joints ride the sole's transient pitch ~2cm below their rest
                      // height (a centimetre through the floor) — a known soft spot, guarded.
                      gateMax("floor penetration, any joint (mm)", r.penetrationMax * 1000.0, 30.0),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 30.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0)}, r, kCrouchLegs);
    }

    // --- LIVE CONTACTS (IkRig::updateContacts): joints that reach the floor MID-DRAG become
    // supports. Before this, only the drag-start contacts were ever pinned: a hand or knee
    // coming down rode the trunk as an inactive subtree straight through the floor (8cm on a
    // deep crouch), and a hand left on the floor by one drag never let go in the next.
    // (1) An ALL-FOURS descent: the figure bent 90° at the hips (the hip bone pitched forward,
    // the thighs counter-rotated so the legs stay vertical, a little spine flexion) with the
    // arms hanging; the hip is dragged down until the fingertips touch and 3cm past. The
    // hands must plant where they touched and stay (the joint-space refinement folds
    // the arms; the solver's own arm chain cannot, see kLiveRefineStep), never sinking below
    // their planting height. The descent is sized per rig from the pre-posed hand height, and
    // skipped where the pose does not bring the hands within reach of the floor.
    // 45° at the hips (not 60°): the oldest generation's thigh range ends at -100°, and a
    // 60° pre-pose left it no room for the crouch's own flexion — its feet strained 5cm
    // before the hands even landed.
    const std::vector<std::pair<std::string, glm::vec3>> kAllFoursPose{
        {"hip", glm::vec3(45.0f, 0.0f, 0.0f)},          {"lThigh", glm::vec3(-45.0f, 0.0f, 0.0f)},
        {"rThigh", glm::vec3(-45.0f, 0.0f, 0.0f)},      {"abdomenLower", glm::vec3(20.0f, 0.0f, 0.0f)},
        {"chest", glm::vec3(20.0f, 0.0f, 0.0f)},        {"lShldr", glm::vec3(0.0f, -90.0f, -20.0f)},
        {"rShldr", glm::vec3(0.0f, 90.0f, 20.0f)}};
    float allFoursDrop = 0.0f; // hip descent that plants the hands, 0 = not on this rig
    float kneelDrop = 0.0f;    // hip descent that brings the knees to the floor
    {
        Armature probe;
        probe.build(bones);
        const int hip = resolveBone(probe, "hip");
        if (hip >= 0) {
            kneelDrop = probe.boneWorldPosition(static_cast<std::size_t>(hip)).y - 0.43f;
        }
        bool posed = true;
        for (const auto& [bone, euler] : restPose) {
            posed = posed && probe.setBoneRotation(resolveBoneName(probe, bone), euler);
        }
        for (const auto& [bone, euler] : kAllFoursPose) {
            posed = posed && probe.setBoneRotation(resolveBoneName(probe, bone), euler);
        }
        const int hand = resolveBone(probe, "lHand");
        if (posed && hand >= 0) {
            // The hand plants when its LOWEST part (a fingertip) reaches its 1.5cm clearance;
            // the hip then goes 3cm further, which the arms absorb by folding. (Sizing the
            // descent from the hand JOINT instead drove the oldest rigs' straight arms 10cm
            // past their plant — a fold no joint-space fit can start from a perfectly straight
            // arm, so their hands ended out of reach and rose 5cm on release.)
            float lowest = probe.boneWorldPosition(static_cast<std::size_t>(hand)).y;
            for (std::size_t i = 0; i < probe.boneCount(); ++i) {
                for (int cur = probe.boneParent(i); cur >= 0;
                     cur = probe.boneParent(static_cast<std::size_t>(cur))) {
                    if (cur == hand) {
                        lowest = std::min(lowest, probe.boneWorldPosition(i).y);
                        break;
                    }
                }
            }
            const float drop = lowest - 0.015f + 0.03f;
            if (drop > 0.15f && drop < 0.75f) {
                allFoursDrop = drop;
            }
        }
    }
    if (report.wants("hands-plant")) {
        if (allFoursDrop <= 0.0f) {
            report.skip("[real] all fours: hands plant",
                        "the all-fours pre-pose does not bring the hands to the floor on this rig");
        } else {
            Scenario sc;
            sc.grab = "hip";
            sc.path = pullPath(glm::vec3(0.0f, -allFoursDrop, 0.0f), 80, 60);
            sc.prePose = kAllFoursPose;
            const RunResult r = run(sc);
            char name[128];
            std::snprintf(name, sizeof(name), "[real] all fours: hands plant (hip -%.0fcm y, bent at the hips)",
                          allFoursDrop * 100.0f);
            phaseLegs(name,
                         {gateMin("live contacts planted (max at once)", r.livePinsMax, 2.0),
                          gateMin("pins at mouse-up (feet + hands)", r.pinsAtRelease, 4.0),
                          gateMin("lower hand height min (m)", r.handsMinY, 0.07),
                          info("lower hand height at the end (m)", r.handsEndY),
                          // 15: seven generations hold within 1.5mm; the oldest (shoulder
                          // flexion ends at -75°, the pre-pose's -90° clamps) rises 12.6mm.
                          gateMax("hand rise off its plant at the end (mm)", (r.handsEndY - r.handsMinY) * 1000.0, 15.0),
                          gateMax("live pin slide (mm)", r.livePinSlideMax * 1000.0, 100.0),
                          gateMin("hip drop at release (m)", r.hipDropAtRelease, allFoursDrop - 0.03),
                          info("floor penetration, any joint (mm)", r.penetrationMax * 1000.0),
                          gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 21.0)}, r);
        }
    }
    // (2) ... and standing back up out of it: the planted hands must LET GO again (a live
    // contact is a unilateral support — the arms go taut, then lift off), and the figure rises
    // with its feet still planted, the hip on the cursor.
    if (report.wants("hands-rise")) {
        if (allFoursDrop <= 0.0f) {
            report.skip("[real] all fours, then rise",
                        "the all-fours pre-pose does not bring the hands to the floor on this rig");
        } else {
            Scenario sc;
            sc.grab = "hip";
            const glm::vec3 down(0.0f, -allFoursDrop, 0.0f);
            const glm::vec3 up(0.0f, -std::max(allFoursDrop - 0.35f, 0.05f), 0.0f);
            sc.path = {{0, glm::vec3(0.0f), "start"}, {80, down, "down"}, {110, down, "hold"},
                       {190, up, "up"},              {250, up, "hold-up"}};
            sc.prePose = kAllFoursPose;
            const RunResult r = run(sc);
            phaseLegs("[real] all fours, then rise (hip back up 35cm)",
                         {gateMin("live contacts planted (max at once)", r.livePinsMax, 2.0),
                          gateMax("pins at mouse-up (hands released)", r.pinsAtRelease, 2.0),
                          gateMin("lower hand height at the end (m)", r.handsEndY, 0.25),
                          gateMin("lower hand height min (m)", r.handsMinY, 0.07),
                          gateMax("hip-to-cursor at rest (mm)", r.restingMiss * 1000.0, 20.0),
                          info("floor penetration, any joint (mm)", r.penetrationMax * 1000.0),
                          gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 21.0)}, r);
        }
    }
    // (3) A KNEEL: the hip dragged down and forward until the knees reach the floor. Both
    // knees must plant as supports (with the feet still pinned: the solver restores each foot
    // through its knee pin) and end on the floor. The feet slide back as a real kneel's do (the
    // shin can only lie flat with the ankle a shin's length behind the knee), and the toes of
    // the pinned feet still dip through the floor — the ankle pin holds the standing height,
    // and the foot's roll onto its top is the kneeling round's job; info rows here.
    if (report.wants("kneel")) {
        Scenario sc;
        sc.grab = "hip";
        sc.path = pullPath(glm::vec3(0.0f, -kneelDrop, 0.20f), 90, 60);
        const RunResult r = run(sc);
        char name[128];
        std::snprintf(name, sizeof(name), "[real] kneel: knees plant (hip -%.0fcm y +20cm z)",
                      kneelDrop * 100.0f);
        phaseLegs(name,
                     {gateMin("live contacts planted (max at once)", r.livePinsMax, 2.0),
                      gateMin("pins at mouse-up (feet + knees)", r.pinsAtRelease, 4.0),
                      gateMax("lower knee height at the end (m)", r.kneesEndY, 0.03),
                      gateMax("live pin slide (mm)", r.livePinSlideMax * 1000.0, 60.0),
                      gateMin("hip drop at release (m)", r.hipDropAtRelease, kneelDrop - 0.03),
                      info("feet drift (mm)", r.contactDriftMax * 1000.0),
                      info("floor penetration, any joint (mm)", r.penetrationMax * 1000.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 21.0)}, r);
    }

    // --- ORIENTATION during a drag: the X/Y/Z wheel hold works mid-drag, rotating the
    // grabbed joint about its own channel while the IK keeps placing it. The rotation must
    // STICK (the drag-tick rotational prior decays every unfitted channel toward the drag start
    // at 5%/tick — over 90 ticks a 40° turn would be gone — so a wheel-nudged bone leaves the
    // prior for the rest of the drag), and the drag itself must be undisturbed: the joint stays
    // on the cursor, the feet stay planted. A hand (a limb effector) and the head (a trunk
    // effector whose neck aims at it).
    if (report.wants("wheel-hand")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.path = pullPath(glm::vec3(0.0f, 0.0f, 0.25f), 60, 90);
        sc.nudges = {{30, glm::vec3(0.0f, 0.0f, 40.0f)}};
        sc.idleJoints = kIdleForHandDrag;
        const RunResult r = run(sc);
        const IdleMetrics idle = runIdle(sc);
        const float turned = r.grabEulerEnd.z - r.grabEulerStart.z;
        phaseLegs("[real] wheel rotate mid-drag (lHand +25cm z, hand z +40 deg at tick 30)",
                     {gateMax("hand rotation kept vs the wheel (deg off)", std::abs(turned - 40.0f), 3.0),
                      info("hand z channel turned (deg)", turned),
                      gateMax("grab-to-cursor at rest (mm)", r.restingMiss * 1000.0, 5.0),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0),
                      gateMax("idle-joint tremble (mm)", idle.oscMax * 1000.0, 35.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 21.0)}, r, kHandLegs);
    }
    if (report.wants("wheel-head")) {
        Scenario sc;
        sc.grab = "head";
        sc.path = pullPath(glm::vec3(0.10f, 0.0f, 0.0f), 60, 90);
        sc.nudges = {{30, glm::vec3(0.0f, 20.0f, 0.0f)}};
        sc.idleJoints = kIdleForHeadDrag;
        const RunResult r = run(sc);
        const IdleMetrics idle = runIdle(sc);
        const float turned = r.grabEulerEnd.y - r.grabEulerStart.y;
        phaseLegs("[real] wheel rotate mid-drag (head +10cm x, head y +20 deg at tick 30)",
                     {gateMax("head rotation kept vs the wheel (deg off)", std::abs(turned - 20.0f), 3.0),
                      info("head y channel turned (deg)", turned),
                      gateMax("grab-to-cursor at rest (mm)", r.restingMiss * 1000.0, 5.0),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0),
                      gateMax("idle-joint tremble (mm)", idle.oscMax * 1000.0, 35.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 21.0)}, r);
    }

    // --- A hovering figure is healed back onto the floor by its next drag.
    if (report.wants("hover-heal")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.hover = 0.03f;
        sc.path = pullPath(glm::vec3(0.08f, 0.0f, 0.0f), 30, 90);
        const RunResult r = run(sc);
        double feetY = 1e9;
        for (const int c : r.contactPins) {
            feetY = std::min(feetY, static_cast<double>(r.endPos[static_cast<std::size_t>(c)].y));
        }
        phaseLegs("[real] hover heal (3cm hover, gentle hand pull)",
                     {gateMax("lowest contact vs floor after (mm)", std::abs(r.contactBindMinY < 1e8 ? feetY - r.contactBindMinY : 0.0) * 1000.0, 5.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0)}, r);
    }

    // --- A strained pull (beyond the arm's reach, forward) and its release: the pins land
    // exactly, every settle tick small.
    if (report.wants("strained-release")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.path = pullPath(glm::vec3(0.0f, 0.10f, 0.55f), 60, 60);
        const RunResult r = run(sc);
        phaseLegs("[real] strained pull + release (lHand +55cm z)",
                     {gateMax("worst pin after settle (mm)", r.settleWorstPinErr * 1000.0, 2.0),
                      gateMax("feet drift after settle (mm)", r.contactDriftMax * 1000.0, 20.0),
                      gateMax("steps taken", r.stepsTaken, 0.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0),
                      gateMax("settle ticks", r.settleTicks, 45.0),
                      info("hand-to-cursor at rest (mm)", r.restingMiss * 1000.0),
                      info("hand moved through settle (mm)", r.holdDrift * 1000.0)}, r);
    }

    // --- Dragging a foot back and up TRACKS the lift and holds after release.
    if (report.wants("foot-lift")) {
        Scenario sc;
        sc.grab = "lFoot";
        sc.path = pullPath(glm::vec3(0.0f, 0.15f, -0.10f), 40, 80);
        sc.idleJoints = {"head", "lHand", "rHand", "rFoot"};
        const RunResult r = run(sc);
        phaseLegs("[real] foot lift (lFoot +15cm y, -10cm z)",
                     {gateMin("foot lift reached (m)", r.grabEnd.y - r.grabStart.y, 0.13),
                      gateMax("foot-to-cursor at rest (mm)", r.restingMiss * 1000.0, 15.0),
                      gateMax("foot moved through settle (mm)", r.holdDrift * 1000.0, 10.0),
                      gateMax("standing foot drift (mm)", r.contactDriftMax * 1000.0, 15.0)},
                  // 100: the lifted leg's knee must not shimmer (151 before the dragged-limb
                  // prior exemption); 36 on the base rig, 60-80 on the male and oldest rigs.
                  r, LegGates{-1.0, -1.0, -1.0, 100.0});
    }

    // --- A lateral HIP drag walks the figure: steps land at floor height, the hip tracks.
    for (const float dist : {0.35f, 0.60f}) {
        char name[96];
        std::snprintf(name, sizeof(name), "[real] hip walk (%.0fcm lateral)", dist * 100.0f);
        if (!report.wants("walk")) {
            break;
        }
        Scenario sc;
        sc.grab = "hip";
        const int moveTicks = static_cast<int>(dist / 0.004f); // ~0.24 m/s
        sc.path = pullPath(glm::vec3(dist, 0.0f, 0.0f), moveTicks, 150);
        const RunResult r = run(sc);
        double feetOffFloor = 0.0;
        for (const int c : r.contactPins) {
            feetOffFloor = std::max(feetOffFloor, std::abs(static_cast<double>(r.endPos[static_cast<std::size_t>(c)].y) - r.contactBindMinY));
        }
        // The stance should end centered under the hip target (the feet land at the target
        // plus their stance offset): both feet moved by roughly the drag distance.
        double feetMoved = 1e9;
        for (const int c : r.contactPins) {
            feetMoved = std::min(feetMoved, static_cast<double>(glm::length(r.endPos[static_cast<std::size_t>(c)] - r.startPos[static_cast<std::size_t>(c)])));
        }
        phaseLegs(name, {gateMin("steps taken", r.stepsTaken, dist > 0.5f ? 5.0 : 3.0),
                            gateMax("hip-to-target at rest (mm)", r.restingMiss * 1000.0, 15.0),
                            gateMax("feet off floor at the end (mm)", feetOffFloor * 1000.0, 15.0),
                            gateMin("both feet followed (m)", feetMoved, dist - 0.13),
                            // 25: the settle's own per-tick cap is 20mm and a landing after a
                            // long walk runs right at it (20.3 base, 23.4 on the male rig).
                            gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 25.0)}, r, kLeanLegs);
    }

    // --- A sustained lateral CHEST drag re-plants the feet (balance-driven stepping) and lands
    // on its feet at release.
    if (report.wants("chest-step")) {
        Scenario sc;
        sc.grab = "chest";
        sc.path = pullPath(glm::vec3(0.45f, 0.0f, 0.0f), 110, 150);
        const RunResult r = run(sc);
        double feetOffFloor = 0.0;
        for (const int c : r.contactPins) {
            feetOffFloor = std::max(feetOffFloor, std::abs(static_cast<double>(r.endPos[static_cast<std::size_t>(c)].y) - r.contactBindMinY));
        }
        phaseLegs("[real] stepping chest drag (chest +45cm x)",
                     {gateMin("steps taken", r.stepsTaken, 2.0),
                      gateMax("chest-to-cursor at rest (mm)", r.restingMiss * 1000.0, 150.0),
                      gateMax("feet off floor at the end (mm)", feetOffFloor * 1000.0, 15.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0)}, r, kLeanLegs);
    }

    // --- Pinned joints hold EXACTLY while other things are dragged.
    if (report.wants("pin-hand-drag")) {
        Scenario sc;
        sc.grab = "rHand";
        sc.userPins = {"lHand"};
        sc.path = pullPath(glm::vec3(-0.45f, 0.10f, 0.0f), 80, 80);
        const RunResult r = run(sc);
        phaseLegs("[real] pinned lHand under a 45cm rHand drag",
                     {gateMax("pinned joint max distance (mm)", r.userPinDistMax * 1000.0, 10.0),
                      gateMax("pinned joint rotation (deg)", r.userPinRotMaxDeg, 1.0),
                      gateMax("pinned joint settle step (mm)", r.userPinStepMax * 1000.0, 1.0)}, r);
    }
    if (report.wants("pin-hand-crouch")) {
        Scenario sc;
        sc.grab = "hip";
        sc.userPins = {"lHand"};
        sc.path = pullPath(glm::vec3(0.0f, -0.12f, 0.0f), 40, 100);
        const RunResult r = run(sc);
        phaseLegs("[real] pinned lHand through a 12cm hip crouch",
                     {gateMax("pinned joint max distance (mm)", r.userPinDistMax * 1000.0, 10.0),
                      gateMax("pinned joint rotation (deg)", r.userPinRotMaxDeg, 1.0),
                      // 0.09: the base rig reaches 0.107-0.114, the character dump 0.092-0.098
                      // — both must pass (the character-dump calibration gap, see CLAUDE.md).
                      gateMin("hip drop at release (m)", r.hipDropAtRelease, 0.09)}, r, kCrouchLegs);
    }
    if (report.wants("pin-foot-chest")) {
        Scenario sc;
        sc.grab = "chest_2";
        sc.userPins = {"lFoot"};
        sc.path = pullPath(glm::vec3(0.45f, 0.0f, 0.0f), 110, 150);
        const RunResult r = run(sc);
        phaseLegs("[real] pinned lFoot under a 45cm chest drag",
                     {gateMax("pinned joint max distance (mm)", r.userPinDistMax * 1000.0, 15.0),
                      gateMax("steps taken", r.stepsTaken, 0.0)}, r, kLeanLegs);
    }

    // --- Trembling: slow and medium pulls with cursor noise must not shake idle joints.
    if (report.wants("tremble-slow")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.path = pullPath(glm::vec3(0.06f, 0.0f, 0.0f), 180, 60); // ~2 cm/s
        sc.cursorNoise = 0.002f;
        sc.idleJoints = kIdleForHandDrag;
        const RunResult r = run(sc);
        const IdleMetrics idle = runIdle(sc);
        phaseLegs("[real] tremble: slow noisy pull (lHand 2cm/s, +-2mm noise)",
                     {gateMax("idle-joint tremble (mm)", idle.oscMax * 1000.0, 10.0),
                      gateMax("idle-joint worst step (mm)", idle.stepMax * 1000.0, 10.0),
                      gateMax("grabbed-joint osc (mm)", (r.phases[0].effOsc + r.phases[1].effOsc) * 1000.0, 60.0)}, r);
    }
    if (report.wants("tremble-med")) {
        Scenario sc;
        sc.grab = "head";
        sc.path = pullPath(glm::vec3(0.10f, 0.0f, 0.05f), 60, 60);
        sc.cursorNoise = 0.002f;
        sc.idleJoints = kIdleForHeadDrag;
        const RunResult r = run(sc);
        const IdleMetrics idle = runIdle(sc);
        phaseLegs("[real] tremble: medium noisy head pull (+-2mm noise)",
                     {gateMax("idle-joint tremble (mm)", idle.oscMax * 1000.0, 35.0),
                      gateMax("grabbed-joint osc (mm)", (r.phases[0].effOsc + r.phases[1].effOsc) * 1000.0, 80.0)}, r);
    }

    // --- Grabbing an EYE (a face bone promoted to the head) must not make the head tremble.
    if (report.wants("eye-grab")) {
        Scenario sc;
        sc.grab = "lEye";
        sc.path = pullPath(glm::vec3(0.10f, 0.0f, 0.05f), 60, 90);
        sc.cursorNoise = 0.002f;
        const RunResult r = run(sc);
        phaseLegs("[real] eye grab (lEye promoted to the head, +-2mm noise)",
                     {gateMax("grabbed-joint osc (mm)", (r.phases[0].effOsc + r.phases[1].effOsc) * 1000.0, 80.0),
                      gateMax("head-to-cursor at rest (mm)", r.restingMiss * 1000.0, 60.0)}, r);
    }

    // --- Grabbing a FINGER is an arm gesture: a straight-up pull raises the arm overhead.
    if (report.wants("finger-raise")) {
        Scenario sc;
        sc.grab = "lIndex3";
        sc.path = pullPath(glm::vec3(0.0f, 0.60f, 0.0f), 60, 90);
        const RunResult r = run(sc);
        phaseLegs("[real] finger grab raise (lIndex3 +60cm y)",
                     {gateMin("finger rise (m)", r.grabEnd.y - r.grabStart.y, 0.59),
                      gateMax("head displacement (mm)", r.headDisp * 1000.0, 20.0),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0)}, r);
    }

    // --- Pulling a slouched figure's head up straightens the back.
    if (report.wants("head-straighten")) {
        Scenario sc;
        sc.grab = "head";
        // A forward slouch (positive X on this figure's spine bends forward — the head's z
        // goes from -0.02 to +0.35 in front of the hip); pulling the head up straightens it.
        // (A deeper slouch — 30/20/12 — puts the CoM so far forward that the pull-up
        // triggers a balance step; this one straightens on planted feet.)
        sc.prePose = {{"abdomenLower", glm::vec3(20.0f, 0.0f, 0.0f)},
                      {"chest", glm::vec3(15.0f, 0.0f, 0.0f)},
                      {"neck", glm::vec3(10.0f, 0.0f, 0.0f)}};
        sc.path = pullPath(glm::vec3(0.0f, 0.15f, 0.0f), 40, 90);
        const RunResult r = run(sc);
        phaseLegs("[real] slouched head pull-up (head +15cm y)",
                     {gateMin("head rise (m)", r.grabEnd.y - r.grabStart.y, 0.035),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0),
                      gateMax("steps taken", r.stepsTaken, 0.0)}, r, kLeanLegs);
    }

    // --- A child-scale figure (0.54) crouches and walks with its feet planted.
    if (report.wants("child")) {
        {
            Scenario sc;
            sc.grab = "hip";
            sc.scale = 0.54f;
            sc.path = pullPath(glm::vec3(0.0f, -0.07f, 0.0f), 40, 80);
            const RunResult r = run(sc);
            phaseLegs("[real] child crouch (0.54 scale, hip -7cm)",
                         {gateMin("hip drop (m)", -r.hipDispY, 0.05),
                          gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 20.0)}, r, kCrouchLegs);
        }
        {
            Scenario sc;
            sc.grab = "hip";
            sc.scale = 0.54f;
            sc.path = pullPath(glm::vec3(0.25f, 0.0f, 0.0f), 80, 150);
            const RunResult r = run(sc);
            phaseLegs("[real] child walk (0.54 scale, hip +25cm x)",
                         {gateMin("steps taken", r.stepsTaken, 2.0),
                          gateMax("hip-to-target at rest (mm)", r.restingMiss * 1000.0, 15.0)}, r, kLeanLegs);
        }
    }

    // --- Hold still: a drag that never moves must never move the pose.
    if (report.wants("hold-still")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.path = {{0, glm::vec3(0.0f), "start"}, {120, glm::vec3(0.0f), "hold"}};
        sc.release = false;
        const RunResult r = run(sc);
        double worst = 0.0;
        for (std::size_t i = 0; i < r.endPos.size(); ++i) {
            worst = std::max(worst, static_cast<double>(glm::length(r.endPos[i] - r.startPos[i])));
        }
        phaseLegs("[real] hold still (no cursor motion)",
                     {gateMax("worst joint drift (mm)", worst * 1000.0, 2.0)}, r);
    }
}

} // namespace ikharness
