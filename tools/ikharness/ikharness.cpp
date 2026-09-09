/**
 * @file ikharness.cpp
 * @brief The full-body-IK harness: unit tests for the IK math and constraints, plus a drag-loop
 *        simulator that runs the REAL engine loop (Armature: solve -> extract rotations -> FK ->
 *        next solve, through the same cursor filter and tick order as the viewport) on a figure's
 *        dumped skeleton and gates the measured behaviour.
 *
 * Every FBIK policy in scene/ik/ and scene/armatureik.cpp traces to a user-visible failure that
 * was first reproduced here; a change to either is not done until this passes. Build target
 * PoseStudioIkHarness (Qt-free, Vulkan-free); `ctest` runs it. The real-skeleton phases need a
 * dump of an imported figure — run the app once with POSESTUDIO_DUMP_SKELETON=<path> and pass
 * that file as the first argument or in POSESTUDIO_IK_SKELETON; without one they are skipped
 * (the synthetic unit tests always run). The dump is a vendor's rig and stays out of the repo.
 *
 * Usage: ikharness [skeleton.skel] [--only <phase-substring>] [--verbose]
 */

#include "armature.h"
#include "balancecontroller.h"
#include "cursorfilter.h"
#include "fabriksolver.h"
#include "ikconstraints.h"
#include "ikmath.h"
#include "ikrig.h"
#include "skeletongraph.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace pose;

namespace {

// ------------------------------------------------------------------------------------------
// Reporting: every phase records named metrics; the ones with a limit are gates.
// ------------------------------------------------------------------------------------------

struct Gate {
    std::string name;
    double      value = 0.0;
    double      limit = 0.0;
    bool        lessIsBetter = true;
    bool        gated = true;
};

struct Report {
    int  phases = 0;
    int  failedPhases = 0;
    int  skippedPhases = 0;
    bool verbose = false;
    std::string only;

    bool wants(const std::string& phase) const {
        return only.empty() || phase.find(only) != std::string::npos;
    }

    void phase(const std::string& name, const std::vector<Gate>& gates) {
        ++phases;
        bool ok = true;
        for (const Gate& g : gates) {
            if (g.gated && ((g.lessIsBetter && g.value > g.limit) ||
                            (!g.lessIsBetter && g.value < g.limit))) {
                ok = false;
            }
        }
        if (!ok) {
            ++failedPhases;
        }
        std::printf("%s %s\n", ok ? "[PASS]" : "[FAIL]", name.c_str());
        for (const Gate& g : gates) {
            const bool gateOk = !g.gated || (g.lessIsBetter ? g.value <= g.limit : g.value >= g.limit);
            if (g.gated) {
                std::printf("       %-34s %10.4f  %s %.4f%s\n", g.name.c_str(), g.value,
                            g.lessIsBetter ? "<=" : ">=", g.limit, gateOk ? "" : "   <-- FAIL");
            } else if (verbose) {
                std::printf("       %-34s %10.4f\n", g.name.c_str(), g.value);
            }
        }
    }

    void skip(const std::string& name, const char* why) {
        ++phases;
        ++skippedPhases;
        std::printf("[SKIP] %s (%s)\n", name.c_str(), why);
    }
};

Gate gateMax(const char* name, double value, double limit) { return {name, value, limit, true, true}; }
Gate gateMin(const char* name, double value, double limit) { return {name, value, limit, false, true}; }
Gate info(const char* name, double value) { return {name, value, 0.0, true, false}; }

// ------------------------------------------------------------------------------------------
// Unit tests: the math and constraint primitives (no skeleton needed).
// ------------------------------------------------------------------------------------------

void testEulerRoundTrip(Report& report) {
    const char* orders[] = {"XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX"};
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> angle(-170.0f, 170.0f);
    double worst = 0.0;
    double worstGimbal = 0.0;
    for (const char* order : orders) {
        for (int trial = 0; trial < 200; ++trial) {
            const glm::vec3 e(angle(rng), angle(rng), angle(rng));
            const glm::mat4 m = eulerMatrix(e, order);
            const glm::vec3 back = eulerFromMatrix(glm::mat3(m), order);
            const glm::mat4 m2 = eulerMatrix(back, order);
            double err = 0.0;
            for (int c = 0; c < 3; ++c) {
                for (int r = 0; r < 3; ++r) {
                    err = std::max(err, static_cast<double>(std::abs(m[c][r] - m2[c][r])));
                }
            }
            worst = std::max(worst, err);
        }
        // Gimbal lock: the middle-applied angle at ±90 must still reproduce the matrix.
        for (const float mid : {90.0f, -90.0f}) {
            glm::vec3 e(angle(rng), angle(rng), angle(rng));
            e[order[1] == 'X' ? 0 : order[1] == 'Y' ? 1 : 2] = mid;
            const glm::mat4 m = eulerMatrix(e, order);
            const glm::vec3 back = eulerFromMatrix(glm::mat3(m), order);
            const glm::mat4 m2 = eulerMatrix(back, order);
            double err = 0.0;
            for (int c = 0; c < 3; ++c) {
                for (int r = 0; r < 3; ++r) {
                    err = std::max(err, static_cast<double>(std::abs(m[c][r] - m2[c][r])));
                }
            }
            worstGimbal = std::max(worstGimbal, err);
        }
    }
    report.phase("[unit] euler round-trip (6 orders)",
                 {gateMax("worst matrix error", worst, 1e-4),
                  gateMax("worst gimbal-lock error", worstGimbal, 1e-3)});
}

void testConstraints(Report& report) {
    // A knee: identity orient, the shin hangs along -Y, X bends [-11, 155], Z locked, Y
    // (twist, along the segment) free.
    const glm::mat3 orient(1.0f);
    const glm::vec3 restDir(0.0f, -1.0f, 0.0f);
    const JointConstraint knee = deriveJointConstraint(
        orient, restDir, glm::vec3(-11.0f, 0.0f, 0.0f), glm::vec3(155.0f, 0.0f, 0.0f),
        glm::bvec3(true, false, true));
    const bool isHinge = knee.type == JointConstraint::Type::Hinge;
    const glm::quat frame(1.0f, 0.0f, 0.0f, 0.0f);
    // Bend the shin forward (+Z) past the limit: 170 deg about +X takes -Y toward +Z... rotate
    // and measure the clamped angle back.
    auto bendAbout = [&](const glm::vec3& axis, float deg) {
        return glm::angleAxis(glm::radians(deg), axis) * restDir;
    };
    auto angleFromRest = [&](const glm::vec3& d) {
        return glm::degrees(std::acos(glm::clamp(glm::dot(glm::normalize(d), restDir), -1.0f, 1.0f)));
    };
    const glm::vec3 deep = constrainSegmentDirection(knee, frame, restDir, bendAbout(glm::vec3(1, 0, 0), 170.0f));
    const glm::vec3 hyper = constrainSegmentDirection(knee, frame, restDir, bendAbout(glm::vec3(1, 0, 0), -40.0f));
    const glm::vec3 side = constrainSegmentDirection(knee, frame, restDir, bendAbout(glm::vec3(0, 0, 1), 30.0f));
    // A hip: asymmetric cone, X [-115, 35] (forward kick far, backward little), Z [-20, 85].
    const JointConstraint hip = deriveJointConstraint(
        orient, restDir, glm::vec3(-115.0f, -75.0f, -20.0f), glm::vec3(35.0f, 75.0f, 85.0f),
        glm::bvec3(true, true, true));
    const bool isCone = hip.type == JointConstraint::Type::Cone && hip.perAxis;
    const glm::vec3 kick = constrainSegmentDirection(hip, frame, restDir, bendAbout(glm::vec3(1, 0, 0), -150.0f));
    const glm::vec3 back = constrainSegmentDirection(hip, frame, restDir, bendAbout(glm::vec3(1, 0, 0), 60.0f));
    report.phase("[unit] joint constraints (hinge + asymmetric cone)",
                 {gateMin("knee derived as hinge", isHinge ? 1.0 : 0.0, 1.0),
                  gateMax("knee 170deg clamps to 155", std::abs(angleFromRest(deep) - 155.0), 0.5),
                  gateMax("knee -40deg clamps to -11", std::abs(angleFromRest(hyper) - 11.0), 0.5),
                  gateMax("knee sideways bend removed", std::abs(side.x), 0.02),
                  gateMin("hip derived as per-axis cone", isCone ? 1.0 : 0.0, 1.0),
                  gateMax("hip 150deg kick clamps to 115", std::abs(angleFromRest(kick) - 115.0), 0.5),
                  gateMax("hip 60deg back-kick clamps to 35", std::abs(angleFromRest(back) - 35.0), 0.5)});
}

void testSkeletonGraph(Report& report) {
    // 0 -> 1 -> 2 -> 3, 1 -> 4 -> 5 (a Y-shaped tree).
    const std::vector<int> parents{-1, 0, 1, 2, 1, 4};
    SkeletonGraph graph;
    graph.build(parents);
    const bool rootedOk = graph.root() == 0 && graph.parentOf(3) == 2;
    graph.setRoot(5);
    const bool rerooted = graph.parentOf(4) == 5 && graph.parentOf(1) == 4 &&
                          graph.parentOf(0) == 1 && graph.parentOf(2) == 1 &&
                          graph.traversalOrder().front() == 5;
    const std::vector<char> active = graph.markActivePaths({3});
    const bool marks = active[3] && active[2] && active[1] && active[4] && active[5] && !active[0];
    report.phase("[unit] skeleton graph re-rooting",
                 {gateMin("initial rooting", rootedOk ? 1.0 : 0.0, 1.0),
                  gateMin("re-rooted at a leaf", rerooted ? 1.0 : 0.0, 1.0),
                  gateMin("active path marking", marks ? 1.0 : 0.0, 1.0)});
}

void testBalance(Report& report) {
    const std::vector<glm::vec2> pts{{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0.5f, 0.5f}};
    const std::vector<glm::vec2> hull = BalanceController::supportPolygon(pts);
    const bool inside = BalanceController::insidePolygon(hull, glm::vec2(0.5f, 0.5f));
    const bool outside = !BalanceController::insidePolygon(hull, glm::vec2(1.5f, 0.5f));
    const glm::vec2 fixed = BalanceController::closestBalancedPoint(hull, glm::vec2(1.5f, 0.5f), 0.05f);
    report.phase("[unit] support polygon",
                 {gateMax("hull vertex count error", std::abs(static_cast<double>(hull.size()) - 4.0), 0.0),
                  gateMin("inside test", inside ? 1.0 : 0.0, 1.0),
                  gateMin("outside test", outside ? 1.0 : 0.0, 1.0),
                  gateMax("balanced point x error", std::abs(fixed.x - 0.95), 1e-3)});
}

// ------------------------------------------------------------------------------------------
// The drag-loop simulator: the viewport's IK tick, without the viewport.
// ------------------------------------------------------------------------------------------

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
    double                   contactMinY = 1e9;       // lowest a contact pin went (world)
    double                   contactBindMinY = 1e9;   // its rest height (the floor reference)
    // Displacements of named joints (start -> end).
    double                   hipDisp = 0.0, hipDispY = 0.0;
    double                   headDisp = 0.0, chestDisp = 0.0;
    double                   idleOscMax = 0.0;        // worst reversal overlap over idle joints
    double                   idleStepMax = 0.0;       // worst single-tick idle-joint step
    // Effector.
    int                      grabIndex = -1, effectorIndex = -1;
    glm::vec3                grabStart{0.0f}, grabEnd{0.0f}, cursorEnd{0.0f};
    double                   restingMiss = 0.0;       // |grab - cursor| at the end of the drag
    double                   holdDrift = 0.0;         // effector motion through the settle
    // Settle.
    int                      settleTicks = 0;
    double                   settleMaxStep = 0.0;
    double                   settleWorstPinErr = 0.0; // contact pins after the settle
    // Stepping / suspension.
    int                      stepsTaken = 0;
    bool                     suspended = false;
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
    double                   minJointY = 1e9;         // lowest any body joint went vs its bind height (penetration)
};

std::vector<ArmatureBone> scaledBones(const std::vector<ArmatureBone>& bones, float scale) {
    std::vector<ArmatureBone> out = bones;
    for (ArmatureBone& b : out) {
        b.localBindTranslation *= scale;
    }
    return out;
}

double rotationAngleDeg(const glm::mat3& a, const glm::mat3& b) {
    const glm::mat3 rel = glm::transpose(a) * b;
    const float c = glm::clamp((rel[0][0] + rel[1][1] + rel[2][2] - 1.0f) * 0.5f, -1.0f, 1.0f);
    return glm::degrees(std::acos(c));
}

RunResult runScenario(const std::vector<ArmatureBone>& baseBones, const Scenario& sc,
                      bool verbose) {
    RunResult r;
    Armature arm;
    arm.build(sc.scale == 1.0f ? baseBones : scaledBones(baseBones, sc.scale));
    const std::size_t n = arm.boneCount();
    for (const auto& [bone, euler] : sc.prePose) {
        if (!arm.setBoneRotation(bone, euler)) {
            r.error = "pre-pose bone not found: " + bone;
            return r;
        }
    }
    if (sc.hover != 0.0f) {
        auto pose = arm.capturePose();
        // The rig root is the first multi-child descendant of the anatomical root (the hip on
        // real figures); the hover goes on the hip's pose translation like the engine writes it.
        int hip = arm.boneIndex("hip");
        if (hip < 0) {
            hip = 0;
        }
        pose.emplace_back("@trans:" + arm.boneName(static_cast<std::size_t>(hip)),
                          glm::vec3(0.0f, sc.hover, 0.0f));
        arm.applyPose(pose);
    }
    for (const std::string& pin : sc.userPins) {
        const int idx = arm.selectBoneByName(pin);
        if (idx < 0) {
            r.error = "pin bone not found: " + pin;
            return r;
        }
        arm.togglePinSelectedBone();
    }
    r.grabIndex = arm.selectBoneByName(sc.grab);
    if (r.grabIndex < 0) {
        r.error = "grab bone not found: " + sc.grab;
        return r;
    }
    if (!arm.beginIkDrag()) {
        r.error = "beginIkDrag failed";
        return r;
    }
    const IkRig* rig = arm.ikRig();
    r.effectorIndex = rig->dragEffector();
    r.contactPins = arm.activeContactPins();
    r.startPos.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        r.startPos[i] = arm.boneWorldPosition(i);
    }
    r.grabStart = r.startPos[static_cast<std::size_t>(r.grabIndex)];
    std::vector<glm::vec3> contactStart;
    for (const int c : r.contactPins) {
        contactStart.push_back(r.startPos[static_cast<std::size_t>(c)]);
        // Bind height: the local translations summed (the armature's transform is identity).
        float y = 0.0f;
        for (int cur = c; cur >= 0; cur = arm.boneParent(static_cast<std::size_t>(cur))) {
            y += (sc.scale == 1.0f ? baseBones : scaledBones(baseBones, sc.scale))[static_cast<std::size_t>(cur)]
                     .localBindTranslation.y;
        }
        r.contactBindMinY = std::min(r.contactBindMinY, static_cast<double>(y));
    }
    // User-pin state (world position + rotation at drag start).
    std::vector<int> userPinIdx;
    std::vector<glm::vec3> userPinPos;
    std::vector<glm::mat3> userPinRot;
    for (const std::string& pin : sc.userPins) {
        const int idx = arm.boneIndex(pin);
        userPinIdx.push_back(idx);
        userPinPos.push_back(arm.boneWorldPosition(static_cast<std::size_t>(idx)));
        userPinRot.push_back(glm::mat3(arm.poseGlobal(static_cast<std::size_t>(idx))));
    }
    const int hip = arm.boneIndex("hip");
    const int head = arm.boneIndex("head");
    const int chest = arm.boneIndex("chest");
    // The feet, for the suspension / floor checks.
    std::vector<int> feet;
    for (const char* f : {"lFoot", "rFoot"}) {
        const int idx = arm.boneIndex(f);
        if (idx >= 0) {
            feet.push_back(idx);
        }
    }
    // Bind heights of every joint (penetration check).
    std::vector<float> bindY(n, 0.0f);
    {
        const std::vector<ArmatureBone>& bones = sc.scale == 1.0f ? baseBones : scaledBones(baseBones, sc.scale);
        for (std::size_t i = 0; i < n; ++i) {
            const int p = bones[i].parent;
            bindY[i] = bones[i].localBindTranslation.y + (p >= 0 ? bindY[static_cast<std::size_t>(p)] : 0.0f);
        }
    }

    // Body joints (below the rig root) — the floor rule applies to these only.
    std::vector<char> bodyNode(n, 0);
    for (std::size_t i = 0; i < n; ++i) {
        for (int cur = static_cast<int>(i); cur >= 0; cur = arm.boneParent(static_cast<std::size_t>(cur))) {
            if (cur == rig->rootNode()) {
                bodyNode[i] = 1;
                break;
            }
        }
    }
    IkCursorFilter filter;
    filter.seed(r.grabStart);
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> noise(-1.0f, 1.0f);
    std::vector<glm::vec3> prev(n);
    for (std::size_t i = 0; i < n; ++i) {
        prev[i] = arm.boneWorldPosition(i);
    }
    glm::vec3 prevEff = r.grabStart;
    glm::vec3 prevEffStep(0.0f);
    double prevEffSpeed = 0.0;
    glm::vec3 prevCursor = r.grabStart;
    double prevCursorSpeed = 0.0;
    r.phases.resize(sc.path.size() > 1 ? sc.path.size() - 1 : 0);
    for (std::size_t s = 0; s + 1 < sc.path.size(); ++s) {
        r.phases[s].label = sc.path[s + 1].label;
    }
    glm::vec3 raw = r.grabStart;
    int tick = 0;
    while (true) {
        ++tick;
        // The scripted cursor (mirrors VulkanWindow::benchAdvance).
        std::size_t seg = sc.path.size();
        for (std::size_t s = 1; s < sc.path.size(); ++s) {
            if (tick <= sc.path[s].tick) {
                const float f = static_cast<float>(tick - sc.path[s - 1].tick) /
                                static_cast<float>(sc.path[s].tick - sc.path[s - 1].tick);
                raw = r.grabStart + glm::mix(sc.path[s - 1].offset, sc.path[s].offset, f);
                seg = s - 1;
                break;
            }
        }
        if (seg >= sc.path.size()) {
            break; // past the last waypoint: release
        }
        if (sc.cursorNoise > 0.0f) {
            raw += glm::vec3(noise(rng), noise(rng), noise(rng)) * sc.cursorNoise;
        }
        PhaseStats& ph = r.phases[seg];
        const bool moved = arm.dragIkTo(filter.update(raw));
        ++ph.ticks;
        ++r.dragTicks;
        if (moved) {
            ++ph.movedTicks;
        }
        const glm::vec3 eff = arm.boneWorldPosition(static_cast<std::size_t>(r.grabIndex));
        const double lag = glm::length(eff - raw);
        ph.lagSum += lag;
        ph.lagMax = std::max(ph.lagMax, lag);
        // Motion profile of the grabbed joint and of the cursor.
        const glm::vec3 effStep = eff - prevEff;
        const double effSpeed = glm::length(effStep);
        ph.effSpeedUpMax = std::max(ph.effSpeedUpMax, effSpeed - prevEffSpeed);
        ph.effSpeedDownMax = std::max(ph.effSpeedDownMax, prevEffSpeed - effSpeed);
        const double cursorSpeed = glm::length(raw - prevCursor);
        ph.cursorSpeedUpMax = std::max(ph.cursorSpeedUpMax, cursorSpeed - prevCursorSpeed);
        ph.cursorSpeedDownMax = std::max(ph.cursorSpeedDownMax, prevCursorSpeed - cursorSpeed);
        prevCursorSpeed = cursorSpeed;
        prevCursor = raw;
        // Oscillation (the app's [ikperf] osc): reversal overlap of the grabbed joint's steps.
        {
            const float pl = glm::length(prevEffStep);
            if (pl > 1e-6f && effSpeed > 1e-6) {
                const float against = -glm::dot(effStep, prevEffStep) / pl;
                if (against > 0.0f) {
                    ph.effOsc += std::min(static_cast<double>(against), static_cast<double>(pl));
                }
            }
            prevEffStep = effStep;
        }
        prevEff = eff;
        prevEffSpeed = effSpeed;
        // Per-joint steps: jumps, idle oscillation, contact drift, pin fidelity, floor.
        for (std::size_t i = 0; i < n; ++i) {
            const glm::vec3 p = arm.boneWorldPosition(i);
            const double step = glm::length(p - prev[i]);
            ph.maxJump = std::max(ph.maxJump, step);
            r.minJointY = std::min(r.minJointY, static_cast<double>(p.y - bindY[i]));
            if (bodyNode[i]) {
                const double pen = rig->floorClearance(static_cast<int>(i)) - p.y;
                if (pen > r.penetrationMax) {
                    r.penetrationMax = pen;
                    r.penetrationNode = static_cast<int>(i);
                }
            }
            prev[i] = p;
        }
        for (std::size_t c = 0; c < r.contactPins.size(); ++c) {
            const glm::vec3 p = arm.boneWorldPosition(static_cast<std::size_t>(r.contactPins[c]));
            r.contactDriftMaxDrag = std::max(r.contactDriftMaxDrag, static_cast<double>(glm::length(p - contactStart[c])));
            r.contactMinY = std::min(r.contactMinY, static_cast<double>(p.y));
        }
        for (std::size_t k = 0; k < userPinIdx.size(); ++k) {
            const std::size_t j = static_cast<std::size_t>(userPinIdx[k]);
            const glm::vec3 p = arm.boneWorldPosition(j);
            r.userPinDistMax = std::max(r.userPinDistMax, static_cast<double>(glm::length(p - userPinPos[k])));
            r.userPinRotMaxDeg = std::max(r.userPinRotMaxDeg, rotationAngleDeg(userPinRot[k], glm::mat3(arm.poseGlobal(j))));
        }
        if (rig->pins().empty() && !r.contactPins.empty()) {
            r.suspended = true;
        }
        r.stepsTaken = rig->stepsTaken();
    }
    // (Idle-joint tremble metrics come from runIdleMetrics — a second, identical run that keeps
    // the per-joint step history; the main loop stays a plain mirror of the viewport's tick.)
    r.grabEnd = arm.boneWorldPosition(static_cast<std::size_t>(r.grabIndex));
    r.cursorEnd = raw;
    r.dragEndPos.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        r.dragEndPos[i] = arm.boneWorldPosition(i);
    }
    if (hip >= 0) {
        r.hipDropAtRelease = r.startPos[static_cast<std::size_t>(hip)].y -
                             r.dragEndPos[static_cast<std::size_t>(hip)].y;
    }
    r.restingMiss = glm::length(r.grabEnd - raw);
    // Release settle.
    if (sc.release) {
        std::vector<glm::vec3> before(n);
        int guard = 0;
        while (true) {
            for (std::size_t i = 0; i < n; ++i) {
                before[i] = arm.boneWorldPosition(i);
            }
            const bool more = arm.settleIkTick();
            if (!more) {
                break;
            }
            ++r.settleTicks;
            for (std::size_t i = 0; i < n; ++i) {
                r.settleMaxStep = std::max(r.settleMaxStep, static_cast<double>(glm::length(arm.boneWorldPosition(i) - before[i])));
                if (bodyNode[i]) {
                    r.penetrationMax = std::max(
                        r.penetrationMax,
                        static_cast<double>(rig->floorClearance(static_cast<int>(i)) -
                                            arm.boneWorldPosition(i).y));
                }
            }
            for (std::size_t k = 0; k < userPinIdx.size(); ++k) {
                const std::size_t j = static_cast<std::size_t>(userPinIdx[k]);
                r.userPinStepMax = std::max(r.userPinStepMax, static_cast<double>(glm::length(arm.boneWorldPosition(j) - before[j])));
                r.userPinDistMax = std::max(r.userPinDistMax, static_cast<double>(glm::length(arm.boneWorldPosition(j) - userPinPos[k])));
            }
            if (++guard > 200) {
                break;
            }
        }
        const std::vector<IkEffector>& pins = rig->pins();
        for (std::size_t p = 0; p < pins.size(); ++p) {
            if (rig->pinIsUser(p)) {
                continue;
            }
            const glm::vec3 pos(arm.poseGlobal(static_cast<std::size_t>(pins[p].node))[3]);
            r.settleWorstPinErr = std::max(r.settleWorstPinErr, static_cast<double>(glm::length(pos - pins[p].target)));
        }
    }
    r.holdDrift = glm::length(arm.boneWorldPosition(static_cast<std::size_t>(r.grabIndex)) - r.grabEnd);
    arm.endIkDrag();
    r.endPos.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        r.endPos[i] = arm.boneWorldPosition(i);
    }
    for (std::size_t c = 0; c < r.contactPins.size(); ++c) {
        r.contactDriftMax = std::max(r.contactDriftMax, static_cast<double>(glm::length(r.endPos[static_cast<std::size_t>(r.contactPins[c])] - contactStart[c])));
    }
    if (hip >= 0) {
        r.hipDisp = glm::length(r.endPos[static_cast<std::size_t>(hip)] - r.startPos[static_cast<std::size_t>(hip)]);
        r.hipDispY = r.endPos[static_cast<std::size_t>(hip)].y - r.startPos[static_cast<std::size_t>(hip)].y;
    }
    if (head >= 0) {
        r.headDisp = glm::length(r.endPos[static_cast<std::size_t>(head)] - r.startPos[static_cast<std::size_t>(head)]);
    }
    if (chest >= 0) {
        r.chestDisp = glm::length(r.endPos[static_cast<std::size_t>(chest)] - r.startPos[static_cast<std::size_t>(chest)]);
    }
    for (const int f : feet) {
        r.feetMinY = std::min(r.feetMinY, static_cast<double>(r.endPos[static_cast<std::size_t>(f)].y));
    }
    r.ok = true;
    if (verbose) {
        for (const PhaseStats& ph : r.phases) {
            std::printf("         %-10s ticks %3d moved %3d | lag mean %6.1f max %6.1f mm | osc %6.1f mm | jump %5.1f mm | eff dv +%5.1f -%5.1f (cursor +%5.1f -%5.1f) mm/tick\n",
                        ph.label.c_str(), ph.ticks, ph.movedTicks, ph.lagMean() * 1000.0, ph.lagMax * 1000.0,
                        ph.effOsc * 1000.0, ph.maxJump * 1000.0, ph.effSpeedUpMax * 1000.0,
                        ph.effSpeedDownMax * 1000.0, ph.cursorSpeedUpMax * 1000.0, ph.cursorSpeedDownMax * 1000.0);
        }
        std::printf("         settle %d ticks, max step %.1f mm, worst pin %.1f mm; steps %d; suspended %d\n",
                    r.settleTicks, r.settleMaxStep * 1000.0, r.settleWorstPinErr * 1000.0, r.stepsTaken,
                    r.suspended ? 1 : 0);
    }
    return r;
}

// A second, lighter simulator pass for the IDLE-joint metrics (per-tick steps of joints that
// should not move during a gesture): tremble = accumulated reversal overlap, and the worst
// single-tick step. Kept separate so the main loop stays a faithful mirror of the viewport.
struct IdleMetrics {
    double oscMax = 0.0;   // worst accumulated reversal overlap over the idle joints
    double stepMax = 0.0;  // worst single-tick step of an idle joint
    double dispMax = 0.0;  // worst start->end displacement of an idle joint
};

IdleMetrics runIdleMetrics(const std::vector<ArmatureBone>& baseBones, const Scenario& sc) {
    IdleMetrics m;
    Armature arm;
    arm.build(sc.scale == 1.0f ? baseBones : scaledBones(baseBones, sc.scale));
    for (const auto& [bone, euler] : sc.prePose) {
        arm.setBoneRotation(bone, euler);
    }
    for (const std::string& pin : sc.userPins) {
        if (arm.selectBoneByName(pin) >= 0) {
            arm.togglePinSelectedBone();
        }
    }
    const int grab = arm.selectBoneByName(sc.grab);
    if (grab < 0 || !arm.beginIkDrag()) {
        return m;
    }
    std::vector<int> idle;
    for (const std::string& name : sc.idleJoints) {
        const int idx = arm.boneIndex(name);
        if (idx >= 0) {
            idle.push_back(idx);
        }
    }
    const glm::vec3 start = arm.boneWorldPosition(static_cast<std::size_t>(grab));
    std::vector<glm::vec3> startPos(idle.size()), prevPos(idle.size()), prevStep(idle.size(), glm::vec3(0.0f));
    std::vector<double> osc(idle.size(), 0.0);
    for (std::size_t k = 0; k < idle.size(); ++k) {
        startPos[k] = prevPos[k] = arm.boneWorldPosition(static_cast<std::size_t>(idle[k]));
    }
    IkCursorFilter filter;
    filter.seed(start);
    std::mt19937 rng(12345);
    std::uniform_real_distribution<float> noise(-1.0f, 1.0f);
    glm::vec3 raw = start;
    int tick = 0;
    while (true) {
        ++tick;
        bool done = true;
        for (std::size_t s = 1; s < sc.path.size(); ++s) {
            if (tick <= sc.path[s].tick) {
                const float f = static_cast<float>(tick - sc.path[s - 1].tick) /
                                static_cast<float>(sc.path[s].tick - sc.path[s - 1].tick);
                raw = start + glm::mix(sc.path[s - 1].offset, sc.path[s].offset, f);
                done = false;
                break;
            }
        }
        if (done) {
            break;
        }
        if (sc.cursorNoise > 0.0f) {
            raw += glm::vec3(noise(rng), noise(rng), noise(rng)) * sc.cursorNoise;
        }
        arm.dragIkTo(filter.update(raw));
        for (std::size_t k = 0; k < idle.size(); ++k) {
            const glm::vec3 p = arm.boneWorldPosition(static_cast<std::size_t>(idle[k]));
            const glm::vec3 step = p - prevPos[k];
            const float len = glm::length(step);
            m.stepMax = std::max(m.stepMax, static_cast<double>(len));
            const float pl = glm::length(prevStep[k]);
            if (pl > 1e-6f && len > 1e-6f) {
                const float against = -glm::dot(step, prevStep[k]) / pl;
                if (against > 0.0f) {
                    osc[k] += std::min(static_cast<double>(against), static_cast<double>(pl));
                }
            }
            prevStep[k] = step;
            prevPos[k] = p;
        }
    }
    for (std::size_t k = 0; k < idle.size(); ++k) {
        m.oscMax = std::max(m.oscMax, osc[k]);
        m.dispMax = std::max(m.dispMax, static_cast<double>(glm::length(prevPos[k] - startPos[k])));
    }
    arm.endIkDrag();
    return m;
}

std::vector<Waypoint> pullPath(const glm::vec3& offset, int moveTicks, int holdTicks) {
    return {{0, glm::vec3(0.0f), "start"}, {moveTicks, offset, "move"}, {moveTicks + holdTicks, offset, "hold"}};
}

// ------------------------------------------------------------------------------------------
// Real-skeleton phases. Each reproduces a user-reported behaviour and gates the measurements
// that once failed (see CLAUDE.md's FBIK section for the history behind every number).
// ------------------------------------------------------------------------------------------

const std::vector<std::string> kIdleForHandDrag{"head", "rHand", "rFoot", "lFoot", "hip"};
const std::vector<std::string> kIdleForHeadDrag{"lHand", "rHand", "rFoot", "lFoot"};

void realPhases(Report& report, const std::vector<ArmatureBone>& bones) {
    const bool v = report.verbose;
    auto run = [&](const Scenario& sc) { return runScenario(bones, sc, v); };

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
        const IdleMetrics idle = runIdleMetrics(bones, sc);
        std::vector<Gate> gates;
        if (!r.ok) {
            gates.push_back(gateMin(("error: " + r.error).c_str(), 0.0, 1.0));
        } else {
            gates = {gateMax("move-med lag mean (mm)", r.phases[0].lagMean() * 1000.0, 15.0),
                     gateMax("hold-1 resting lag (mm)", r.phases[1].lagMax * 1000.0, 15.0),
                     gateMax("return lag mean (mm)", r.phases[2].lagMean() * 1000.0, 8.0),
                     gateMax("hold-2 return-to-rest miss (mm)", r.phases[3].lagMax * 1000.0, 3.0),
                     gateMax("move-fast lag mean (mm)", r.phases[4].lagMean() * 1000.0, 30.0),
                     gateMax("hold-3 flick resting miss (mm)", r.phases[5].lagMax * 1000.0, 5.0),
                     gateMax("worst single-tick jump (mm)", std::max({r.phases[0].maxJump, r.phases[2].maxJump, r.phases[4].maxJump}) * 1000.0, 60.0),
                     gateMax("grabbed-joint accel excess (mm/tick^2)", std::max(0.0, r.phases[4].effSpeedUpMax - r.phases[4].cursorSpeedUpMax) * 1000.0, 12.0),
                     gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0),
                     gateMax("idle-joint tremble (mm)", idle.oscMax * 1000.0, 35.0),
                     gateMax("settle ticks", r.settleTicks, 45.0),
                     gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0)};
            for (const PhaseStats& ph : r.phases) {
                gates.push_back(info((ph.label + " lag mean (mm)").c_str(), ph.lagMean() * 1000.0));
            }
        }
        report.phase("[real] bench path: lHand (the app's POSESTUDIO_IK_BENCH)", gates);
    }

    // --- A single gentle side pull holds the feet and the hip; the hand lands near the cursor.
    if (report.wants("gentle")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.path = pullPath(glm::vec3(0.10f, 0.0f, 0.0f), 30, 60);
        sc.idleJoints = kIdleForHandDrag;
        const RunResult r = run(sc);
        report.phase("[real] gentle side pull (lHand +10cm x)",
                     {gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0),
                      gateMax("hip displacement (mm)", r.hipDisp * 1000.0, 10.0),
                      gateMax("hand-to-cursor at rest (mm)", r.restingMiss * 1000.0, 25.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0),
                      info("steps", r.stepsTaken)});
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
        double stepJumpMax = 0.0;
        double stepSpeedUpMax = 0.0;
        for (const PhaseStats& ph : r.phases) {
            stepJumpMax = std::max(stepJumpMax, ph.maxJump);
            stepSpeedUpMax = std::max(stepSpeedUpMax, ph.effSpeedUpMax);
        }
        report.phase("[real] wheel depth steps (lHand 6 x 5cm z notches)",
                     {gateMax("hand-to-cursor at rest (mm)", r.restingMiss * 1000.0, 25.0),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0),
                      gateMax("hip displacement (mm)", r.hipDisp * 1000.0, 40.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0),
                      gateMax("idle-joint oscillation", r.idleOscMax, 0.035),
                      info("worst single-tick joint jump (mm)", stepJumpMax * 1000.0),
                      info("worst hand speed-up per tick (mm)", stepSpeedUpMax * 1000.0),
                      info("steps", r.stepsTaken)});
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
        const int lShin = arm.boneIndex("lShin"), rShin = arm.boneIndex("rShin");
        const int lFoot = arm.boneIndex("lFoot"), rFoot = arm.boneIndex("rFoot");
        const int lForeArm = arm.boneIndex("lForeArm"), rForeArm = arm.boneIndex("rForeArm");
        const int lHand = arm.boneIndex("lHand"), rHand = arm.boneIndex("rHand");
        const int abdomen = arm.boneIndex("abdomenLower"), hip = arm.boneIndex("hip");
        const int head = arm.boneIndex("head");
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
                {"lShin", glm::vec3(120.0f, -8.0f, 3.0f)},
                {"lForeArm", glm::vec3(0.0f, -100.0f, 0.0f)},
                {"lHand", glm::vec3(5.0f, -20.0f, 30.0f)},
                {"abdomenLower", glm::vec3(10.0f, 5.0f, -5.0f)},
                {"head", glm::vec3(-10.0f, 10.0f, 5.0f)},
                {"@trans:hip", glm::vec3(0.05f, -0.10f, 0.02f)},
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
            arm.selectBoneByName("rHand");
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
        report.phase("[real] arm raise (lHand +60cm y, reachable)",
                     {gateMin("hand rise (m)", r.grabEnd.y - r.grabStart.y, 0.59),
                      gateMax("head displacement (mm)", r.headDisp * 1000.0, 15.0),
                      gateMax("chest displacement (mm)", r.chestDisp * 1000.0, 15.0),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0),
                      gateMax("suspended", r.suspended ? 1.0 : 0.0, 0.0),
                      info("hand-to-cursor at rest (mm)", r.restingMiss * 1000.0)});
    }

    // --- A sustained pull far beyond reach LIFTS the figure: the feet leave the floor and the
    // body dangles below the grab point.
    if (report.wants("suspension")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.path = pullPath(glm::vec3(0.0f, 1.20f, 0.0f), 60, 120);
        sc.release = false;
        const RunResult r = run(sc);
        report.phase("[real] suspension (lHand +120cm y, beyond reach)",
                     {gateMin("suspended", r.suspended ? 1.0 : 0.0, 1.0),
                      gateMin("feet height at the end (m)", r.feetMinY, 0.20),
                      gateMin("hip rise (m)", r.hipDispY, 0.10)});
    }

    // --- Pushing the chest down folds a crouch over the planted feet; no foot buries itself.
    if (report.wants("crouch-push")) {
        Scenario sc;
        sc.grab = "chest_2";
        sc.path = pullPath(glm::vec3(0.0f, -0.25f, 0.0f), 60, 90);
        const RunResult r = run(sc);
        report.phase("[real] chest push-down (chest_2 -25cm y)",
                     {gateMin("hip drop at release (m)", r.hipDropAtRelease, 0.05),
                      info("hip drop after settle (m)", -r.hipDispY),
                      gateMax("lowest contact vs floor, drag (mm)", (r.contactBindMinY - r.contactMinY) * 1000.0, 10.0),
                      gateMax("floor penetration, any joint (mm)", r.penetrationMax * 1000.0, 30.0),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 80.0),
                      gateMin("lowest contact vs floor (mm)", (r.contactMinY - r.contactBindMinY) * 1000.0, -45.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0)});
    }

    // --- Dragging the hip down is an explicit crouch: deep, feet planted.
    if (report.wants("hip-crouch")) {
        Scenario sc;
        sc.grab = "hip";
        sc.path = pullPath(glm::vec3(0.0f, -0.12f, 0.0f), 40, 80);
        const RunResult r = run(sc);
        report.phase("[real] hip crouch (hip -12cm y)",
                     {gateMin("hip drop at release (m)", r.hipDropAtRelease, 0.10),
                      info("hip drop after settle (m)", -r.hipDispY),
                      gateMax("lowest contact vs floor, drag (mm)", (r.contactBindMinY - r.contactMinY) * 1000.0, 5.0),
                      // The toe joints ride the sole's transient pitch ~2cm below their rest
                      // height (a centimetre through the floor) — a known soft spot, guarded.
                      gateMax("floor penetration, any joint (mm)", r.penetrationMax * 1000.0, 30.0),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 30.0),
                      gateMin("lowest contact vs floor (mm)", (r.contactMinY - r.contactBindMinY) * 1000.0, -30.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0)});
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
        report.phase("[real] hover heal (3cm hover, gentle hand pull)",
                     {gateMax("lowest contact vs floor after (mm)", std::abs(r.contactBindMinY < 1e8 ? feetY - r.contactBindMinY : 0.0) * 1000.0, 5.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0)});
    }

    // --- A strained pull (beyond the arm's reach, forward) and its release: the pins land
    // exactly, every settle tick small.
    if (report.wants("strained-release")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.path = pullPath(glm::vec3(0.0f, 0.10f, 0.55f), 60, 60);
        const RunResult r = run(sc);
        report.phase("[real] strained pull + release (lHand +55cm z)",
                     {gateMax("worst pin after settle (mm)", r.settleWorstPinErr * 1000.0, 2.0),
                      gateMax("feet drift after settle (mm)", r.contactDriftMax * 1000.0, 20.0),
                      gateMax("steps taken", r.stepsTaken, 0.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0),
                      gateMax("settle ticks", r.settleTicks, 45.0),
                      info("hand-to-cursor at rest (mm)", r.restingMiss * 1000.0),
                      info("hand moved through settle (mm)", r.holdDrift * 1000.0)});
    }

    // --- Dragging a foot back and up TRACKS the lift and holds after release.
    if (report.wants("foot-lift")) {
        Scenario sc;
        sc.grab = "lFoot";
        sc.path = pullPath(glm::vec3(0.0f, 0.15f, -0.10f), 40, 80);
        sc.idleJoints = {"head", "lHand", "rHand", "rFoot"};
        const RunResult r = run(sc);
        report.phase("[real] foot lift (lFoot +15cm y, -10cm z)",
                     {gateMin("foot lift reached (m)", r.grabEnd.y - r.grabStart.y, 0.13),
                      gateMax("foot-to-cursor at rest (mm)", r.restingMiss * 1000.0, 15.0),
                      gateMax("foot moved through settle (mm)", r.holdDrift * 1000.0, 10.0),
                      gateMax("standing foot drift (mm)", r.contactDriftMax * 1000.0, 15.0)});
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
        report.phase(name, {gateMin("steps taken", r.stepsTaken, dist > 0.5f ? 5.0 : 3.0),
                            gateMax("hip-to-target at rest (mm)", r.restingMiss * 1000.0, 15.0),
                            gateMax("feet off floor at the end (mm)", feetOffFloor * 1000.0, 15.0),
                            gateMin("both feet followed (m)", feetMoved, dist - 0.13),
                            gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0)});
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
        report.phase("[real] stepping chest drag (chest +45cm x)",
                     {gateMin("steps taken", r.stepsTaken, 2.0),
                      gateMax("chest-to-cursor at rest (mm)", r.restingMiss * 1000.0, 150.0),
                      gateMax("feet off floor at the end (mm)", feetOffFloor * 1000.0, 15.0),
                      gateMax("settle max step (mm)", r.settleMaxStep * 1000.0, 20.0)});
    }

    // --- Pinned joints hold EXACTLY while other things are dragged.
    if (report.wants("pin-hand-drag")) {
        Scenario sc;
        sc.grab = "rHand";
        sc.userPins = {"lHand"};
        sc.path = pullPath(glm::vec3(-0.45f, 0.10f, 0.0f), 80, 80);
        const RunResult r = run(sc);
        report.phase("[real] pinned lHand under a 45cm rHand drag",
                     {gateMax("pinned joint max distance (mm)", r.userPinDistMax * 1000.0, 10.0),
                      gateMax("pinned joint rotation (deg)", r.userPinRotMaxDeg, 1.0),
                      gateMax("pinned joint settle step (mm)", r.userPinStepMax * 1000.0, 1.0)});
    }
    if (report.wants("pin-hand-crouch")) {
        Scenario sc;
        sc.grab = "hip";
        sc.userPins = {"lHand"};
        sc.path = pullPath(glm::vec3(0.0f, -0.12f, 0.0f), 40, 100);
        const RunResult r = run(sc);
        report.phase("[real] pinned lHand through a 12cm hip crouch",
                     {gateMax("pinned joint max distance (mm)", r.userPinDistMax * 1000.0, 10.0),
                      gateMax("pinned joint rotation (deg)", r.userPinRotMaxDeg, 1.0),
                      gateMin("hip drop at release (m)", r.hipDropAtRelease, 0.10)});
    }
    if (report.wants("pin-foot-chest")) {
        Scenario sc;
        sc.grab = "chest_2";
        sc.userPins = {"lFoot"};
        sc.path = pullPath(glm::vec3(0.45f, 0.0f, 0.0f), 110, 150);
        const RunResult r = run(sc);
        report.phase("[real] pinned lFoot under a 45cm chest drag",
                     {gateMax("pinned joint max distance (mm)", r.userPinDistMax * 1000.0, 15.0),
                      gateMax("steps taken", r.stepsTaken, 0.0)});
    }

    // --- Trembling: slow and medium pulls with cursor noise must not shake idle joints.
    if (report.wants("tremble-slow")) {
        Scenario sc;
        sc.grab = "lHand";
        sc.path = pullPath(glm::vec3(0.06f, 0.0f, 0.0f), 180, 60); // ~2 cm/s
        sc.cursorNoise = 0.002f;
        sc.idleJoints = kIdleForHandDrag;
        const RunResult r = run(sc);
        const IdleMetrics idle = runIdleMetrics(bones, sc);
        report.phase("[real] tremble: slow noisy pull (lHand 2cm/s, +-2mm noise)",
                     {gateMax("idle-joint tremble (mm)", idle.oscMax * 1000.0, 10.0),
                      gateMax("idle-joint worst step (mm)", idle.stepMax * 1000.0, 10.0),
                      gateMax("grabbed-joint osc (mm)", (r.phases[0].effOsc + r.phases[1].effOsc) * 1000.0, 60.0)});
    }
    if (report.wants("tremble-med")) {
        Scenario sc;
        sc.grab = "head";
        sc.path = pullPath(glm::vec3(0.10f, 0.0f, 0.05f), 60, 60);
        sc.cursorNoise = 0.002f;
        sc.idleJoints = kIdleForHeadDrag;
        const RunResult r = run(sc);
        const IdleMetrics idle = runIdleMetrics(bones, sc);
        report.phase("[real] tremble: medium noisy head pull (+-2mm noise)",
                     {gateMax("idle-joint tremble (mm)", idle.oscMax * 1000.0, 35.0),
                      gateMax("grabbed-joint osc (mm)", (r.phases[0].effOsc + r.phases[1].effOsc) * 1000.0, 80.0)});
    }

    // --- Grabbing an EYE (a face bone promoted to the head) must not make the head tremble.
    if (report.wants("eye-grab")) {
        Scenario sc;
        sc.grab = "lEye";
        sc.path = pullPath(glm::vec3(0.10f, 0.0f, 0.05f), 60, 90);
        sc.cursorNoise = 0.002f;
        const RunResult r = run(sc);
        report.phase("[real] eye grab (lEye promoted to the head, +-2mm noise)",
                     {gateMax("grabbed-joint osc (mm)", (r.phases[0].effOsc + r.phases[1].effOsc) * 1000.0, 80.0),
                      gateMax("head-to-cursor at rest (mm)", r.restingMiss * 1000.0, 60.0)});
    }

    // --- Grabbing a FINGER is an arm gesture: a straight-up pull raises the arm overhead.
    if (report.wants("finger-raise")) {
        Scenario sc;
        sc.grab = "lIndex3";
        sc.path = pullPath(glm::vec3(0.0f, 0.60f, 0.0f), 60, 90);
        const RunResult r = run(sc);
        report.phase("[real] finger grab raise (lIndex3 +60cm y)",
                     {gateMin("finger rise (m)", r.grabEnd.y - r.grabStart.y, 0.59),
                      gateMax("head displacement (mm)", r.headDisp * 1000.0, 20.0),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0)});
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
        report.phase("[real] slouched head pull-up (head +15cm y)",
                     {gateMin("head rise (m)", r.grabEnd.y - r.grabStart.y, 0.035),
                      gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 10.0),
                      gateMax("steps taken", r.stepsTaken, 0.0)});
    }

    // --- A child-scale figure (0.54) crouches and walks with its feet planted.
    if (report.wants("child")) {
        {
            Scenario sc;
            sc.grab = "hip";
            sc.scale = 0.54f;
            sc.path = pullPath(glm::vec3(0.0f, -0.07f, 0.0f), 40, 80);
            const RunResult r = run(sc);
            report.phase("[real] child crouch (0.54 scale, hip -7cm)",
                         {gateMin("hip drop (m)", -r.hipDispY, 0.05),
                          gateMax("feet drift (mm)", r.contactDriftMax * 1000.0, 20.0)});
        }
        {
            Scenario sc;
            sc.grab = "hip";
            sc.scale = 0.54f;
            sc.path = pullPath(glm::vec3(0.25f, 0.0f, 0.0f), 80, 150);
            const RunResult r = run(sc);
            report.phase("[real] child walk (0.54 scale, hip +25cm x)",
                         {gateMin("steps taken", r.stepsTaken, 2.0),
                          gateMax("hip-to-target at rest (mm)", r.restingMiss * 1000.0, 15.0)});
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
        report.phase("[real] hold still (no cursor motion)",
                     {gateMax("worst joint drift (mm)", worst * 1000.0, 2.0)});
    }
}

} // namespace

// --custom <bone> <dx> <dy> <dz> <moveTicks> <holdTicks> [pin ...]: one ad-hoc scenario, fully
// reported (verbose), for investigating a behaviour before it becomes a gated phase.
struct CustomSpec {
    bool     set = false;
    Scenario scenario;
};

int main(int argc, char** argv) {
    Report report;
    std::string skeletonPath;
    CustomSpec custom;
    if (const char* env = std::getenv("POSESTUDIO_IK_SKELETON")) {
        skeletonPath = env;
    }
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--verbose") == 0 || std::strcmp(argv[i], "-v") == 0) {
            report.verbose = true;
        } else if (std::strcmp(argv[i], "--only") == 0 && i + 1 < argc) {
            report.only = argv[++i];
        } else if (std::strcmp(argv[i], "--custom") == 0 && i + 6 < argc) {
            custom.set = true;
            custom.scenario.grab = argv[i + 1];
            const glm::vec3 offset(static_cast<float>(std::atof(argv[i + 2])),
                                   static_cast<float>(std::atof(argv[i + 3])),
                                   static_cast<float>(std::atof(argv[i + 4])));
            custom.scenario.path = pullPath(offset, std::atoi(argv[i + 5]), std::atoi(argv[i + 6]));
            i += 6;
            while (i + 1 < argc && argv[i + 1][0] != '-') {
                custom.scenario.userPins.emplace_back(argv[++i]);
            }
            report.only = "custom";
            report.verbose = true;
        } else if (std::strcmp(argv[i], "--pose") == 0 && i + 4 < argc) {
            // --pose <bone> <x> <y> <z>: an FK pre-pose (Euler degrees) for --custom; repeatable.
            custom.scenario.prePose.emplace_back(
                argv[i + 1], glm::vec3(static_cast<float>(std::atof(argv[i + 2])),
                                       static_cast<float>(std::atof(argv[i + 3])),
                                       static_cast<float>(std::atof(argv[i + 4]))));
            i += 4;
        } else if (std::strcmp(argv[i], "--hover") == 0 && i + 1 < argc) {
            custom.scenario.hover = static_cast<float>(std::atof(argv[++i]));
        } else if (std::strcmp(argv[i], "--scale") == 0 && i + 1 < argc) {
            custom.scenario.scale = static_cast<float>(std::atof(argv[++i]));
        } else {
            skeletonPath = argv[i];
        }
    }

    if (report.only.empty() || report.only.find("unit") != std::string::npos) {
        testEulerRoundTrip(report);
        testConstraints(report);
        testSkeletonGraph(report);
        testBalance(report);
    }

    std::vector<ArmatureBone> bones;
    if (skeletonPath.empty()) {
        report.skip("[real] figure phases", "no skeleton dump: pass a .skel path or set POSESTUDIO_IK_SKELETON");
    } else if (!Armature::loadDump(skeletonPath, bones) || bones.empty()) {
        report.skip("[real] figure phases", ("could not read " + skeletonPath).c_str());
        ++report.failedPhases;
    } else {
        std::printf("[info] skeleton %s: %zu bones\n", skeletonPath.c_str(), bones.size());
        if (custom.set) {
            const RunResult r = runScenario(bones, custom.scenario, true);
            std::vector<Gate> gates{info("grab start y (m)", r.grabStart.y),
                                    info("grab start z (m)", r.grabStart.z),
                                    info("hip drop at release (m)", r.hipDropAtRelease),
                                    info("hip displacement (m)", r.hipDisp),
                                    info("head displacement (m)", r.headDisp),
                                    info("chest displacement (m)", r.chestDisp),
                                    info("grab rise (m)", r.grabEnd.y - r.grabStart.y),
                                    info("grab-to-cursor at rest (mm)", r.restingMiss * 1000.0),
                                    info("contact drift, drag (mm)", r.contactDriftMaxDrag * 1000.0),
                                    info("contact drift, after (mm)", r.contactDriftMax * 1000.0),
                                    info("lowest contact vs floor (mm)", (r.contactMinY - r.contactBindMinY) * 1000.0),
                                    info("floor penetration (mm)", r.penetrationMax * 1000.0),
                                    info("feet min y at the end (m)", r.feetMinY),
                                    info("steps", r.stepsTaken),
                                    info("suspended", r.suspended ? 1.0 : 0.0),
                                    info("user pin max distance (mm)", r.userPinDistMax * 1000.0)};
            if (!r.ok) {
                gates.push_back(gateMin(("error: " + r.error).c_str(), 0.0, 1.0));
            }
            if (r.penetrationNode >= 0) {
                std::printf("         worst floor penetration: %s\n",
                            bones[static_cast<std::size_t>(r.penetrationNode)].name.c_str());
            }
            report.phase("[custom] " + custom.scenario.grab, gates);
        } else {
            realPhases(report, bones);
        }
    }

    std::printf("\n%d phase(s): %d failed, %d skipped\n", report.phases, report.failedPhases,
                report.skippedPhases);
    return report.failedPhases == 0 ? 0 : 1;
}
