/**
 * @file simulator.cpp
 * @brief The drag-loop simulator's implementation (see simulator.h): the scripted cursor mirrors
 *        VulkanWindow::benchAdvance, and every per-tick measurement is taken from the Armature's
 *        world positions exactly as the viewport would render them.
 */

#include "simulator.h"

#include "armature.h"
#include "cursorfilter.h"
#include "fabriksolver.h"
#include "ikrig.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

using namespace pose;

namespace ikharness {

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
    // One scaled copy for the whole run (a child-scale scenario); the base list otherwise.
    const std::vector<ArmatureBone> scaled =
        sc.scale == 1.0f ? std::vector<ArmatureBone>{} : scaledBones(baseBones, sc.scale);
    const std::vector<ArmatureBone>& bones = sc.scale == 1.0f ? baseBones : scaled;
    arm.build(bones);
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
            y += bones[static_cast<std::size_t>(cur)].localBindTranslation.y;
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
    std::vector<glm::vec3> prevPos(idle.size()), prevStep(idle.size(), glm::vec3(0.0f));
    std::vector<double> osc(idle.size(), 0.0);
    for (std::size_t k = 0; k < idle.size(); ++k) {
        prevPos[k] = arm.boneWorldPosition(static_cast<std::size_t>(idle[k]));
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
    }
    arm.endIkDrag();
    return m;
}

std::vector<Waypoint> pullPath(const glm::vec3& offset, int moveTicks, int holdTicks) {
    return {{0, glm::vec3(0.0f), "start"}, {moveTicks, offset, "move"}, {moveTicks + holdTicks, offset, "hold"}};
}

} // namespace ikharness
