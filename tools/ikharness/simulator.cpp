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
#include <cstdlib>
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

namespace {

// Canonical (G8-generation) bone name -> the other generations' names for the same joint.
// Filled from real skeleton dumps; a name absent here resolves only to itself.
struct BoneAlias {
    const char* canonical;
    const char* alternates[4];
};
const BoneAlias kBoneAliases[] = {
    {"lHand", {"l_hand"}},
    {"rHand", {"r_hand"}},
    {"lFoot", {"l_foot"}},
    {"rFoot", {"r_foot"}},
    {"lShin", {"l_shin"}},
    {"rShin", {"r_shin"}},
    {"lThigh", {"l_thigh"}},
    {"rThigh", {"r_thigh"}},
    // The newest generation's thigh twist bones hang OFF the chain (siblings of the shin); the
    // oldest generations have none — the leg diagnostics then skip the twist metric.
    {"lThighTwist", {"l_thightwist1"}},
    {"rThighTwist", {"r_thightwist1"}},
    {"lForeArm", {"lForearmBend", "l_forearm"}},
    {"rForeArm", {"rForearmBend", "r_forearm"}},
    {"lShldr", {"lShldrBend", "l_upperarm"}},
    {"rShldr", {"rShldrBend", "r_upperarm"}},
    {"lCollar", {"l_shoulder"}},
    {"rCollar", {"r_shoulder"}},
    {"lIndex3", {"l_index3"}},
    {"lEye", {"l_eye"}},
    {"head", {"head"}},
    {"neck", {"neckLower", "neck1"}},
    {"hip", {"hip"}},
    {"abdomenLower", {"abdomen", "spine1"}},
    {"chest", {"chestLower", "spine3"}},
    {"chest_2", {"chestUpper", "spine4", "chest"}}, // the two-bone-chest generations: the top
};

} // namespace

int resolveBone(const Armature& arm, const std::string& canonical) {
    int idx = arm.boneIndex(canonical);
    if (idx >= 0) {
        return idx;
    }
    for (const BoneAlias& alias : kBoneAliases) {
        if (canonical != alias.canonical) {
            continue;
        }
        for (const char* alt : alias.alternates) {
            if (alt != nullptr && (idx = arm.boneIndex(alt)) >= 0) {
                return idx;
            }
        }
    }
    return -1;
}

std::string resolveBoneName(const Armature& arm, const std::string& canonical) {
    const int idx = resolveBone(arm, canonical);
    return idx >= 0 ? arm.boneName(static_cast<std::size_t>(idx)) : canonical;
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
        if (!arm.setBoneRotation(resolveBoneName(arm, bone), euler)) {
            r.error = "pre-pose bone not found: " + bone;
            return r;
        }
    }
    if (sc.hover != 0.0f) {
        auto pose = arm.capturePose();
        // The rig root is the first multi-child descendant of the anatomical root (the hip on
        // real figures); the hover goes on the hip's pose translation like the engine writes it.
        int hip = resolveBone(arm, "hip");
        if (hip < 0) {
            hip = 0;
        }
        pose.emplace_back("@trans:" + arm.boneName(static_cast<std::size_t>(hip)),
                          glm::vec3(0.0f, sc.hover, 0.0f));
        arm.applyPose(pose);
    }
    for (const std::string& pin : sc.userPins) {
        const int idx = resolveBone(arm, pin);
        if (idx < 0) {
            r.error = "pin bone not found: " + pin;
            return r;
        }
        arm.setSelectedBone(idx);
        arm.togglePinSelectedBone();
    }
    r.grabIndex = resolveBone(arm, sc.grab);
    if (r.grabIndex >= 0) {
        arm.setSelectedBone(r.grabIndex);
    }
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
    r.grabEulerStart = arm.boneEuler(static_cast<std::size_t>(r.grabIndex));
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
        const int idx = resolveBone(arm, pin);
        userPinIdx.push_back(idx);
        userPinPos.push_back(arm.boneWorldPosition(static_cast<std::size_t>(idx)));
        userPinRot.push_back(glm::mat3(arm.poseGlobal(static_cast<std::size_t>(idx))));
    }
    const int hip = resolveBone(arm, "hip");
    const int head = resolveBone(arm, "head");
    const int chest = resolveBone(arm, "chest");
    // The feet, for the suspension / floor checks; the hands, for the live-contact phases.
    std::vector<int> feet;
    for (const char* f : {"lFoot", "rFoot"}) {
        const int idx = resolveBone(arm, f);
        if (idx >= 0) {
            feet.push_back(idx);
        }
    }
    std::vector<int> hands;
    for (const char* h : {"lHand", "rHand"}) {
        const int idx = resolveBone(arm, h);
        if (idx >= 0) {
            hands.push_back(idx);
        }
    }
    const auto handsNow = [&]() {
        double y = 1e9;
        for (const int h : hands) {
            y = std::min(y, static_cast<double>(arm.boneWorldPosition(static_cast<std::size_t>(h)).y));
        }
        return y;
    };
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
    // Leg diagnostics (see RunResult): each leg's hip socket, thigh twist bone (its one free
    // channel = the widest authored range), knee and ankle; plus each contact-pinned foot's
    // drag-start rotation.
    struct Leg {
        int       socket = -1, twist = -1, knee = -1, ankle = -1, twistAxis = 1;
        float     twistStart = 0.0f;
        glm::vec3 prevKnee{0.0f}, prevKneeStep{0.0f};
        double    osc = 0.0;
    };
    std::vector<Leg> legs;
    for (const char* side : {"l", "r"}) {
        Leg leg;
        const std::string s(side);
        leg.socket = resolveBone(arm, s + "Thigh");
        leg.twist = resolveBone(arm, s + "ThighTwist");
        leg.knee = resolveBone(arm, s + "Shin");
        leg.ankle = resolveBone(arm, s + "Foot");
        if (leg.socket < 0 || leg.knee < 0 || leg.ankle < 0) {
            continue;
        }
        if (leg.twist >= 0) {
            const ArmatureBone& tb = bones[static_cast<std::size_t>(leg.twist)];
            float widest = -1.0f;
            for (int a = 0; a < 3; ++a) {
                const float range = tb.rotationLimited[a] ? tb.rotationMax[a] - tb.rotationMin[a] : 360.0f;
                if (range > widest) {
                    widest = range;
                    leg.twistAxis = a;
                }
            }
            leg.twistStart = arm.boneEuler(static_cast<std::size_t>(leg.twist))[leg.twistAxis];
        }
        leg.prevKnee = arm.boneWorldPosition(static_cast<std::size_t>(leg.knee));
        legs.push_back(leg);
    }
    const auto wrapDeg = [](float d) {
        while (d > 180.0f) d -= 360.0f;
        while (d <= -180.0f) d += 360.0f;
        return d;
    };
    const auto kneeTwist = [&](const Leg& leg) {
        if (leg.twist < 0) {
            return 0.0; // a generation without thigh twist bones: no twist channel to read
        }
        return static_cast<double>(std::abs(wrapDeg(
            arm.boneEuler(static_cast<std::size_t>(leg.twist))[leg.twistAxis] - leg.twistStart)));
    };
    const auto kneeInward = [&](const Leg& leg) {
        const glm::vec3 socket = arm.boneWorldPosition(static_cast<std::size_t>(leg.socket));
        const glm::vec3 ankle = arm.boneWorldPosition(static_cast<std::size_t>(leg.ankle));
        const glm::vec3 knee = arm.boneWorldPosition(static_cast<std::size_t>(leg.knee));
        glm::vec3 axis = ankle - socket;
        const float len = glm::length(axis);
        if (len < 1e-4f) {
            return 0.0;
        }
        axis /= len;
        const glm::vec3 off = (knee - socket) - axis * glm::dot(knee - socket, axis);
        glm::vec3 inward(0.0f); // toward the other leg's socket, perpendicular to the line
        for (const Leg& other : legs) {
            if (other.socket != leg.socket) {
                inward = arm.boneWorldPosition(static_cast<std::size_t>(other.socket)) - socket;
            }
        }
        inward -= axis * glm::dot(inward, axis);
        const float il = glm::length(inward);
        return il > 1e-4f ? static_cast<double>(glm::dot(off, inward / il)) : 0.0;
    };
    std::vector<glm::mat3> contactStartRot;
    for (const int c : r.contactPins) {
        contactStartRot.push_back(glm::mat3(arm.poseGlobal(static_cast<std::size_t>(c))));
    }
    const auto footRotNow = [&]() {
        double worst = 0.0;
        for (std::size_t c = 0; c < r.contactPins.size(); ++c) {
            worst = std::max(worst, rotationAngleDeg(contactStartRot[c],
                                                     glm::mat3(arm.poseGlobal(static_cast<std::size_t>(r.contactPins[c])))));
        }
        return worst;
    };
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
        for (const auto& [nudgeTick, delta] : sc.nudges) {
            if (nudgeTick == tick) {
                arm.nudgeSelectedBone(delta); // the X/Y/Z wheel mid-drag (the window's path)
            }
        }
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
        // IK_HARNESS_POSE_TRACE=1 also dumps the pose at the first few ticks any joint jumps
        // more than 3cm — a basin flip caught in the act.
        static const bool kJumpTrace = std::getenv("IK_HARNESS_POSE_TRACE") != nullptr;
        static int jumpDumps = 0;
        double tickJump = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            const glm::vec3 p = arm.boneWorldPosition(i);
            const double step = glm::length(p - prev[i]);
            tickJump = std::max(tickJump, step);
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
        if (kJumpTrace && tickJump > 0.03 && jumpDumps < 6) {
            ++jumpDumps;
            std::printf("[pose] tick %d: a joint jumped %.1f mm; cursor(%.3f %.3f %.3f) grab(%.3f %.3f %.3f)\n",
                        tick, tickJump * 1000.0, raw.x, raw.y, raw.z, eff.x, eff.y, eff.z);
            for (std::size_t i = 0; i < n; ++i) {
                const glm::vec3& e = arm.boneEuler(i);
                if (std::max({std::abs(e.x), std::abs(e.y), std::abs(e.z)}) > 3.0f) {
                    std::printf("[pose]   %-18s euler(%7.1f %7.1f %7.1f) step %.1f mm\n",
                                arm.boneName(i).c_str(), e.x, e.y, e.z,
                                glm::length(arm.boneWorldPosition(i) - prev[i]) * 1000.0);
                }
            }
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
        {
            const std::vector<IkEffector>& pins = rig->pins();
            r.pinsMax = std::max(r.pinsMax, static_cast<int>(pins.size()));
            int live = 0;
            for (std::size_t p = 0; p < pins.size(); ++p) {
                if (!rig->pinIsLive(p)) {
                    continue;
                }
                ++live;
                const glm::vec3 pos(arm.poseGlobal(static_cast<std::size_t>(pins[p].node))[3]);
                r.livePinSlideMax = std::max(
                    r.livePinSlideMax,
                    static_cast<double>(glm::length(
                        glm::vec2(pos.x - pins[p].target.x, pos.z - pins[p].target.z))));
            }
            r.livePinsMax = std::max(r.livePinsMax, live);
        }
        r.handsMinY = std::min(r.handsMinY, handsNow());
        // Leg diagnostics per drag tick (IK_HARNESS_LEG_TRACE=1 prints them).
        static const bool kLegTrace = std::getenv("IK_HARNESS_LEG_TRACE") != nullptr;
        for (Leg& leg : legs) {
            r.kneeTwistMaxDeg = std::max(r.kneeTwistMaxDeg, kneeTwist(leg));
            r.kneeInwardMax = std::max(r.kneeInwardMax, kneeInward(leg));
            const glm::vec3 knee = arm.boneWorldPosition(static_cast<std::size_t>(leg.knee));
            if (kLegTrace) {
                // ... plus the pelvis bone's Euler and the hip's world position, so a knee
                // alternation can be traced to the pelvis tilt or to the leg itself.
                const int pelvisBone = resolveBone(arm, "pelvis");
                const glm::vec3 pe = pelvisBone >= 0 ? arm.boneEuler(static_cast<std::size_t>(pelvisBone))
                                                      : glm::vec3(0.0f);
                const glm::vec3 hipPos = hip >= 0 ? arm.boneWorldPosition(static_cast<std::size_t>(hip))
                                                  : glm::vec3(0.0f);
                std::printf("[leg] tick %d %s knee(%.4f %.4f %.4f) twist %.2f inward %.4f pelvis(%.2f %.2f %.2f) hip(%.4f %.4f %.4f)\n",
                            tick, arm.boneName(static_cast<std::size_t>(leg.knee)).c_str(), knee.x,
                            knee.y, knee.z, kneeTwist(leg), kneeInward(leg), pe.x, pe.y, pe.z, hipPos.x,
                            hipPos.y, hipPos.z);
            }
            const glm::vec3 kneeStep = knee - leg.prevKnee;
            const float kl = glm::length(kneeStep);
            r.kneeStepMax = std::max(r.kneeStepMax, static_cast<double>(kl));
            const float pl = glm::length(leg.prevKneeStep);
            if (pl > 1e-6f && kl > 1e-6f) {
                const float against = -glm::dot(kneeStep, leg.prevKneeStep) / pl;
                if (against > 0.0f) {
                    leg.osc += std::min(static_cast<double>(against), static_cast<double>(pl));
                }
            }
            leg.prevKneeStep = kneeStep;
            leg.prevKnee = knee;
        }
        r.footRotMaxDeg = std::max(r.footRotMaxDeg, footRotNow());
    }
    for (const Leg& leg : legs) {
        r.kneeOscMax = std::max(r.kneeOscMax, leg.osc);
    }
    // (Idle-joint tremble metrics come from runIdleMetrics — a second, identical run that keeps
    // the per-joint step history; the main loop stays a plain mirror of the viewport's tick.)
    r.grabEnd = arm.boneWorldPosition(static_cast<std::size_t>(r.grabIndex));
    r.cursorEnd = raw;
    r.pinsAtRelease = static_cast<int>(rig->pins().size());
    // IK_HARNESS_POSE_TRACE=1: the joints the drag rotated most (Euler degrees) and any pose
    // translation, at mouse-up — which part of the body served the gesture.
    static const bool kPoseTrace = std::getenv("IK_HARNESS_POSE_TRACE") != nullptr;
    if (kPoseTrace) {
        std::vector<std::pair<float, std::size_t>> byMagnitude;
        for (std::size_t i = 0; i < n; ++i) {
            const glm::vec3& e = arm.boneEuler(i);
            const float mag = std::max({std::abs(e.x), std::abs(e.y), std::abs(e.z)});
            if (mag > 3.0f || glm::length(arm.boneTranslation(i)) > 0.005f) {
                byMagnitude.emplace_back(mag, i);
            }
        }
        std::sort(byMagnitude.begin(), byMagnitude.end(), std::greater<>());
        std::printf("[pose] at mouse-up, %zu joints past 3 deg:\n", byMagnitude.size());
        for (std::size_t k = 0; k < byMagnitude.size() && k < 24; ++k) {
            const std::size_t i = byMagnitude[k].second;
            const glm::vec3& e = arm.boneEuler(i);
            const glm::vec3& t = arm.boneTranslation(i);
            std::printf("[pose]   %-18s euler(%7.1f %7.1f %7.1f) trans(%.3f %.3f %.3f)\n",
                        arm.boneName(i).c_str(), e.x, e.y, e.z, t.x, t.y, t.z);
        }
    }
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
            r.footRotMaxDeg = std::max(r.footRotMaxDeg, footRotNow());
            r.handsMinY = std::min(r.handsMinY, handsNow());
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
    r.grabEulerEnd = arm.boneEuler(static_cast<std::size_t>(r.grabIndex));
    for (const Leg& leg : legs) {
        r.kneeTwistEndDeg = std::max(r.kneeTwistEndDeg, kneeTwist(leg));
        r.kneeInwardEnd = std::max(r.kneeInwardEnd, kneeInward(leg));
    }
    if (r.kneeInwardMax < -1e8) {
        r.kneeInwardMax = 0.0;
    }
    r.footRotEndDeg = footRotNow();
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
    r.handsEndY = handsNow();
    for (const Leg& leg : legs) {
        r.kneesEndY = std::min(r.kneesEndY, static_cast<double>(r.endPos[static_cast<std::size_t>(leg.knee)].y));
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
        arm.setBoneRotation(resolveBoneName(arm, bone), euler);
    }
    for (const std::string& pin : sc.userPins) {
        const int idx = resolveBone(arm, pin);
        if (idx >= 0) {
            arm.setSelectedBone(idx);
            arm.togglePinSelectedBone();
        }
    }
    const int grab = resolveBone(arm, sc.grab);
    if (grab >= 0) {
        arm.setSelectedBone(grab);
    }
    if (grab < 0 || !arm.beginIkDrag()) {
        return m;
    }
    std::vector<int> idle;
    for (const std::string& name : sc.idleJoints) {
        const int idx = resolveBone(arm, name);
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
        for (const auto& [nudgeTick, delta] : sc.nudges) {
            if (nudgeTick == tick) {
                arm.nudgeSelectedBone(delta); // mirror the main pass's wheel nudges
            }
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
