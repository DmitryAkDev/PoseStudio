/**
 * @file unitphases.cpp
 * @brief The unit phases (see unitphases.h): synthetic inputs, exact expectations.
 */

#include "unitphases.h"

#include "balancecontroller.h"
#include "ikconstraints.h"
#include "ikmath.h"
#include "report.h"
#include "skeletongraph.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

using namespace pose;

namespace ikharness {

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

} // namespace ikharness
