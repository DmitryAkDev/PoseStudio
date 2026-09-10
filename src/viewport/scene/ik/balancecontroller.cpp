/**
 * @file balancecontroller.cpp
 * @brief The balance controller's mass model, center of mass, support polygon and correction.
 *
 * Segment masses come from a tolerant NAME-FRAGMENT table (biomechanical fractions; same-class
 * same-side bones split a share so a Bend+Twist pair is one thigh; unclassified bones get a token
 * mass so they perturb, never dominate) — a rig whose names match nothing degrades to token
 * masses everywhere. The support polygon is Andrew's monotone-chain hull of the contacts in XZ;
 * degenerate one- and two-point hulls get a foot-area slack radius (kDegenerateSupportRadius) so
 * point-feet never demand exact-line balance and the controller does not fight the pins forever.
 * Qt-free (std + GLM).
 */
#include "balancecontroller.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iterator>

namespace pose {

namespace {

// One body-segment class: name fragments that identify it (checked in table order, first hit
// wins) and its share of total body mass (biomechanical segment fractions, Winter/Dempster-ish).
// Same-class same-side bones split the share (a ThighBend+ThighTwist pair is still one thigh).
struct SegmentClass {
    const char* fragment;
    float       massFraction;
};

// Order matters: more specific fragments before the substrings they contain.
constexpr SegmentClass kSegmentTable[] = {
    {"forearm", 0.016f},   {"upperarm", 0.028f}, {"shldr", 0.028f},  {"shoulder", 0.028f},
    {"collar", 0.021f},    {"hand", 0.006f},     {"thigh", 0.100f},  {"shin", 0.0465f},
    {"calf", 0.0465f},     {"metatarsal", 0.005f}, {"toe", 0.004f},  {"foot", 0.0137f},
    {"head", 0.081f},      {"neck", 0.012f},     {"chest", 0.170f},  {"abdomen", 0.100f},
    {"spine", 0.100f},     {"pelvis", 0.112f},   {"hip", 0.050f},
};

constexpr float kUnclassifiedMass = 0.0015f; // face/finger/helper bones: perturb, don't dominate

// Degenerate support hulls (one contact point, or two — a line segment between point-feet) get
// this much slack radius: real feet have AREA, so demanding the CoM sit exactly on the contact
// point/line would make the balance controller correct forever and fight the drag/pins.
constexpr float kDegenerateSupportRadius = 0.05f;

std::string normalizedName(const std::string& name) {
    std::string out;
    out.reserve(name.size());
    for (const char c : name) {
        if (std::isalpha(static_cast<unsigned char>(c))) {
            out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
        }
    }
    return out;
}

// Side prefix for share-splitting: 'l'/'r' when the name starts with one (lShin, r_foot,
// leftFoot), 'c' (center) otherwise.
char sideOf(const std::string& normalized) {
    if (normalized.rfind("left", 0) == 0) return 'l';
    if (normalized.rfind("right", 0) == 0) return 'r';
    if (!normalized.empty() && (normalized[0] == 'l' || normalized[0] == 'r')) {
        return normalized[0];
    }
    return 'c';
}

int classify(const std::string& normalized) {
    for (std::size_t t = 0; t < std::size(kSegmentTable); ++t) {
        if (normalized.find(kSegmentTable[t].fragment) != std::string::npos) {
            return static_cast<int>(t);
        }
    }
    return -1;
}

float cross2(const glm::vec2& o, const glm::vec2& a, const glm::vec2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

glm::vec2 closestOnSegment(const glm::vec2& a, const glm::vec2& b, const glm::vec2& p) {
    const glm::vec2 ab = b - a;
    const float len2 = glm::dot(ab, ab);
    if (len2 < 1e-12f) {
        return a;
    }
    const float t = glm::clamp(glm::dot(p - a, ab) / len2, 0.0f, 1.0f);
    return a + ab * t;
}

} // namespace

std::vector<float> BalanceController::assignMasses(const std::vector<std::string>& names) {
    const std::size_t n = names.size();
    std::vector<float> masses(n, kUnclassifiedMass);
    std::vector<int>   classOf(n, -1);

    // Count how many bones share each (class, side) so they split the segment's mass share.
    // Key = class * 4 + side slot.
    std::vector<int> shareCount(std::size(kSegmentTable) * 4, 0);
    auto slotOf = [](char side) { return side == 'l' ? 0 : (side == 'r' ? 1 : 2); };
    std::vector<char> side(n, 'c');
    for (std::size_t i = 0; i < n; ++i) {
        const std::string norm = normalizedName(names[i]);
        classOf[i] = classify(norm);
        side[i] = sideOf(norm);
        if (classOf[i] >= 0) {
            ++shareCount[static_cast<std::size_t>(classOf[i]) * 4 +
                         static_cast<std::size_t>(slotOf(side[i]))];
        }
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (classOf[i] >= 0) {
            const int count = shareCount[static_cast<std::size_t>(classOf[i]) * 4 +
                                         static_cast<std::size_t>(slotOf(side[i]))];
            masses[i] = kSegmentTable[classOf[i]].massFraction / static_cast<float>(std::max(1, count));
        }
    }
    return masses;
}

glm::vec3 BalanceController::centerOfMass(const std::vector<glm::vec3>& positions,
                                          const std::vector<int>& parents,
                                          const std::vector<float>& masses) {
    const std::size_t n = positions.size();
    if (n == 0 || masses.size() != n || parents.size() != n) {
        return glm::vec3(0.0f);
    }
    // Each bone's segment reaches to the mean of its anatomical children; accumulate child sums.
    std::vector<glm::vec3> childSum(n, glm::vec3(0.0f));
    std::vector<int>       childCount(n, 0);
    for (std::size_t i = 0; i < n; ++i) {
        const int p = parents[i];
        if (p >= 0 && static_cast<std::size_t>(p) < n) {
            childSum[static_cast<std::size_t>(p)] += positions[i];
            ++childCount[static_cast<std::size_t>(p)];
        }
    }
    glm::vec3 weighted(0.0f);
    float total = 0.0f;
    for (std::size_t i = 0; i < n; ++i) {
        const glm::vec3 segmentEnd =
            childCount[i] > 0 ? childSum[i] / static_cast<float>(childCount[i]) : positions[i];
        weighted += masses[i] * 0.5f * (positions[i] + segmentEnd);
        total += masses[i];
    }
    return total > 0.0f ? weighted / total : glm::vec3(0.0f);
}

std::vector<glm::vec2> BalanceController::supportPolygon(std::vector<glm::vec2> points) {
    if (points.size() < 3) {
        return points; // a point or a segment — closestBalancedPoint handles the degeneracy
    }
    // Andrew's monotone chain, counter-clockwise.
    std::sort(points.begin(), points.end(), [](const glm::vec2& a, const glm::vec2& b) {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    });
    points.erase(std::unique(points.begin(), points.end(),
                             [](const glm::vec2& a, const glm::vec2& b) {
                                 return glm::dot(a - b, a - b) < 1e-10f;
                             }),
                 points.end());
    if (points.size() < 3) {
        return points;
    }
    std::vector<glm::vec2> hull(points.size() * 2);
    std::size_t k = 0;
    for (const glm::vec2& p : points) { // lower hull
        while (k >= 2 && cross2(hull[k - 2], hull[k - 1], p) <= 0.0f) --k;
        hull[k++] = p;
    }
    const std::size_t lower = k + 1;
    for (std::size_t i = points.size() - 1; i-- > 0;) { // upper hull
        const glm::vec2& p = points[i];
        while (k >= lower && cross2(hull[k - 2], hull[k - 1], p) <= 0.0f) --k;
        hull[k++] = p;
    }
    hull.resize(k - 1); // last point == first
    return hull;
}

bool BalanceController::insidePolygon(const std::vector<glm::vec2>& hull, const glm::vec2& p) {
    if (hull.size() < 3) {
        return false;
    }
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const glm::vec2& a = hull[i];
        const glm::vec2& b = hull[(i + 1) % hull.size()];
        if (cross2(a, b, p) < -1e-7f) { // CCW hull: p must be left of (or on) every edge
            return false;
        }
    }
    return true;
}

glm::vec2 BalanceController::closestBalancedPoint(const std::vector<glm::vec2>& hull,
                                                  const glm::vec2& p, float margin) {
    if (hull.empty()) {
        return p;
    }
    if (hull.size() == 1) {
        return hull[0];
    }
    // Closest point on the boundary (a segment for 2 points, edges otherwise) and the centroid to
    // pull inward toward.
    glm::vec2 centroid(0.0f);
    for (const glm::vec2& v : hull) {
        centroid += v;
    }
    centroid /= static_cast<float>(hull.size());

    const bool inside = insidePolygon(hull, p);
    glm::vec2 best(0.0f);
    float bestDist2 = 1e30f;
    const std::size_t edges = hull.size() == 2 ? 1 : hull.size();
    for (std::size_t i = 0; i < edges; ++i) {
        const glm::vec2 c = closestOnSegment(hull[i], hull[(i + 1) % hull.size()], p);
        const float d2 = glm::dot(c - p, c - p);
        if (d2 < bestDist2) {
            bestDist2 = d2;
            best = c;
        }
    }
    if (inside && bestDist2 >= margin * margin) {
        return p; // safely inside: balanced as-is
    }
    // On/near/outside the boundary: come to the boundary point, inset toward the centroid.
    const glm::vec2 toCentroid = centroid - best;
    const float len = glm::length(toCentroid);
    return len > 1e-6f ? best + toCentroid * std::min(1.0f, margin / len) : best;
}

bool BalanceController::balanceCorrection(const std::vector<glm::vec3>& positions,
                                          const std::vector<int>& parents,
                                          const std::vector<float>& masses,
                                          const std::vector<glm::vec2>& hull, float margin,
                                          glm::vec2& correctionXZ) {
    correctionXZ = glm::vec2(0.0f);
    if (hull.empty()) {
        return false;
    }
    const glm::vec3 com = centerOfMass(positions, parents, masses);
    const glm::vec2 comXZ(com.x, com.z);
    const glm::vec2 target = closestBalancedPoint(hull, comXZ, margin);
    correctionXZ = target - comXZ;
    if (hull.size() < 3) {
        // Degenerate support (a point / the line between two point-feet): allow the foot-area
        // slack, and only correct the CoM to the edge of that band, not exactly onto the line.
        const float len = glm::length(correctionXZ);
        if (len <= kDegenerateSupportRadius) {
            correctionXZ = glm::vec2(0.0f);
            return false;
        }
        correctionXZ *= 1.0f - kDegenerateSupportRadius / len;
    }
    return glm::dot(correctionXZ, correctionXZ) > 1e-8f;
}

} // namespace pose
