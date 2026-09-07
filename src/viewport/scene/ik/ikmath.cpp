#include "ikmath.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>

namespace pose {

glm::quat shortestArc(const glm::vec3& from, const glm::vec3& to) {
    // Guarded on the CROSS length, not the dot: the inputs ride through long chains of quaternion
    // products whose norms drift, so two IDENTICAL slightly-non-unit vectors can have a dot just
    // below any (1 - eps) parallel test while their cross is exactly zero — an acos/normalize
    // formulation then returns NaN, which poisons every frame downstream (this was the in-app
    // "figure flies off the screen" bug: NaN skin matrices). atan2(|cross|, dot) is also
    // scale-tolerant, so slightly-short inputs still give the right angle.
    const glm::vec3 c = glm::cross(from, to);
    const float cLen = glm::length(c);
    const float d = glm::dot(from, to);
    if (cLen < 1e-7f) {
        if (d >= 0.0f) {
            return glm::quat(1.0f, 0.0f, 0.0f, 0.0f); // aligned (or degenerate): identity
        }
        // Antiparallel: 180 degrees about any axis perpendicular to `from`.
        glm::vec3 axis = glm::cross(glm::vec3(1.0f, 0.0f, 0.0f), from);
        if (glm::dot(axis, axis) < 1e-8f) {
            axis = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), from);
        }
        return glm::angleAxis(3.14159265f, glm::normalize(axis));
    }
    return glm::angleAxis(std::atan2(cLen, d), c / cLen);
}

float signedAngleAround(const glm::vec3& from, const glm::vec3& to, const glm::vec3& axis) {
    const glm::vec3 f = from - axis * glm::dot(from, axis);
    const glm::vec3 t = to - axis * glm::dot(to, axis);
    if (glm::dot(f, f) < 1e-10f || glm::dot(t, t) < 1e-10f) {
        return 0.0f; // a vector is (nearly) parallel to the axis — no measurable angle
    }
    return std::atan2(glm::dot(glm::cross(f, t), axis), glm::dot(f, t));
}

namespace {

int axisIndex(char c) {
    if (c == 'X' || c == 'x') return 0;
    if (c == 'Y' || c == 'y') return 1;
    if (c == 'Z' || c == 'z') return 2;
    return -1;
}

} // namespace

glm::mat4 eulerMatrix(const glm::vec3& degrees, const std::string& order) {
    glm::mat4 m(1.0f);
    for (const char c : order) {
        const int axis = axisIndex(c);
        if (axis < 0) {
            continue;
        }
        glm::vec3 axisVec(0.0f);
        axisVec[axis] = 1.0f;
        m = m * glm::rotate(glm::mat4(1.0f), glm::radians(degrees[axis]), axisVec);
    }
    return m;
}

glm::vec3 eulerFromMatrix(const glm::mat3& m, const std::string& order) {
    // Axis indices of the three applied rotations, M = Ri(a) · Rj(b) · Rk(c).
    int i = 0, j = 1, k = 2;
    if (order.size() >= 3) {
        const int oi = axisIndex(order[0]);
        const int oj = axisIndex(order[1]);
        const int ok = axisIndex(order[2]);
        if (oi >= 0 && oj >= 0 && ok >= 0 && oi != oj && oj != ok && oi != ok) {
            i = oi;
            j = oj;
            k = ok;
        }
    }
    // Parity: +1 for cyclic orders (XYZ, YZX, ZXY), -1 for the anticyclic ones. For
    // M = Ri(a)·Rj(b)·Rk(c) (right-handed principal rotations), the row-major element
    // M[i][k] = eps·sin(b); GLM is column-major, so row r / col c reads m[c][r].
    const float eps = (j == (i + 1) % 3) ? 1.0f : -1.0f;
    const float sb = glm::clamp(eps * m[k][i], -1.0f, 1.0f);
    // cos(b) from the rest of row i (cb·cc, ±cb·sc): far better conditioned near |b| = 90 than
    // asin alone, whose derivative blows up as sb -> ±1.
    const float cb = std::sqrt(m[i][i] * m[i][i] + m[j][i] * m[j][i]);
    const float b = std::atan2(sb, cb);
    float a = 0.0f;
    float c = 0.0f;
    if (cb > 1e-5f) {
        a = std::atan2(-eps * m[k][j], m[k][k]); // rows j,k of column k
        c = std::atan2(-eps * m[j][i], m[i][i]); // columns j,i of row i
    } else {
        // Gimbal lock (|b| = 90 deg): only a±c is determined. Fold c into a (c = 0); the row-major
        // elements M[j][i] / M[j][j] read sin/cos of that combined angle for every order.
        const float t = std::atan2(m[i][j], m[j][j]);
        a = (b > 0.0f) ? t : -t;
    }
    glm::vec3 outDegrees(0.0f);
    outDegrees[i] = glm::degrees(a);
    outDegrees[j] = glm::degrees(b);
    outDegrees[k] = glm::degrees(c);
    return outDegrees;
}

} // namespace pose
