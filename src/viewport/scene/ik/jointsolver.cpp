/**
 * @file jointsolver.cpp
 * @brief The whole-body joint-space IK solver (see jointsolver.h): forward kinematics in the
 *        Armature's exact composition, the analytic task Jacobian, and the Levenberg-Marquardt
 *        loop in its dual (rows x rows) form with an active set on the channel limits.
 *
 * One linearization = the weighted residual vector r and its Jacobian J over the problem's
 * DoFs. The step minimizes |J d - r|^2 + sum s_i (d_i - pull_i)^2 + mu sum t_i d_i^2, pull_i
 * being the distance back to the posture reference and t_i the damping metric (1 per rad^2,
 * 25 per m^2: 20cm of translation is damped like a radian); with D = diag(s_i + mu t_i) that is
 *     d = d0 + D^-1 J^T (J D^-1 J^T + I)^-1 (r - J d0),   d0 = D^-1 S pull,
 * one Cholesky the size of the ROW count. A step is accepted only if the true (nonlinear,
 * limit-projected) cost went down; otherwise mu grows tenfold and it is retried. The damping
 * is ADDITIVE, not a multiple of the stiffness: near a singular configuration (a straight
 * pinned leg asked to give height) the linear model buys millimetres with radians, the hard
 * rows turn that into metres of true error, and only a mu that rivals J^T J — 1e4 and up, far
 * above any posture stiffness — shortens such a step; scaled with the stiffness it never bit.
 * Qt-free (std + GLM).
 */

#include "jointsolver.h"

#include "balancecontroller.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace pose {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
/// The slack of a degenerate (one- or two-point) support, as in BalanceController.
constexpr double kDegenerateSupportRadius = 0.05;

int axisIndexOf(char c) {
    switch (c) {
    case 'X': case 'x': return 0;
    case 'Y': case 'y': return 1;
    case 'Z': case 'z': return 2;
    default: return -1;
    }
}

/// Right-handed rotation about principal axis @p axis — glm::rotate, as eulerMatrix composes.
glm::dmat3 axisRotation(int axis, double radians) {
    glm::dvec3 v(0.0);
    v[axis] = 1.0;
    return glm::dmat3(glm::rotate(glm::dmat4(1.0), radians, v));
}

/// The rotation vector (axis * angle, radians) of rotation matrix @p m.
glm::dvec3 rotationVector(const glm::dmat3& m) {
    glm::dquat q = glm::normalize(glm::quat_cast(m));
    if (q.w < 0.0) {
        q = -q;
    }
    const glm::dvec3 v(q.x, q.y, q.z);
    const double s = glm::length(v);
    if (s < 1e-12) {
        return 2.0 * v; // small angle: angle*axis ~= 2*v
    }
    return v * (2.0 * std::atan2(s, q.w) / s);
}

/// The PER-AXIS orientation error of a task whose frame is (lateral, up, fore): the bone's
/// lateral axis (frame[0] at the target) must keep its HEADING (the error about `up`) and stay
/// LEVEL (the error about `fore`), and the rotation ABOUT it — the pitch — is measured as an
/// angle. Returned as (pitch, heading, level), each the rotation that takes the current frame
/// to the target about that axis, radians; @p lateralNow is the bone's lateral axis now.
///
/// Swing/twist about the lateral axis, NOT the rotation vector split into components: that
/// split is only meaningful while the whole error is small. A planted foot's pitch is nearly
/// free (weight 20) beside a hard heading (4e6), and in a kneel the foot stands pitched 90
/// degrees from the orientation it is held to — where one degree of ROLL reads as 0.8 of a
/// degree of heading in the rotation vector, and the rows' small-error Jacobian predicted the
/// opposite of what a step did: the solve accepted only 0.0002-radian steps, the hip crawled
/// 10-18cm behind the cursor through every kneel and jumped 10cm when a step finally broke out.
glm::dvec3 lateralAxisError(const glm::dmat3& target, const glm::dmat3& current, const glm::dmat3& frame,
                            glm::dvec3* lateralNow) {
    const glm::dmat3 delta = current * glm::transpose(target); // takes the target frame to now
    const glm::dvec3 a = delta * frame[0];
    const glm::dvec3 swing = glm::cross(a, frame[0]); // small angle: the rotation vector, off-axis
    // The pitch: the fore axis now, in the plane across the TARGET lateral axis.
    glm::dvec3 fore = delta * frame[2];
    fore -= frame[0] * glm::dot(fore, frame[0]);
    double pitch = 0.0;
    if (glm::dot(fore, fore) > 1e-12) {
        pitch = std::atan2(glm::dot(glm::cross(fore, frame[2]), frame[0]), glm::dot(fore, frame[2]));
    }
    if (lateralNow != nullptr) {
        *lateralNow = a;
    }
    return glm::dvec3(pitch, glm::dot(swing, frame[1]), glm::dot(swing, frame[2]));
}

/// Solves the symmetric positive-definite system A x = b in place (A is n x n row-major,
/// destroyed; b becomes x). Returns false if a pivot fails (A not positive definite).
bool choleskySolve(std::vector<double>& A, std::vector<double>& b, int n) {
    for (int j = 0; j < n; ++j) {
        double d = A[static_cast<std::size_t>(j * n + j)];
        for (int k = 0; k < j; ++k) {
            const double l = A[static_cast<std::size_t>(j * n + k)];
            d -= l * l;
        }
        if (!(d > 1e-300)) {
            return false;
        }
        d = std::sqrt(d);
        A[static_cast<std::size_t>(j * n + j)] = d;
        for (int i = j + 1; i < n; ++i) {
            double v = A[static_cast<std::size_t>(i * n + j)];
            for (int k = 0; k < j; ++k) {
                v -= A[static_cast<std::size_t>(i * n + k)] * A[static_cast<std::size_t>(j * n + k)];
            }
            A[static_cast<std::size_t>(i * n + j)] = v / d;
        }
    }
    for (int i = 0; i < n; ++i) { // L y = b
        double v = b[static_cast<std::size_t>(i)];
        for (int k = 0; k < i; ++k) {
            v -= A[static_cast<std::size_t>(i * n + k)] * b[static_cast<std::size_t>(k)];
        }
        b[static_cast<std::size_t>(i)] = v / A[static_cast<std::size_t>(i * n + i)];
    }
    for (int i = n - 1; i >= 0; --i) { // L^T x = y
        double v = b[static_cast<std::size_t>(i)];
        for (int k = i + 1; k < n; ++k) {
            v -= A[static_cast<std::size_t>(k * n + i)] * b[static_cast<std::size_t>(k)];
        }
        b[static_cast<std::size_t>(i)] = v / A[static_cast<std::size_t>(i * n + i)];
    }
    return true;
}

/// A/B probe: IK_JS_OLD_ASK=1 bounds a position row's ask at its full weight, as it used to be
/// (see residuals()).
bool askKeepsGradient() {
    static const bool kOld = std::getenv("IK_JS_OLD_ASK") != nullptr;
    return !kOld;
}

double dofValue(const JointSolver::Pose& pose, const JointSolverDof& dof) {
    const std::size_t b = static_cast<std::size_t>(dof.bone);
    return dof.axis < 3 ? pose.eulerDeg[b][dof.axis] * kDegToRad : pose.translation[b][dof.axis - 3];
}

void setDofValue(JointSolver::Pose& pose, const JointSolverDof& dof, double value) {
    const std::size_t b = static_cast<std::size_t>(dof.bone);
    if (dof.axis < 3) {
        pose.eulerDeg[b][dof.axis] = value * kRadToDeg;
    } else {
        pose.translation[b][dof.axis - 3] = value;
    }
}

} // namespace

void JointSolver::setSkeleton(std::vector<JointSolverBone> bones) {
    m_bones = std::move(bones);
    const std::size_t n = m_bones.size();
    m_order.assign(n * 3, -1);
    m_children.assign(n, {});
    for (std::size_t i = 0; i < n; ++i) {
        const std::string& order = m_bones[i].rotationOrder;
        int slot = 0;
        for (const char c : order) {
            const int axis = axisIndexOf(c);
            if (axis >= 0 && slot < 3) {
                m_order[i * 3 + static_cast<std::size_t>(slot++)] = axis;
            }
        }
        const int p = m_bones[i].parent;
        if (p >= 0 && static_cast<std::size_t>(p) < n) {
            m_children[static_cast<std::size_t>(p)].push_back(static_cast<int>(i));
        }
    }
    // DFS numbering for the ancestor test (iterative: figures nest a hundred bones deep).
    m_tin.assign(n, 0);
    m_tout.assign(n, 0);
    int clock = 0;
    std::vector<std::pair<int, std::size_t>> stack;
    for (std::size_t root = 0; root < n; ++root) {
        if (m_bones[root].parent >= 0) {
            continue;
        }
        stack.push_back({static_cast<int>(root), 0});
        m_tin[root] = clock++;
        while (!stack.empty()) {
            auto& [node, next] = stack.back();
            const auto& kids = m_children[static_cast<std::size_t>(node)];
            if (next < kids.size()) {
                const int child = kids[next++];
                m_tin[static_cast<std::size_t>(child)] = clock++;
                stack.push_back({child, 0});
            } else {
                m_tout[static_cast<std::size_t>(node)] = clock;
                stack.pop_back();
            }
        }
    }
}

void JointSolver::forwardKinematics(const Pose& pose, Frames& frames) const {
    const std::size_t n = m_bones.size();
    frames.rot.resize(n);
    frames.pos.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        const JointSolverBone& b = m_bones[i];
        glm::dmat3 euler(1.0);
        for (int k = 0; k < 3; ++k) {
            const int axis = m_order[i * 3 + static_cast<std::size_t>(k)];
            if (axis >= 0) {
                euler = euler * axisRotation(axis, pose.eulerDeg[i][axis] * kDegToRad);
            }
        }
        const glm::dmat3 local = b.orient * euler * glm::transpose(b.orient);
        const glm::dvec3 offset = b.bindOffset + pose.translation[i];
        if (b.parent >= 0) {
            const std::size_t p = static_cast<std::size_t>(b.parent);
            frames.rot[i] = frames.rot[p] * local;
            frames.pos[i] = frames.pos[p] + frames.rot[p] * offset;
        } else {
            frames.rot[i] = local;
            frames.pos[i] = offset;
        }
    }
}

glm::dvec3 JointSolver::dofAxis(const Pose& pose, const Frames& frames,
                                const JointSolverDof& dof) const {
    const std::size_t i = static_cast<std::size_t>(dof.bone);
    const JointSolverBone& b = m_bones[i];
    const glm::dmat3 parentRot =
        b.parent >= 0 ? frames.rot[static_cast<std::size_t>(b.parent)] : glm::dmat3(1.0);
    glm::dvec3 unit(0.0);
    if (dof.axis >= 3) {
        unit[dof.axis - 3] = 1.0;
        return parentRot * unit;
    }
    // The channel's axis rides the rotations composed BEFORE it in the rotation order.
    glm::dmat3 prefix(1.0);
    bool found = false;
    for (int k = 0; k < 3; ++k) {
        const int axis = m_order[i * 3 + static_cast<std::size_t>(k)];
        if (axis < 0) {
            continue;
        }
        if (axis == dof.axis) {
            found = true;
            break;
        }
        prefix = prefix * axisRotation(axis, pose.eulerDeg[i][axis] * kDegToRad);
    }
    if (!found) {
        return glm::dvec3(0.0); // a channel the rotation order never applies
    }
    unit[dof.axis] = 1.0;
    return parentRot * b.orient * prefix * unit;
}

double JointSolver::postureCost(const Pose& pose, const Problem& problem) const {
    double cost = 0.0;
    for (const JointSolverDof& dof : problem.dofs) {
        const double ref = dof.axis < 3 ? dof.reference * kDegToRad : dof.reference;
        const double d = dofValue(pose, dof) - ref;
        cost += dof.stiffness * d * d;
    }
    return cost;
}

JointSolver::ComModel JointSolver::buildComModel(const Problem& problem) const {
    ComModel model;
    const std::size_t n = m_bones.size();
    model.coeff.assign(n, 0.0);
    if (problem.masses == nullptr || problem.masses->size() != n || problem.supportHull == nullptr ||
        problem.supportHull->empty() || problem.balanceWeight <= 0.0) {
        return model; // total 0 = the balance row is off
    }
    for (std::size_t i = 0; i < n; ++i) {
        const double m = (*problem.masses)[i];
        if (m <= 0.0) {
            continue;
        }
        model.total += m;
        const auto& kids = m_children[i];
        if (kids.empty()) {
            model.coeff[i] += m;
        } else {
            model.coeff[i] += 0.5 * m;
            const double share = 0.5 * m / static_cast<double>(kids.size());
            for (const int c : kids) {
                model.coeff[static_cast<std::size_t>(c)] += share;
            }
        }
    }
    return model;
}

double JointSolver::balanceNeed(const Frames& frames, const Problem& problem, const ComModel& com,
                                glm::dvec2& direction) const {
    direction = glm::dvec2(0.0);
    if (com.total <= 0.0) {
        return 0.0;
    }
    glm::dvec3 sum(0.0);
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        sum += com.coeff[i] * frames.pos[i];
    }
    glm::dvec3 c = sum / com.total;
    c.x += problem.comShift.x;
    c.z += problem.comShift.y;
    const glm::vec2 comXZ(static_cast<float>(c.x), static_cast<float>(c.z));
    const glm::vec2 balanced = BalanceController::closestBalancedPoint(
        *problem.supportHull, comXZ, static_cast<float>(problem.balanceMargin));
    glm::dvec2 need(static_cast<double>(balanced.x) - c.x, static_cast<double>(balanced.y) - c.z);
    if (glm::dot(problem.balanceAxis, problem.balanceAxis) > 0.5) {
        need = problem.balanceAxis * glm::dot(need, problem.balanceAxis);
    }
    double dist = glm::length(need);
    if (dist < 1e-7) {
        return 0.0;
    }
    direction = need / dist;
    if (problem.supportHull->size() < 3) {
        dist -= kDegenerateSupportRadius; // a point / a line of support: the foot-area slack
    }
    return std::max(dist - problem.balanceSlack, 0.0);
}

double JointSolver::residuals(const Pose& pose, const Frames& frames, const Problem& problem,
                              const std::vector<JointPlaneTask>& planes, const ComModel& com,
                              std::vector<double>& r, const Settings* stepBounds) const {
    // What a row ASKS of one linearized step: its miss, bounded (the cost is never bounded).
    const auto bounded = [stepBounds](double len, double Settings::*bound) {
        return (stepBounds != nullptr && len > stepBounds->*bound) ? stepBounds->*bound / len : 1.0;
    };
    const std::size_t rows = problem.positions.size() * 3 + problem.orientations.size() * 3 +
                             planes.size() + 1 + problem.couplings.size();
    r.assign(rows, 0.0);
    double cost = 0.0;
    std::size_t row = 0;
    for (const JointPositionTask& task : problem.positions) {
        const std::size_t tb = static_cast<std::size_t>(task.bone);
        const glm::dvec3 x = frames.pos[tb] + frames.rot[tb] * task.offset;
        const glm::dvec3 e = (task.target - x) * task.axisScale;
        const double len = glm::length(e);
        double w = task.weight;
        if (task.huber > 0.0 && len > task.huber) {
            w = task.weight * task.huber / len;
            cost += task.weight * task.huber * (2.0 * len - task.huber);
        } else {
            cost += task.weight * len * len;
        }
        // A bounded ASK must not be a weaker PULL: the row asks for k of its miss at 1/k of its
        // weight (r = sqrt(w/k) k e, J = sqrt(w/k) J_geo — see linearize), so its gradient J^T r is
        // the true cost's, w J^T e, whatever the bound. Bounded at its full weight (r = sqrt(w) k e)
        // the model's pull was k of the true one — nothing beside a hard row, which wins either
        // way, but the CURSOR's row is soft once its Huber pull saturates, and it rests against
        // the posture terms: held 30cm out of reach the model pulled a sixth as hard as the cost
        // it was descending. The model's resting pose lay well short of the cost's, every step
        // back toward it was rejected, every tick of such a hold ended in ten rejected dampings
        // (the "out-of-reach hold" tick cost), and the pose stood wherever it had been between
        // the two: a figure sat on the floor and leaned back by the chest stopped with her chest
        // 44cm over its cursor.
        const double k = bounded(len, &Settings::maxTaskStep);
        const double sw = std::sqrt(w) * (askKeepsGradient() ? std::sqrt(k) : k);
        r[row++] = e.x * sw;
        r[row++] = e.y * sw;
        r[row++] = e.z * sw;
    }
    for (const JointOrientationTask& task : problem.orientations) {
        const std::size_t tb = static_cast<std::size_t>(task.bone);
        // The weight matrix sum_k w_k a_k a_k^T, through its square root.
        glm::dvec3 weighted(0.0);
        double errorLen = 0.0;
        if (task.perAxis) {
            const glm::dvec3 err = lateralAxisError(task.target, frames.rot[tb], task.frame, nullptr);
            for (int k = 0; k < 3; ++k) {
                cost += task.axisWeights[k] * err[k] * err[k];
                weighted += task.frame[k] * (err[k] * std::sqrt(task.axisWeights[k]));
            }
            errorLen = glm::length(err);
        } else {
            const glm::dvec3 e = rotationVector(task.target * glm::transpose(frames.rot[tb]));
            cost += task.weight * glm::dot(e, e);
            weighted = e * std::sqrt(task.weight);
            errorLen = glm::length(e);
        }
        weighted *= bounded(errorLen, &Settings::maxOrientStep);
        r[row++] = weighted.x;
        r[row++] = weighted.y;
        r[row++] = weighted.z;
    }
    for (const JointPlaneTask& task : planes) {
        const std::size_t tb = static_cast<std::size_t>(task.bone);
        const glm::dvec3 x = frames.pos[tb] + frames.rot[tb] * task.offset;
        glm::dvec3 P = task.planePoint;
        glm::dvec3 nrm = task.normal;
        if (task.refBone >= 0) {
            const std::size_t rb = static_cast<std::size_t>(task.refBone);
            P = frames.pos[rb] + frames.rot[rb] * task.planePoint;
            nrm = frames.rot[rb] * task.normal;
        }
        const double depth = glm::dot(nrm, P - x);
        if (depth > 0.0) {
            cost += task.weight * depth * depth;
            r[row] = depth * std::sqrt(task.weight) * bounded(depth, &Settings::maxTaskStep);
        }
        ++row;
    }
    {
        glm::dvec2 direction;
        const double need = balanceNeed(frames, problem, com, direction);
        if (need > 0.0) {
            cost += problem.balanceWeight * need * need;
            r[row] = need * std::sqrt(problem.balanceWeight) * bounded(need, &Settings::maxBalanceStep);
        }
        ++row;
    }
    // The couplings (joint space, linear): residual = target - value, as every row's.
    for (const JointCouplingTask& task : problem.couplings) {
        if (task.dofA >= 0 && task.dofB >= 0 && static_cast<std::size_t>(task.dofA) < problem.dofs.size() &&
            static_cast<std::size_t>(task.dofB) < problem.dofs.size()) {
            const double a = dofValue(pose, problem.dofs[static_cast<std::size_t>(task.dofA)]) - task.refA;
            const double b = dofValue(pose, problem.dofs[static_cast<std::size_t>(task.dofB)]) - task.refB;
            const double e = -(task.scaleA * a - task.scaleB * b);
            cost += task.weight * e * e;
            r[row] = e * std::sqrt(task.weight);
        }
        ++row;
    }
    return cost;
}

/// A one-sided row is IN CONTACT when its joint is clear of the surface by less than this (m).
constexpr double kContactBand = 0.0005;

double JointSolver::linearize(const Pose& pose, const Frames& frames, const Problem& problem,
                              const std::vector<JointPlaneTask>& planes, const ComModel& com,
                              std::vector<double>& r, std::vector<double>& J,
                              const Settings* stepBounds, std::vector<double>* planeClear) const {
    const double cost = residuals(pose, frames, problem, planes, com, r, stepBounds);
    const std::size_t nDof = problem.dofs.size();
    const std::size_t rows = r.size();
    J.assign(rows * nDof, 0.0);
    if (planeClear != nullptr) {
        planeClear->assign(rows, 0.0);
        std::size_t row = 3 * problem.positions.size() + 3 * problem.orientations.size();
        for (const JointPlaneTask& task : planes) {
            if (r[row] == 0.0) {
                const std::size_t tb = static_cast<std::size_t>(task.bone);
                const glm::dvec3 x = frames.pos[tb] + frames.rot[tb] * task.offset;
                glm::dvec3 P = task.planePoint;
                glm::dvec3 nrm = task.normal;
                if (task.refBone >= 0) {
                    const std::size_t rb = static_cast<std::size_t>(task.refBone);
                    P = frames.pos[rb] + frames.rot[rb] * task.planePoint;
                    nrm = frames.rot[rb] * task.normal;
                }
                (*planeClear)[row] = std::min(0.0, glm::dot(nrm, P - x)) * std::sqrt(task.weight);
            }
            ++row;
        }
    }

    // The balance row: d(CoM) = sum_k c_k d(p_k) / M over a DoF's subtree, so the subtree sums
    // of c_k and c_k p_k give every DoF's column in O(1).
    const std::size_t       balanceRow = rows - 1 - problem.couplings.size();
    glm::dvec2              balanceDir(0.0);
    std::vector<double>     subCoeff;
    std::vector<glm::dvec3> subMoment;
    const bool balanceActive = r[balanceRow] != 0.0;
    if (balanceActive) {
        balanceNeed(frames, problem, com, balanceDir);
        const std::size_t n = m_bones.size();
        subCoeff.assign(n, 0.0);
        subMoment.assign(n, glm::dvec3(0.0));
        for (std::size_t i = n; i-- > 0;) { // children follow parents: accumulate upward
            subCoeff[i] += com.coeff[i];
            subMoment[i] += com.coeff[i] * frames.pos[i];
            const int p = m_bones[i].parent;
            if (p >= 0) {
                subCoeff[static_cast<std::size_t>(p)] += subCoeff[i];
                subMoment[static_cast<std::size_t>(p)] += subMoment[i];
            }
        }
    }

    for (std::size_t j = 0; j < nDof; ++j) {
        const JointSolverDof& dof = problem.dofs[j];
        const glm::dvec3 axis = dofAxis(pose, frames, dof);
        const bool translation = dof.axis >= 3;
        const glm::dvec3& pivot = frames.pos[static_cast<std::size_t>(dof.bone)];
        std::size_t row = 0;
        for (const JointPositionTask& task : problem.positions) {
            if (isAncestorOrSelf(dof.bone, task.bone)) {
                const std::size_t tb = static_cast<std::size_t>(task.bone);
                const glm::dvec3 x = frames.pos[tb] + frames.rot[tb] * task.offset;
                const double len = glm::length((task.target - x) * task.axisScale);
                double w = (task.huber > 0.0 && len > task.huber)
                               ? task.weight * task.huber / len
                               : task.weight;
                if (stepBounds != nullptr && len > stepBounds->maxTaskStep && askKeepsGradient()) {
                    w *= len / stepBounds->maxTaskStep; // (the bounded ask at 1/k of the weight: residuals())
                }
                const glm::dvec3 d = (translation ? axis : glm::cross(axis, x - pivot)) *
                                     task.axisScale * std::sqrt(w);
                J[(row + 0) * nDof + j] = d.x;
                J[(row + 1) * nDof + j] = d.y;
                J[(row + 2) * nDof + j] = d.z;
            }
            row += 3;
        }
        for (const JointOrientationTask& task : problem.orientations) {
            if (!translation && isAncestorOrSelf(dof.bone, task.bone)) {
                glm::dvec3 d(0.0);
                if (task.perAxis) {
                    // d(error)/d(theta), negated like every row (r - J d): the lateral axis
                    // turns by axis x a, so the swing turns by (axis x a) x a_target — which is
                    // -(axis's part across a) when a sits on its target, the old row exactly —
                    // and the pitch by the rotation's part ALONG the target lateral axis.
                    glm::dvec3 lateralNow(0.0);
                    lateralAxisError(task.target, frames.rot[static_cast<std::size_t>(task.bone)], task.frame,
                                     &lateralNow);
                    const glm::dvec3 swingRate = -glm::cross(glm::cross(axis, lateralNow), task.frame[0]);
                    // The pitch's exact rate: pitch = atan2(N, D) over the fore axis projected
                    // across the target lateral axis (the rotation's part along that axis when
                    // the lateral axis sits on its target).
                    const glm::dmat3 delta = frames.rot[static_cast<std::size_t>(task.bone)] * glm::transpose(task.target);
                    const glm::dvec3 foreNow = delta * task.frame[2];
                    const glm::dvec3 f = foreNow - task.frame[0] * glm::dot(foreNow, task.frame[0]);
                    glm::dvec3 df = glm::cross(axis, foreNow);
                    df -= task.frame[0] * glm::dot(df, task.frame[0]);
                    const double N = glm::dot(glm::cross(f, task.frame[2]), task.frame[0]);
                    const double D = glm::dot(f, task.frame[2]);
                    const double dN = glm::dot(glm::cross(df, task.frame[2]), task.frame[0]);
                    const double dD = glm::dot(df, task.frame[2]);
                    const double norm2 = N * N + D * D;
                    const double pitchRate = norm2 > 1e-12 ? -(D * dN - N * dD) / norm2
                                                           : glm::dot(axis, task.frame[0]);
                    const glm::dvec3 rate(pitchRate, glm::dot(swingRate, task.frame[1]),
                                          glm::dot(swingRate, task.frame[2]));
                    for (int k = 0; k < 3; ++k) {
                        d += task.frame[k] * (rate[k] * std::sqrt(task.axisWeights[k]));
                    }
                } else {
                    d = axis * std::sqrt(task.weight);
                }
                J[(row + 0) * nDof + j] = d.x;
                J[(row + 1) * nDof + j] = d.y;
                J[(row + 2) * nDof + j] = d.z;
            }
            row += 3;
        }
        for (const JointPlaneTask& task : planes) {
            const bool movesPoint = isAncestorOrSelf(dof.bone, task.bone);
            const bool movesPlane = task.refBone >= 0 && isAncestorOrSelf(dof.bone, task.refBone);
            if ((r[row] != 0.0 || planeClear != nullptr) && movesPoint != movesPlane) {
                // (A DoF that carries both the point and the plane changes nothing between them.)
                const std::size_t tb = static_cast<std::size_t>(task.bone);
                const glm::dvec3 x = frames.pos[tb] + frames.rot[tb] * task.offset;
                glm::dvec3 P = task.planePoint;
                glm::dvec3 nrm = task.normal;
                if (task.refBone >= 0) {
                    const std::size_t rb = static_cast<std::size_t>(task.refBone);
                    P = frames.pos[rb] + frames.rot[rb] * task.planePoint;
                    nrm = frames.rot[rb] * task.normal;
                }
                // The row asks for MORE separation n.(x - P): its derivative over the DoF.
                double d = 0.0;
                if (movesPoint) {
                    d = glm::dot(nrm, translation ? axis : glm::cross(axis, x - pivot));
                } else if (translation) {
                    d = -glm::dot(nrm, axis);
                } else {
                    d = -glm::dot(nrm, glm::cross(axis, P - pivot)) -
                        glm::dot(glm::cross(axis, nrm), P - x);
                }
                J[row * nDof + j] = d * std::sqrt(task.weight);
            }
            row += 1;
        }
        if (balanceActive) {
            const std::size_t b = static_cast<std::size_t>(dof.bone);
            const glm::dvec3 d =
                translation ? axis * (subCoeff[b] / com.total)
                            : glm::cross(axis, subMoment[b] - subCoeff[b] * pivot) / com.total;
            J[balanceRow * nDof + j] =
                (d.x * balanceDir.x + d.z * balanceDir.y) * std::sqrt(problem.balanceWeight);
        }
    }
    for (std::size_t c = 0; c < problem.couplings.size(); ++c) {
        const JointCouplingTask& task = problem.couplings[c];
        if (task.dofA < 0 || task.dofB < 0 || static_cast<std::size_t>(task.dofA) >= nDof ||
            static_cast<std::size_t>(task.dofB) >= nDof) {
            continue;
        }
        const std::size_t row = balanceRow + 1 + c;
        J[row * nDof + static_cast<std::size_t>(task.dofA)] = task.scaleA * std::sqrt(task.weight);
        J[row * nDof + static_cast<std::size_t>(task.dofB)] = -task.scaleB * std::sqrt(task.weight);
    }
    return cost;
}

/// A row at or over this weight is one whose miss means the solve is STUCK (the cursor's 1e6, the
/// pins' 1e8, the floor's and the volumes' 1e7, balance's 1e4 are tallied apart); under it a row is
/// a preference - a heel's soft hold, a sole's pitch, a palm's pull - that rests unmet by design.
constexpr double kStallRowWeight = 1.0e5;

double JointSolver::softCost(const Pose& pose, const Problem& problem) const {
    Frames frames;
    forwardKinematics(pose, frames);
    double cost = 0.0;
    for (const JointPositionTask& task : problem.positions) {
        if (task.weight >= kStallRowWeight) {
            continue;
        }
        const std::size_t tb = static_cast<std::size_t>(task.bone);
        const glm::dvec3 e = (task.target - (frames.pos[tb] + frames.rot[tb] * task.offset)) * task.axisScale;
        const double len = glm::length(e);
        cost += (task.huber > 0.0 && len > task.huber) ? task.weight * task.huber * (2.0 * len - task.huber)
                                                        : task.weight * len * len;
    }
    for (const JointOrientationTask& task : problem.orientations) {
        const std::size_t tb = static_cast<std::size_t>(task.bone);
        if (task.perAxis) {
            const glm::dvec3 err = lateralAxisError(task.target, frames.rot[tb], task.frame, nullptr);
            for (int k = 0; k < 3; ++k) {
                if (task.axisWeights[k] < kStallRowWeight) {
                    cost += task.axisWeights[k] * err[k] * err[k];
                }
            }
        } else if (task.weight < kStallRowWeight) {
            const glm::dvec3 e = rotationVector(task.target * glm::transpose(frames.rot[tb]));
            cost += task.weight * glm::dot(e, e);
        }
    }
    for (const JointCouplingTask& task : problem.couplings) {
        if (task.dofA >= 0 && task.dofB >= 0 && static_cast<std::size_t>(task.dofA) < problem.dofs.size() &&
            static_cast<std::size_t>(task.dofB) < problem.dofs.size()) {
            const double a = dofValue(pose, problem.dofs[static_cast<std::size_t>(task.dofA)]) - task.refA;
            const double b = dofValue(pose, problem.dofs[static_cast<std::size_t>(task.dofB)]) - task.refB;
            const double e = task.scaleA * a - task.scaleB * b;
            cost += task.weight * e * e;
        }
    }
    // (... and BALANCE, which rests a centimetre out by design wherever she leans on the edge of
    // her support: its pull meets the cursor's there. IK_JS_RETRY_ON_BALANCE counts it again.)
    static const bool kRetryOnBalance = std::getenv("IK_JS_RETRY_ON_BALANCE") != nullptr; // A/B probe
    if (!kRetryOnBalance) {
        glm::dvec2   direction;
        const double need = balanceNeed(frames, problem, buildComModel(problem), direction);
        if (need > 0.0) {
            cost += problem.balanceWeight * need * need;
        }
    }
    return cost;
}

double JointSolver::cost(const Pose& pose, const Problem& problem) const {
    Frames frames;
    forwardKinematics(pose, frames);
    std::vector<double> r;
    std::vector<JointPlaneTask> planes = problem.planes;
    if (problem.planeSource) {
        planes.clear();
        problem.planeSource(frames, planes);
    }
    return residuals(pose, frames, problem, planes, buildComModel(problem), r) + postureCost(pose, problem);
}

void JointSolver::dofBounds(const Problem& problem, std::vector<double>& lo,
                            std::vector<double>& hi) const {
    const std::size_t nDof = problem.dofs.size();
    const double inf = std::numeric_limits<double>::infinity();
    lo.assign(nDof, -inf);
    hi.assign(nDof, inf);
    for (std::size_t j = 0; j < nDof; ++j) {
        const JointSolverDof& dof = problem.dofs[j];
        if (dof.axis >= 3) {
            continue;
        }
        const JointSolverBone& b = m_bones[static_cast<std::size_t>(dof.bone)];
        if (b.limited[dof.axis]) {
            lo[j] = b.minDeg[dof.axis] * kDegToRad;
            hi[j] = b.maxDeg[dof.axis] * kDegToRad;
        }
        if (dof.minDeg > -1.0e29) {
            lo[j] = std::max(lo[j], dof.minDeg * kDegToRad);
        }
        if (dof.maxDeg < 1.0e29) {
            hi[j] = std::min(hi[j], dof.maxDeg * kDegToRad);
        }
        if (lo[j] > hi[j]) {
            lo[j] = hi[j]; // contradictory bounds: the authored limit wins its side
        }
    }
}

JointSolver::Result JointSolver::solve(Pose& pose, const Problem& problem,
                                       const Settings& settings) const {
    std::vector<char> blocked;
    Result result = descend(pose, problem, settings, blocked);
    // The FOLD retry (see JointSolverDof::foldSign): a fold channel the descent left NEARLY
    // STRAIGHT while a task is still unmet is tried a few degrees into flexion, and the cheaper
    // pose kept. "Nearly straight", not "pinned at its bound": a knee a hair off its bound is on
    // the same saddle (a softly started pelvis drag asked a perfectly straight-legged rig for a
    // fraction of a millimetre, a micro-step moved the knees 1e-5 rad off the bound, and the
    // crouch then never began). An unreachable cursor keeps a straight arm retrying every tick —
    // one extra descent, whose result must beat the first by a real margin to be taken, so two
    // descents that land in the same minimum never trade places.
    constexpr double kFoldKick = 6.0 * kDegToRad;
    constexpr double kFoldNearStraight = 3.0 * kDegToRad;
    constexpr double kFoldTaskCost = 1.0e-2;
    (void)blocked;
    // ("A task still unmet" counts the SOFT rows too - a heel's hold, a sole's pitch, the couplings,
    // balance at rest on the support's edge - and so the retry fires on most ticks a knee or an
    // elbow stands near straight. Narrowed to the hard rows (IK_JS_RETRY_HARD_ONLY: softCost) it
    // was tried and put back: a foot lifted, carried and set down had been unlocking its straight
    // knee early and gently on exactly such a soft row, 49mm in its worst tick, and unlocked it
    // late and all at once without, 125mm. What must not be kicked says so: JointSolverDof::foldRetry.)
    static const bool kNoFoldRetry = std::getenv("IK_JS_NO_FOLD_RETRY") != nullptr;   // A/B probe
    static const bool kRetryHardOnly = std::getenv("IK_JS_RETRY_HARD_ONLY") != nullptr; // A/B probe
    if (!kNoFoldRetry &&
        result.cost - postureCost(pose, problem) - (kRetryHardOnly ? softCost(pose, problem) : 0.0) > kFoldTaskCost) {
        static const bool kRetryTrace = std::getenv("IK_JS_RETRY_TRACE") != nullptr;
        if (kRetryTrace) {
            Frames frames;
            forwardKinematics(pose, frames);
            std::fprintf(stderr, "[js-retry] cost %.4f posture %.4f soft %.4f:", result.cost, postureCost(pose, problem), softCost(pose, problem));
            for (const JointPositionTask& task : problem.positions) {
                const std::size_t tb = static_cast<std::size_t>(task.bone);
                const glm::dvec3 e = (task.target - (frames.pos[tb] + frames.rot[tb] * task.offset)) * task.axisScale;
                if (task.weight >= kStallRowWeight && task.weight * glm::dot(e, e) > 1.0e-3) {
                    std::fprintf(stderr, " pos[bone %d w %.0e miss %.2fmm]", task.bone, task.weight, glm::length(e) * 1000.0);
                }
            }
            for (const JointOrientationTask& task : problem.orientations) {
                if (!task.perAxis && task.weight >= kStallRowWeight) {
                    const glm::dvec3 e = rotationVector(task.target * glm::transpose(frames.rot[static_cast<std::size_t>(task.bone)]));
                    if (task.weight * glm::dot(e, e) > 1.0e-3) {
                        std::fprintf(stderr, " orient[bone %d w %.0e err %.3fdeg]", task.bone, task.weight, glm::length(e) * 57.2958);
                    }
                }
            }
            std::fputc(10, stderr);
        }
        std::vector<double> lo, hi;
        dofBounds(problem, lo, hi);
        Pose kicked = pose;
        bool any = false;
        for (std::size_t j = 0; j < problem.dofs.size(); ++j) {
            const JointSolverDof& dof = problem.dofs[j];
            if (dof.foldSign == 0 || !dof.foldRetry) {
                continue;
            }
            const double bound = dof.foldSign > 0 ? lo[j] : hi[j];
            const double flexed = (dofValue(pose, dof) - bound) * dof.foldSign;
            if (flexed < kFoldNearStraight) {
                setDofValue(kicked, dof, glm::clamp(bound + dof.foldSign * kFoldKick, lo[j], hi[j]));
                if (kRetryTrace) {
                    std::fprintf(stderr, "[js-retry]   kicks bone %d axis %d (flexed %.2f deg)\n", dof.bone, dof.axis, flexed * 57.2958);
                }
                any = true;
            }
        }
        if (any) {
            std::vector<char> blockedKicked;
            const Result retry = descend(kicked, problem, settings, blockedKicked);
            if (retry.cost < result.cost * (1.0 - 1.0e-3) - 1.0e-9) {
                const int spent = result.iterations;
                pose = kicked;
                result = retry;
                result.iterations += spent;
            }
        }
    }
    return result;
}

JointSolver::Result JointSolver::descend(Pose& pose, const Problem& problem, const Settings& settings,
                                         std::vector<char>& blockedOut) const {
    Result result;
    const std::size_t nDof = problem.dofs.size();
    blockedOut.assign(nDof, 0);
    if (nDof == 0 || m_bones.empty()) {
        result.converged = true;
        return result;
    }
    std::vector<double> lo, hi, ref(nDof, 0.0);
    dofBounds(problem, lo, hi);
    for (std::size_t j = 0; j < nDof; ++j) {
        const JointSolverDof& dof = problem.dofs[j];
        ref[j] = dof.axis < 3 ? dof.reference * kDegToRad : dof.reference;
    }

    const ComModel com = buildComModel(problem);
    Frames frames;
    forwardKinematics(pose, frames);
    std::vector<double> r, rTrial, J, planeClear;
    std::vector<char>   rowOff, rowEntered;
    std::vector<JointPlaneTask> planes = problem.planes;
    std::vector<JointPlaneTask> trialPlanes;
    double cost0 = residuals(pose, frames, problem, planes, com, r) + postureCost(pose, problem);

    std::vector<double> q(nDof), pull(nDof), dInv(nDof), step(nDof), d0(nDof), M, rhs;
    std::vector<char>   blocked(nDof);
    Pose   trial = pose;
    Frames trialFrames;
    double mu = 1e-4;

    bool planesArePoses = false; // the planes are already this pose's own: the accepted trial's
    for (int it = 0; it < settings.maxIterations; ++it) {
        if (problem.planeSource && !planesArePoses) {
            // Fresh tangent planes for this linearization, the reference cost read under them:
            // a pose's cost is the cost under ITS rows (a trial's too — see the halving loop,
            // which hands an accepted trial's rows on to the next linearization).
            planes.clear();
            problem.planeSource(frames, planes);
            cost0 = residuals(pose, frames, problem, planes, com, r) + postureCost(pose, problem);
        }
        planesArePoses = false;
        linearize(pose, frames, problem, planes, com, r, J, &settings, &planeClear);
        const std::size_t rows = r.size();
        // A ONE-SIDED row costs and asks nothing until its joint is THROUGH the surface, so the
        // linear model cannot see a surface a joint is resting exactly ON. Under load that never
        // happens (a pressed contact sits a micron inside: visible) — but a joint that comes to
        // rest AT a surface with nothing pressing it is a trap: every step the model proposes
        // goes through, the true cost rises by 1e7 x depth^2 against a cursor's bounded pull, and
        // only steps far smaller than ten dampings reach are downhill — the solve "converges"
        // where it stands. A flat foot slid along the floor (its ankle and eight toe joints
        // exactly at their clearances) stopped dead 4cm short of a cursor it could reach; a knee
        // resting on the other thigh's capsule held a crossing foot 5cm short. So a row IN
        // CONTACT (clear by less than kContactBand) comes INTO the model when the step would
        // cross it, with the room it has (a negative residual): the step may use the room and
        // no more. Rows further out stay out, as ever — a capsule's tangent plane centimetres
        // from its surface is a poor stand-in for it, and brought in from there it held joints off
        // volumes they were sliding along.
        const std::size_t planeRow0 = 3 * problem.positions.size() + 3 * problem.orientations.size();
        const std::vector<double> rLinear = r;
        for (std::size_t j = 0; j < nDof; ++j) {
            q[j] = dofValue(pose, problem.dofs[j]);
            pull[j] = ref[j] - q[j];
        }
        bool accepted = false;
        double appliedMax = 0.0;
        for (int attempt = 0; attempt < 10 && !accepted; ++attempt) {
            std::fill(blocked.begin(), blocked.end(), 0);
            r = rLinear;
            rowOff.assign(rows, 0);
            rowEntered.assign(rows, 0);
            for (std::size_t a = planeRow0; a < planeRow0 + planes.size(); ++a) {
                rowOff[a] = r[a] == 0.0 ? 1 : 0;
            }
            for (int pass = 0; pass < 16; ++pass) {
                for (std::size_t j = 0; j < nDof; ++j) {
                    const double s = std::max(problem.dofs[j].stiffness, 1e-9);
                    const double d = s + mu * (problem.dofs[j].axis < 3 ? 1.0 : 25.0);
                    dInv[j] = blocked[j] ? 0.0 : 1.0 / d;
                    d0[j] = blocked[j] ? 0.0 : pull[j] * s / d;
                }
                // rhs = r - J d0;  M = J D^-1 J^T + I
                rhs.assign(rows, 0.0);
                M.assign(rows * rows, 0.0);
                for (std::size_t a = 0; a < rows; ++a) {
                    if (rowOff[a]) {
                        M[a * rows + a] = 1.0; // out of the model: an identity row, a zero ask
                        continue;
                    }
                    const double* Ja = &J[a * nDof];
                    double acc = r[a];
                    for (std::size_t j = 0; j < nDof; ++j) {
                        acc -= Ja[j] * d0[j];
                    }
                    rhs[a] = acc;
                    for (std::size_t b = 0; b <= a; ++b) {
                        if (rowOff[b]) {
                            continue;
                        }
                        const double* Jb = &J[b * nDof];
                        double m = 0.0;
                        for (std::size_t j = 0; j < nDof; ++j) {
                            m += Ja[j] * dInv[j] * Jb[j];
                        }
                        M[a * rows + b] = m;
                        M[b * rows + a] = m;
                    }
                    M[a * rows + a] += 1.0;
                }
                if (rows > 0 && !choleskySolve(M, rhs, static_cast<int>(rows))) {
                    std::fill(rhs.begin(), rhs.end(), 0.0); // posture-only step
                }
                for (std::size_t j = 0; j < nDof; ++j) {
                    double acc = 0.0;
                    for (std::size_t a = 0; a < rows; ++a) {
                        if (!rowOff[a]) {
                            acc += J[a * nDof + j] * rhs[a];
                        }
                    }
                    step[j] = d0[j] + dInv[j] * acc;
                }
                // Active set: a channel AT its limit whose step points outward is frozen.
                bool grew = false;
                for (std::size_t j = 0; j < nDof; ++j) {
                    if (blocked[j]) {
                        continue;
                    }
                    // ... and so is a channel a HAIR inside its limit whose step would go far past
                    // it (its room under a fiftieth of the step): the projection below clamps such
                    // a step to nothing, the model has counted on all of it, and — the pose
                    // unchanged by a rejected step — no damping ever makes the step small enough
                    // to fit: a wall of REJECTs for good. A custom character's shin, its side-bend
                    // channel 0.0004 degrees inside its 5-degree limit after the first tick of a
                    // crouch, locked both legs straight: the hips did not come down a millimetre
                    // under a cursor half a metre below them. (IK_JS_EXACT_ACTIVE_SET is the A/B.)
                    static const bool kExactActiveSet = std::getenv("IK_JS_EXACT_ACTIVE_SET") != nullptr;
                    const double hair = kExactActiveSet ? 0.0 : 0.02 * std::abs(step[j]);
                    if ((q[j] <= lo[j] + std::max(1e-9, hair) && step[j] < 0.0) ||
                        (q[j] >= hi[j] - std::max(1e-9, hair) && step[j] > 0.0)) {
                        blocked[j] = 1;
                        grew = true;
                    }
                }
                // ... and a clear one-sided row the step would cross comes into the model — and
                // goes back out if, with the others in, it ends up PULLING its joint toward the
                // surface (its residual after the step is the row's multiplier: negative = the
                // joint ends clear and the row is an attraction, which a surface is not). ONE
                // change of a kind per pass, drops first: out, a pulling row's joint can only end
                // further from its surface — but only with every other row as it was (dropped
                // together, or dropped while others entered, rows came straight back across, ran
                // out of entries, and the step went through the floor: every attempt rejected, a
                // "converged" pose 2cm short of the one the solve had always found).
                static const bool kNoRowActive = std::getenv("IK_JS_NO_ROW_ACTIVE") != nullptr; // A/B probe
                if (!grew && !kNoRowActive) {
                    std::size_t drop = rows;
                    for (std::size_t a = planeRow0; a < planeRow0 + planes.size(); ++a) {
                        if (!rowOff[a] && rowEntered[a] && rhs[a] < -1e-9 && (drop == rows || rhs[a] < rhs[drop])) {
                            drop = a;
                        }
                    }
                    if (drop != rows) {
                        rowOff[drop] = 1;
                        r[drop] = 0.0;
                        grew = true;
                    }
                }
                if (!grew && !kNoRowActive) {
                    for (std::size_t a = planeRow0; a < planeRow0 + planes.size(); ++a) {
                        const double band = kContactBand * std::sqrt(planes[a - planeRow0].weight);
                        if (!rowOff[a] || rowEntered[a] >= 4 || planeClear[a] < -band) {
                            continue;
                        }
                        double predicted = planeClear[a];
                        for (std::size_t j = 0; j < nDof; ++j) {
                            predicted -= blocked[j] ? 0.0 : J[a * nDof + j] * step[j];
                        }
                        if (predicted > 1e-9) {
                            rowOff[a] = 0;
                            ++rowEntered[a];
                            r[a] = planeClear[a];
                            grew = true;
                        }
                    }
                }
                if (!grew) {
                    break;
                }
            }
            // Trust bound: the whole step scaled (direction kept) to the per-iteration caps.
            double scale = 1.0;
            for (std::size_t j = 0; j < nDof; ++j) {
                if (blocked[j]) {
                    step[j] = 0.0;
                    continue;
                }
                const double cap =
                    problem.dofs[j].axis < 3 ? settings.maxStepRad : settings.maxStepMeters;
                const double mag = std::abs(step[j]);
                if (mag * scale > cap) {
                    scale = cap / mag;
                }
            }
            // ... and the RATIO TEST (2026-09-22): the largest fraction of the step that keeps every
            // channel inside its box. The trial clamps each channel to its limit, and a channel
            // the model counts on — a compensator: the toes' pitch keeping the toe tips on the
            // floor while a tucked foot rolls onto its ball, asked 20 degrees with 7 left — is
            // then clamped ALONE, the rest of the step taken in full, and the model's prediction
            // broken by exactly the part it lost: seven rejected dampings a tick, the solve out
            // of iterations, a kneeling knee stalled 13cm behind its goal (a half-size character's
            // half kneel found its height through 85 degrees of thigh abduction instead, the foot
            // flung above the knee). The hair rule above blocks a channel with under 2% of its
            // ask; one with more is taken to its bound and no further, the whole step scaled so
            // the model holds for it, and blocked at the next pass. Tried THIRD, after the full
            // and the half step have failed clamped: where the clamped channel was not
            // load-bearing the full step is the faster one. IK_JS_NO_RATIO_STEP is the A/B.
            static const bool kNoRatioStep = std::getenv("IK_JS_NO_RATIO_STEP") != nullptr;
            double ratio = 1.0;
            for (std::size_t j = 0; j < nDof && !kNoRatioStep; ++j) {
                if (blocked[j] || std::abs(step[j]) < 1e-12) {
                    continue;
                }
                const double asked = step[j] * scale;
                const double room = asked > 0.0 ? hi[j] - q[j] : q[j] - lo[j];
                if (room < std::abs(asked)) {
                    ratio = std::min(ratio, std::max(0.0, room) / std::abs(asked));
                }
            }
            // Backtracking along the step: a Gauss-Newton direction descends for a short
            // enough step, and halving it is cheaper than re-solving at a higher damping (which
            // barely shortens a step the hard rows dictate).
            for (int halving = 0; halving < 3 && !accepted; ++halving) {
                if (halving == 2 && (kNoRatioStep || ratio >= 0.5 || ratio <= 0.0)) {
                    break; // (the half step was already inside the box, or nothing is)
                }
                const double factor = halving < 2 ? scale * std::pow(0.5, halving) : scale * ratio;
                trial = pose;
                double maxApplied = 0.0;
                for (std::size_t j = 0; j < nDof; ++j) {
                    const double v = glm::clamp(q[j] + step[j] * factor, lo[j], hi[j]);
                    maxApplied = std::max(maxApplied, std::abs(v - q[j]));
                    setDofValue(trial, problem.dofs[j], v);
                }
                forwardKinematics(trial, trialFrames);
                // A TRIAL IS JUDGED BY THE ROWS IT MAKES (2026-09-28; IK_JS_STALE_TRIAL_PLANES is
                // the A/B). The one-sided rows are regenerated per linearization, within a margin
                // of each surface — and a step of 20 degrees at a shoulder moves its elbow 9cm:
                // from outside the margin, where there is no row, to 15mm INSIDE the chest. Judged
                // by the rows of the pose it left, that trial cost 211 against 245 and was taken;
                // the next linearization read the same pose at 4900, threw the arm back out past
                // the margin, and the one after took it in again — a two-cycle, sixteen iterations
                // a tick, the pose left wherever the count ran out (a custom character's prone
                // descent: her upper arms wedged between the chest and hands that had let go, the
                // head thrown 258mm when the cycle broke). The true cost of a pose is the cost
                // under ITS rows.
                static const bool kStaleTrialPlanes = std::getenv("IK_JS_STALE_TRIAL_PLANES") != nullptr; // A/B probe
                const std::vector<JointPlaneTask>* judgedBy = &planes;
                if (problem.planeSource && !kStaleTrialPlanes) {
                    trialPlanes.clear();
                    problem.planeSource(trialFrames, trialPlanes);
                    judgedBy = &trialPlanes;
                }
                const double costTrial =
                    residuals(trial, trialFrames, problem, *judgedBy, com, rTrial) + postureCost(trial, problem);
                static const bool kLmTrace = std::getenv("IK_JS_LM_TRACE") != nullptr;
                if (kLmTrace) {
                    int nBlocked = 0;
                    for (const char b : blocked) {
                        nBlocked += b ? 1 : 0;
                    }
                    std::fprintf(stderr,
                                 "[lm] it=%d try=%d/%d mu=%.3g factor=%.3g blocked=%d step=%.3g cost %.9g -> %.9g%s\n",
                                 it, attempt, halving, mu, factor, nBlocked, maxApplied, cost0, costTrial,
                                 costTrial < cost0 ? " ok" : " REJECT");
                }
                // IK_JS_TRIAL_PLANE_TRACE: for a REJECTED trial, its cost under the linearization's
                // rows beside its own, and every row of either set that costs at the trial — which
                // surface the step ran into that the model had no row for.
                static const bool kTrialPlaneTrace = std::getenv("IK_JS_TRIAL_PLANE_TRACE") != nullptr;
                if (kTrialPlaneTrace && costTrial >= cost0 && judgedBy == &trialPlanes && (attempt == 0 || attempt == 9) && halving == 0) {
                    std::vector<double> rStale;
                    const double costStale = residuals(trial, trialFrames, problem, planes, com, rStale) + postureCost(trial, problem);
                    std::fprintf(stderr, "[trial-planes] attempt %d: cost %.6g at the pose, %.6g at the trial under its own rows, %.6g under the pose's (%zu rows, %zu at the pose)\n",
                                 attempt, cost0, costTrial, costStale, trialPlanes.size(), planes.size());
                    const auto dump = [&](const char* label, const std::vector<JointPlaneTask>& set, const Frames& at) {
                        for (const JointPlaneTask& plane : set) {
                            const std::size_t b = static_cast<std::size_t>(plane.bone);
                            const glm::dvec3  point = at.pos[b] + at.rot[b] * plane.offset;
                            glm::dvec3        onPlane = plane.planePoint;
                            glm::dvec3        normal = plane.normal;
                            if (plane.refBone >= 0) {
                                const std::size_t ref = static_cast<std::size_t>(plane.refBone);
                                onPlane = at.pos[ref] + at.rot[ref] * plane.planePoint;
                                normal = at.rot[ref] * plane.normal;
                            }
                            const double depth = -glm::dot(point - onPlane, normal);
                            static const bool kAllNear = std::getenv("IK_JS_TRIAL_PLANE_TRACE_NEAR") != nullptr; // ... and the rows within 5mm of their surface
                            if (depth > 1.0e-4 || (kAllNear && depth > -5.0e-3)) {
                                std::fprintf(stderr, "[trial-planes]   %s: bone %d (offset %.3f %.3f %.3f) against %d, %.2fmm through, weight %.3g\n", label, plane.bone,
                                             plane.offset.x, plane.offset.y, plane.offset.z, plane.refBone, depth * 1000.0, plane.weight);
                            }
                        }
                    };
                    dump("the pose's rows, at the pose", planes, frames);
                    dump("the pose's rows, at the trial", planes, trialFrames);
                    dump("the trial's rows, at the trial", trialPlanes, trialFrames);
                }
                static const bool kRowTrace = std::getenv("IK_JS_ROW_TRACE") != nullptr;
                static const bool kRowTraceFirst = std::getenv("IK_JS_ROW_TRACE_FIRST") != nullptr; // the FIRST rejected attempt, not the last
                if (kRowTrace && costTrial >= cost0 && ((attempt == 9 && halving == 1) || (kRowTraceFirst && attempt == 0 && halving == 0))) {
                    // Which row did the model get wrong? Per row: its residual, what the model
                    // predicted for it after this (tiny) step, and what it came to.
                    const double p0 = postureCost(pose, problem), p1 = postureCost(trial, problem);
                    std::fprintf(stderr, "[rows] posture %.9g -> %.9g; rows %zu (pos %zu, ori %zu, planes %zu)\n", p0, p1, rows,
                                 problem.positions.size(), problem.orientations.size(), planes.size());
                    for (std::size_t j = 0; j < nDof; ++j) {
                        const double asked = q[j] + step[j] * factor;
                        const double got = glm::clamp(asked, lo[j], hi[j]);
                        if (std::abs(asked - got) > 1e-6) {
                            std::fprintf(stderr, "[rows]   clamped: bone %d axis %d at %.4f asked %+.4f got %+.4f%s", problem.dofs[j].bone, problem.dofs[j].axis, q[j], step[j] * factor, got - q[j], blocked[j] ? " BLOCKED" : "");
                            std::fputc(10, stderr);
                        }
                    }
                    std::vector<double> rNow;
                    residuals(pose, frames, problem, planes, com, rNow);
                    residuals(trial, trialFrames, problem, planes, com, rTrial); // (row for row with the model's: the linearization's planes)
                    for (std::size_t a = 0; a < rows && a < rTrial.size() && a < rNow.size(); ++a) {
                        double moved = 0.0;
                        for (std::size_t j = 0; j < nDof; ++j) {
                            moved -= J[a * nDof + j] * step[j] * factor;
                        }
                        const double trueMoved = rTrial[a] - rNow[a];
                        const double dTrue = rTrial[a] * rTrial[a] - rNow[a] * rNow[a];
                        if (std::abs(trueMoved - moved) > 0.2 * std::abs(moved) + 1e-7 || std::abs(dTrue) > 1e-5) {
                            const std::size_t posRows = problem.positions.size() * 3;
                            const std::size_t oriRows = problem.orientations.size() * 3;
                            const int rowBone = a < posRows ? problem.positions[a / 3].bone
                                                : a < posRows + oriRows ? problem.orientations[(a - posRows) / 3].bone
                                                : a < posRows + oriRows + planes.size() ? planes[a - posRows - oriRows].bone : -1;
                            std::fprintf(stderr, "[rows]   row %zu (task %zu bone %d): r %.6g -> %.6g (model moved %+.3g, true %+.3g) cost %+.3g\n", a, a < posRows ? a / 3 : a, rowBone, rNow[a],
                                         rTrial[a], moved, trueMoved, dTrue);
                        }
                    }
                }
                if (costTrial < cost0) {
                    pose = trial;
                    frames = trialFrames;
                    cost0 = costTrial;
                    accepted = true;
                    appliedMax = maxApplied;
                    if (judgedBy == &trialPlanes) {
                        planes.swap(trialPlanes); // (the next linearization's: no second reading)
                        planesArePoses = true;
                    }
                    if (halving == 0) {
                        mu = std::max(mu * 0.2, 1e-6);
                    }
                }
            }
            if (!accepted) {
                mu = std::max(mu * 10.0, 1e-6);
            }
        }
        result.iterations = it + 1;
        blockedOut = blocked;
        if (!accepted) {
            result.converged = true; // no step improves the cost: a (local) minimum
            break;
        }
        result.maxStep = appliedMax;
        if (appliedMax < settings.stepTolerance) {
            result.converged = true;
            break;
        }
    }
    // IK_JS_GRAD_TRACE=<bone index>: at the end of a descent, what each row and the posture term
    // ask of that bone's channels (the true gradient's terms, J^T r with r unbounded).
    static const char* kGradTrace = std::getenv("IK_JS_GRAD_TRACE");
    if (kGradTrace != nullptr) {
        const int bone = std::atoi(kGradTrace);
        std::vector<double> rTrue, Jt;
        if (problem.planeSource) {
            planes.clear();
            problem.planeSource(frames, planes);
        }
        linearize(pose, frames, problem, planes, com, rTrue, Jt, nullptr, nullptr);
        for (std::size_t j = 0; j < nDof; ++j) {
            if (problem.dofs[j].bone != bone) {
                continue;
            }
            const double q0 = dofValue(pose, problem.dofs[j]);
            std::fprintf(stderr, "[grad] bone %d axis %d: value %.4f ref %.4f stiffness %.4g -> posture pull %+.4g (rows: pos %zu, ori %zu, planes %zu)\n",
                         bone, problem.dofs[j].axis, q0, ref[j], problem.dofs[j].stiffness,
                         problem.dofs[j].stiffness * (ref[j] - q0), problem.positions.size(), problem.orientations.size(),
                         planes.size());
            for (std::size_t a = 0; a < rTrue.size(); ++a) {
                const double g = Jt[a * nDof + j] * rTrue[a];
                if (std::abs(g) > 1e-3) {
                    std::fprintf(stderr, "[grad]   row %zu: r %+.5g, J %+.5g -> asks %+.4g\n", a, rTrue[a], Jt[a * nDof + j], g);
                }
            }
        }
    }
    result.cost = cost0;
    return result;
}

double JointSolver::jacobianError(const Pose& pose, const Problem& problem, int* worstRow, int* worstDof) const {
    const ComModel com = buildComModel(problem);
    Frames frames;
    forwardKinematics(pose, frames);
    std::vector<double> r, J;
    linearize(pose, frames, problem, problem.planes, com, r, J);
    const std::size_t nDof = problem.dofs.size();
    const std::size_t rows = r.size();
    double worst = 0.0;
    const double h = 1e-6;
    for (std::size_t j = 0; j < nDof; ++j) {
        Pose plus = pose, minus = pose;
        setDofValue(plus, problem.dofs[j], dofValue(pose, problem.dofs[j]) + h);
        setDofValue(minus, problem.dofs[j], dofValue(pose, problem.dofs[j]) - h);
        Frames fp, fm;
        forwardKinematics(plus, fp);
        forwardKinematics(minus, fm);
        std::vector<double> rp, rm;
        residuals(plus, fp, problem, problem.planes, com, rp);
        residuals(minus, fm, problem, problem.planes, com, rm);
        for (std::size_t a = 0; a < rows; ++a) {
            // The residual is (target - value): its derivative is MINUS the Jacobian's row.
            const double numeric = -(rp[a] - rm[a]) / (2.0 * h);
            if (std::abs(numeric - J[a * nDof + j]) > worst) {
                worst = std::abs(numeric - J[a * nDof + j]);
                if (worstRow != nullptr) {
                    *worstRow = static_cast<int>(a);
                }
                if (worstDof != nullptr) {
                    *worstDof = static_cast<int>(j);
                }
            }
        }
    }
    return worst;
}

} // namespace pose
