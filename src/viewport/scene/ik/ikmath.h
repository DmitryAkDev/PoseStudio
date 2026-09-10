/**
 * @file ikmath.h
 * @brief Small shared rotation math for the IK module (and the Armature's IK pose extraction):
 *        shortest-arc quaternions, signed angles about an axis, and Euler decomposition for the
 *        figure format's arbitrary per-joint rotation orders.
 *
 * eulerMatrix composes the figure format's per-joint Euler pose (M = R(order[0]) · R(order[1]) ·
 * R(order[2]), each a right-handed rotation about a principal axis) and eulerFromMatrix is its
 * exact inverse — what turns a solved IK world rotation back into the per-channel Euler degrees
 * the engine's pose model (and its per-axis anatomical limits) are expressed in. The pair lives
 * together here so the composition the Armature poses with and the decomposition the IK
 * extraction fits with can never drift apart. Qt-free (std + GLM).
 */

#ifndef IKMATH_H
#define IKMATH_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <string>

namespace pose {

/// The rotation carrying unit vector @p from onto unit vector @p to by the shortest arc.
/// Antiparallel inputs rotate 180 degrees about an arbitrary perpendicular axis.
glm::quat shortestArc(const glm::vec3& from, const glm::vec3& to);

/// Signed angle (radians) from @p from to @p to measured about @p axis (right-handed), using the
/// two vectors' projections onto the plane perpendicular to @p axis. Returns 0 when either
/// projection degenerates.
float signedAngleAround(const glm::vec3& from, const glm::vec3& to, const glm::vec3& axis);

/// Builds the rotation matrix of Euler angles @p degrees (per world channel: .x about X, .y about
/// Y, .z about Z) composed in @p order (e.g. "YZX", the figure format's per-joint rotation_order):
/// each listed axis rotation is applied in turn, M = R(order[0]) · R(order[1]) · R(order[2]).
/// Unknown characters in @p order are skipped.
glm::mat4 eulerMatrix(const glm::vec3& degrees, const std::string& order);

/// Decomposes rotation @p m into Euler angles (DEGREES, per world channel: .x about X, .y about Y,
/// .z about Z) such that composing them in @p order (e.g. "YZX", the figure format's per-joint
/// rotation_order) reproduces @p m — the exact inverse of the engine's eulerMatrix. Near gimbal
/// lock the third-applied angle is folded into the first (the standard degenerate-case choice).
glm::vec3 eulerFromMatrix(const glm::mat3& m, const std::string& order);

} // namespace pose

#endif // IKMATH_H
