/**
 * @file ray.h
 * @brief A world-space ray: the picking primitive shared by the camera and the models.
 *
 * Split out of camera.h so that code which only intersects rays (Model::intersectRay, the
 * scene's model picking) doesn't pull in the whole orbit camera. Pure GLM data — no Vulkan,
 * no Qt.
 */

#ifndef RAY_H
#define RAY_H

#include <glm/glm.hpp>

namespace pose {

/// A world-space ray (origin + normalized direction), e.g. for mouse picking.
struct Ray {
    glm::vec3 origin;
    glm::vec3 direction;
};

} // namespace pose

#endif // RAY_H
