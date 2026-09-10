/**
 * @file groundsampler.h
 * @brief A compact CPU copy of a skinned model's vertices, for finding the CURRENT pose's lowest
 *        point (the ground button / the animated ground drop).
 *
 * The GPU vertex buffers are device-local and unreadable, and a posed figure's lowest point is
 * not its bind box's: a knee bend lifts the feet, so grounding by the bind AABB would sink or
 * float the figure. So every render vertex's skinning inputs (position + joints + weights,
 * ~44 bytes/vertex, ~10 MB for a figure) are kept here and CPU-skinned on demand with the
 * armature's live pose. Static models need no samples: their bind AABB through the transform is
 * exact, and lowestY() handles that case from the bounds alone. Qt-free, Vulkan-free.
 */

#ifndef GROUNDSAMPLER_H
#define GROUNDSAMPLER_H

#include <glm/glm.hpp>

#include <cstddef>
#include <vector>

namespace pose {

class Armature;
struct Vertex;

class GroundSampler {
public:
    /// Reserves room for @p vertexCount samples. Reserve ONCE, summed across meshes: an
    /// exact-fit reserve per mesh would reallocate-and-copy the whole store every mesh (exact-fit
    /// defeats geometric growth), turning a figure's ~10 MB of samples into O(zones × total)
    /// redundant copying.
    void reserve(std::size_t vertexCount);
    /// Appends every vertex of @p vertices (its position, joints, and weights).
    void add(const std::vector<Vertex>& vertices);
    bool empty() const { return m_samples.empty(); }

    /// The world height of the CURRENT pose's lowest point: positive = hovering that far above
    /// the floor, negative = sunk into it. With samples, CPU-skins every sample with
    /// @p armature's live pose; without (a static model), uses the bind AABB (@p boundsMin /
    /// @p boundsMax, valid when @p hasBounds) through the transform, which is exact for a model
    /// that can't pose. False when there is nothing to measure.
    bool lowestY(const Armature& armature, const glm::vec3& boundsMin, const glm::vec3& boundsMax,
                 bool hasBounds, float& out) const;

private:
    struct GroundSample {
        glm::vec3  pos;
        glm::uvec4 joints;
        glm::vec4  weights;
    };
    std::vector<GroundSample> m_samples;
};

} // namespace pose

#endif // GROUNDSAMPLER_H
