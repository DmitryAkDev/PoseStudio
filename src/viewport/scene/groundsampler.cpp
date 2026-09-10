/**
 * @file groundsampler.cpp
 * @brief The ground-sample store and the posed lowest-point scan. See groundsampler.h.
 */

#include "groundsampler.h"

#include "armature.h"
#include "vertex.h"

#include <algorithm>
#include <limits>

namespace pose {

void GroundSampler::reserve(std::size_t vertexCount) { m_samples.reserve(vertexCount); }

void GroundSampler::add(const std::vector<Vertex>& vertices) {
    for (const Vertex& v : vertices) {
        m_samples.push_back({v.pos, v.joints, v.weights});
    }
}

bool GroundSampler::lowestY(const Armature& armature, const glm::vec3& boundsMin,
                            const glm::vec3& boundsMax, bool hasBounds, float& out) const {
    float minY = std::numeric_limits<float>::max();
    if (armature.hasSkeleton() && !m_samples.empty()) {
        // Posed lowest point: CPU-skin every ground sample with the CURRENT pose's skin matrices
        // (poseGlobal · inverseBind — the armature's poseGlobal always holds the last pose
        // update) and track the world-space minimum. A one-shot ~few-ms walk; corrective deltas
        // are ignored (they move contact regions by millimetres at most). Linear matrix blending
        // is fine here even though the GPU skins with dual quaternions: contact regions (feet,
        // knees) are near-single-joint weighted, where LBS and DQS agree exactly.
        const std::size_t boneCount = armature.boneCount();
        std::vector<glm::mat4> skin(boneCount);
        for (std::size_t i = 0; i < boneCount; ++i) {
            skin[i] = armature.poseGlobal(i) * armature.inverseBind(i);
        }
        for (const GroundSample& s : m_samples) {
            const glm::vec4 p(s.pos, 1.0f);
            glm::vec3 posed(0.0f);
            for (int j = 0; j < 4; ++j) {
                const float w = s.weights[j];
                if (w > 0.0f && s.joints[j] < skin.size()) {
                    posed += w * glm::vec3(skin[s.joints[j]] * p);
                }
            }
            minY = std::min(minY, (armature.transform() * glm::vec4(posed, 1.0f)).y);
        }
    } else if (hasBounds) {
        // Static model: the bind AABB through the transform is exact (it can't pose).
        for (int i = 0; i < 8; ++i) {
            const glm::vec3 c((i & 1) ? boundsMax.x : boundsMin.x, (i & 2) ? boundsMax.y : boundsMin.y,
                              (i & 4) ? boundsMax.z : boundsMin.z);
            minY = std::min(minY, (armature.transform() * glm::vec4(c, 1.0f)).y);
        }
    } else {
        return false; // no geometry to ground
    }
    if (!(minY < std::numeric_limits<float>::max())) {
        return false; // nothing sampled
    }
    out = minY;
    return true;
}

} // namespace pose
