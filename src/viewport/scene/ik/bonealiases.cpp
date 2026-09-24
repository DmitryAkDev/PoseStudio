/**
 * @file bonealiases.cpp
 * @brief The bone-alias table (see bonealiases.h): reference-generation names -> the other
 *        generations' names, tried in order after the name itself.
 */

#include "bonealiases.h"

#include "armature.h"

#include <cstddef>

namespace pose {

namespace {

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
    // oldest generations have none — the harness's leg diagnostics then skip the twist metric.
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

int aliasedBoneIndex(const Armature& arm, const std::string& canonical) {
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

std::string aliasedBoneName(const Armature& arm, const std::string& canonical) {
    const int idx = aliasedBoneIndex(arm, canonical);
    return idx >= 0 ? arm.boneName(static_cast<std::size_t>(idx)) : canonical;
}

} // namespace pose
