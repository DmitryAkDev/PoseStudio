/**
 * @file jointtransform.h
 * @brief What the Transform tab shows for the selected joint: its name and its three dials.
 *
 * A plain-data contract between the scene and the properties panel (like LightingSettings): the
 * scene fills one from the active figure's selected joint (Scene::selectedJointTransform), the
 * viewport hands it out, the panel mirrors it into its rows. The dials themselves — which channel
 * each turns, its scale and its range — are the armature's (Armature::jointDial).
 */

#ifndef JOINTTRANSFORM_H
#define JOINTTRANSFORM_H

#include <string>

namespace pose {

struct JointTransform {
    /// One row: a dial on the 0-100 scale of its joint's limit (see JointDial). `enabled` is
    /// false for a motion the joint does not have (a locked channel).
    struct Dial {
        bool  enabled = false;
        float value = 0.0f;
        float minValue = 0.0f;
        float maxValue = 0.0f;
        bool operator==(const Dial& o) const {
            return enabled == o.enabled && value == o.value && minValue == o.minValue && maxValue == o.maxValue;
        }
    };

    bool        valid = false; ///< A joint is selected on the posable figure.
    std::string name;          ///< The joint's name (the bend bone's, for a selected twist bone).
    Dial        dials[3];      ///< Indexed by JointDialKind: BendForward, BendSideways, Twist (the tab labels them Bend, Side-Side, Twist).

    bool operator==(const JointTransform& o) const {
        return valid == o.valid && name == o.name && dials[0] == o.dials[0] && dials[1] == o.dials[1] &&
               dials[2] == o.dials[2];
    }
    bool operator!=(const JointTransform& o) const { return !(*this == o); }
};

} // namespace pose

#endif // JOINTTRANSFORM_H
