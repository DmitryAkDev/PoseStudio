/**
 * @file armature.cpp
 * @brief Armature's skeleton construction, forward kinematics, pose snapshots, and the skeleton
 *        dump/load used by the IK harness. The full-body-IK integration lives in armatureik.cpp.
 */

#include "armature.h"

#include "ikmath.h" // eulerMatrix (the pose composition)
#include "ikrig.h"  // complete type for the unique_ptr

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cmath>
#include <fstream>
#include <sstream>

namespace pose {

namespace {

// Pose-snapshot rows carrying a bone's pose TRANSLATION rather than a rotation are keyed
// "@trans:<boneName>" — flowing through the existing (name, vec3) snapshot/undo/.pose-file
// machinery unchanged (bone names never contain '@', and readers ignore unknown names, so old
// files load and old builds skip the rows). Written by FBIK for the skeleton root: rotations
// alone can't move it, and a feet-pinned crouch must drop the hip.
constexpr char kPoseTranslationPrefix[] = "@trans:";
// Likewise "@pin:<boneName>" rows carry the user's joint PINS (value unused, written as 1 0 0):
// a pin is part of the pose snapshot, so pin toggles are undoable, a drag's undo restores the
// pins of that moment, and a .pose file reproduces its pins. Old builds skip the rows; a file
// without them loads with no pins.
constexpr char kPosePinPrefix[] = "@pin:";

} // namespace

Armature::Armature() = default;
Armature::~Armature() = default;
Armature::Armature(Armature&&) noexcept = default;
Armature& Armature::operator=(Armature&&) noexcept = default;

namespace {

/// The other side's name for a left/right-prefixed bone name, or "" for a centre/unpaired one.
/// Figures prefix `l`/`r` before an uppercase letter or underscore (`lShin`, `l_thigh`); a few
/// use `Left`/`Right`. A plain lowercase continuation (`lowerJaw`, `rectus`) is not a side.
std::string mirroredBoneName(const std::string& name) {
    if (name.size() >= 2 && (name[0] == 'l' || name[0] == 'r')) {
        const char next = name[1];
        if ((next >= 'A' && next <= 'Z') || next == '_') {
            std::string other = name;
            other[0] = name[0] == 'l' ? 'r' : 'l';
            return other;
        }
    }
    static const char* const kWords[][2] = {{"Left", "Right"}, {"Right", "Left"},
                                            {"left", "right"}, {"right", "left"}};
    for (const auto& pair : kWords) {
        const std::string from(pair[0]);
        if (name.size() > from.size() && name.compare(0, from.size(), from) == 0) {
            return pair[1] + name.substr(from.size());
        }
    }
    return {};
}

} // namespace

void Armature::build(const std::vector<ArmatureBone>& bones) {
    m_bones.clear();
    m_boneIndex.clear();
    m_boneNames.clear();
    m_ikRig.reset();
    m_ikChildren.clear();
    m_ikBindPos.clear();
    m_selectedBone = -1;

    // inverseBind and localBind are precomputed; computeSkinMatrices() then fills the skin data —
    // every skin transform is identity at bind pose, so a freshly imported model draws exactly as
    // its geometry.
    m_bones.resize(bones.size());
    std::vector<glm::vec3> bindGlobal(bones.size(), glm::vec3(0.0f));
    for (std::size_t i = 0; i < bones.size(); ++i) {
        const ArmatureBone& src = bones[i];
        Bone& bone = m_bones[i];
        const bool hasParent = src.parent >= 0 && static_cast<std::size_t>(src.parent) < i;
        bone.parent = hasParent ? src.parent : -1;
        const glm::vec3 parentGlobal =
            hasParent ? bindGlobal[static_cast<std::size_t>(src.parent)] : glm::vec3(0.0f);
        bindGlobal[i] = parentGlobal + src.localBindTranslation;
        bone.localBind = glm::translate(glm::mat4(1.0f), src.localBindTranslation);
        bone.inverseBind = glm::translate(glm::mat4(1.0f), -bindGlobal[i]);
        bone.orient = eulerMatrix(src.orientation, "XYZ"); // rest orientation frame for pose rotations
        bone.invOrient = glm::inverse(bone.orient);
        bone.orientationDeg = src.orientation;
        bone.rotationOrder = src.rotationOrder;
        bone.rotMin = src.rotationMin;
        bone.rotMax = src.rotationMax;
        bone.rotLimited = src.rotationLimited;
        bone.poseLocal = bone.localBind;
        m_boneIndex.emplace(src.name, static_cast<int>(i));
        m_boneNames.push_back(src.name);
    }
    m_boneEuler.assign(m_bones.size(), glm::vec3(0.0f));
    m_boneTranslation.assign(m_bones.size(), glm::vec3(0.0f));
    m_boneWorldPos.assign(m_bones.size(), glm::vec3(0.0f));
    m_bonePinned.assign(m_bones.size(), 0);
    m_poseGlobal.assign(m_bones.size(), glm::mat4(1.0f));

    // Highlight twins (see m_highlightTwin): a child whose two swing axes are LOCKED is a twist
    // bone — pair it with its bend parent both ways. First twist child wins for the parent.
    m_highlightTwin.assign(m_bones.size(), -1);
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        const Bone& b = m_bones[i];
        if (b.parent < 0 || b.parent >= static_cast<int>(m_bones.size())) {
            continue;
        }
        int lockedAxes = 0;
        for (int a = 0; a < 3; ++a) {
            if (b.rotLimited[a] && (b.rotMax[a] - b.rotMin[a]) < 2.0f) {
                ++lockedAxes;
            }
        }
        const auto parent = static_cast<std::size_t>(b.parent);
        if (lockedAxes == 2 && m_highlightTwin[parent] < 0) {
            m_highlightTwin[parent] = static_cast<int>(i);
            m_highlightTwin[i] = b.parent;
        }
    }

    // Children lists (subtree walks for the reset/mirror utilities) and each bone's mirror —
    // the other side's bone by name, itself for centre bones and names without a counterpart.
    m_children.assign(m_bones.size(), {});
    m_mirrorBone.resize(m_bones.size());
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        if (m_bones[i].parent >= 0) {
            m_children[static_cast<std::size_t>(m_bones[i].parent)].push_back(static_cast<int>(i));
        }
        const std::string other = mirroredBoneName(m_boneNames[i]);
        const int j = other.empty() ? -1 : boneIndex(other);
        m_mirrorBone[i] = j >= 0 ? j : static_cast<int>(i);
    }

    // Skin data: two vec4s per joint (a static armature has one identity joint).
    const std::uint32_t joints = jointCount();
    m_skinDualQuats.assign(std::size_t{2} * joints, glm::vec4(0.0f));
    for (std::uint32_t j = 0; j < joints; ++j) {
        m_skinDualQuats[2 * j] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f); // identity rotation
    }
    computeSkinMatrices(); // bind pose until a caller poses a bone
}

bool Armature::dump(const std::string& path) const {
    std::ofstream out(path);
    if (!out) {
        return false;
    }
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        const Bone& b = m_bones[i];
        const glm::vec3 t(b.localBind[3]);
        const glm::vec3& o = b.orientationDeg;
        out << m_boneNames[i] << ' ' << b.parent << ' ' << b.rotationOrder << ' ' << t.x << ' '
            << t.y << ' ' << t.z << ' ' << o.x << ' ' << o.y << ' ' << o.z << ' ' << b.rotMin.x
            << ' ' << b.rotMin.y << ' ' << b.rotMin.z << ' ' << b.rotMax.x << ' ' << b.rotMax.y
            << ' ' << b.rotMax.z << ' ' << (b.rotLimited.x ? 1 : 0) << ' '
            << (b.rotLimited.y ? 1 : 0) << ' ' << (b.rotLimited.z ? 1 : 0) << '\n';
    }
    return static_cast<bool>(out);
}

bool Armature::loadDump(const std::string& path, std::vector<ArmatureBone>& out) {
    out.clear();
    std::ifstream in(path);
    if (!in) {
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::istringstream ss(line);
        ArmatureBone b;
        int lx = 0, ly = 0, lz = 0;
        if (!(ss >> b.name >> b.parent >> b.rotationOrder >> b.localBindTranslation.x >>
              b.localBindTranslation.y >> b.localBindTranslation.z >> b.orientation.x >>
              b.orientation.y >> b.orientation.z >> b.rotationMin.x >> b.rotationMin.y >>
              b.rotationMin.z >> b.rotationMax.x >> b.rotationMax.y >> b.rotationMax.z >> lx >>
              ly >> lz)) {
            out.clear();
            return false;
        }
        b.rotationLimited = glm::bvec3(lx != 0, ly != 0, lz != 0);
        if (b.parent >= static_cast<int>(out.size())) {
            out.clear();
            return false; // parents must precede children
        }
        out.push_back(std::move(b));
    }
    return true;
}

void Armature::setTransform(const glm::mat4& transform) {
    m_transform = transform;
    computeSkinMatrices(); // refresh the transform-dependent bone world positions
}

void Armature::translateY(float dy) {
    m_transform = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, dy, 0.0f)) * m_transform;
    computeSkinMatrices(); // refresh the transform-dependent bone world positions (picking/markers)
}

void Armature::computeSkinMatrices() {
    ++m_skinVersion;
    if (m_skinDualQuats.size() < 2) {
        return; // not built yet
    }
    glm::vec4* dst = m_skinDualQuats.data();
    if (m_bones.empty()) {
        dst[0] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f); // static model: one identity joint
        dst[1] = glm::vec4(0.0f);
        return;
    }
    // Accumulate each bone's posed global transform down the hierarchy (parents precede children),
    // then convert the rigid skinTransform = poseGlobal · inverseBind into a unit dual quaternion
    // (real = rotation stored (x,y,z,w); dual = 0.5·(0,t)·real). The transform is exactly rigid —
    // binds are translation-only and every pose factor is a rotation or translation — so the
    // conversion is lossless. m_poseGlobal is a member scratch buffer — this runs on every
    // drag-move while posing, so it mustn't allocate per call.
    std::vector<glm::mat4>& poseGlobal = m_poseGlobal;
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        const Bone& bone = m_bones[i];
        poseGlobal[i] =
            (bone.parent >= 0) ? poseGlobal[bone.parent] * bone.poseLocal : bone.poseLocal;
        const glm::mat4 skin = poseGlobal[i] * bone.inverseBind;
        const glm::quat q = glm::normalize(glm::quat_cast(glm::mat3(skin)));
        const glm::vec3 t = glm::vec3(skin[3]);
        const glm::quat d = glm::quat(0.0f, t.x, t.y, t.z) * q; // (0,t)·q
        dst[2 * i] = glm::vec4(q.x, q.y, q.z, q.w);
        dst[2 * i + 1] = 0.5f * glm::vec4(d.x, d.y, d.z, d.w);
        // The joint's world position (for the posing overlay / picking) = model · poseGlobal · origin.
        m_boneWorldPos[i] = glm::vec3(m_transform * poseGlobal[i][3]);
    }
}

void Armature::clampBoneEuler(int index) {
    if (index < 0 || index >= static_cast<int>(m_bones.size())) {
        return;
    }
    const Bone& bone = m_bones[static_cast<std::size_t>(index)];
    glm::vec3& e = m_boneEuler[static_cast<std::size_t>(index)];
    for (int a = 0; a < 3; ++a) {
        if (bone.rotLimited[a]) {
            e[a] = glm::clamp(e[a], bone.rotMin[a], bone.rotMax[a]);
        }
    }
}

void Armature::recomposePoseLocal(std::size_t index) {
    Bone& bone = m_bones[index];
    bone.poseLocal = bone.localBind * bone.orient *
                     eulerMatrix(m_boneEuler[index], bone.rotationOrder) * bone.invOrient;
    // Pose translation adds in the parent frame (only ever non-zero where FBIK moved the root).
    bone.poseLocal[3] += glm::vec4(m_boneTranslation[index], 0.0f);
}

void Armature::applyBoneEuler(int index) {
    if (index < 0 || index >= static_cast<int>(m_bones.size())) {
        return;
    }
    clampBoneEuler(index); // keep the joint within its anatomical range of motion
    recomposePoseLocal(static_cast<std::size_t>(index));
    computeSkinMatrices();
}

void Armature::nudgeSelectedBone(const glm::vec3& deltaEulerDegrees) {
    if (m_selectedBone < 0 || m_selectedBone >= static_cast<int>(m_bones.size())) {
        return;
    }
    m_boneEuler[static_cast<std::size_t>(m_selectedBone)] += deltaEulerDegrees;
    applyBoneEuler(m_selectedBone);
}

bool Armature::setBoneRotation(const std::string& boneName, const glm::vec3& eulerDegrees) {
    const int index = boneIndex(boneName);
    if (index < 0) {
        return false;
    }
    // applyBoneEuler poses in the joint's oriented frame: localBind · orient · R(euler, order) ·
    // orient⁻¹. At rest (euler 0) orient·orient⁻¹ cancels, leaving the bind pose; a rotation then acts
    // about the joint's true axes (an elbow bends anatomically), composed in its rotation order.
    m_boneEuler[static_cast<std::size_t>(index)] = eulerDegrees;
    applyBoneEuler(index);
    return true;
}

std::vector<std::pair<std::string, glm::vec3>> Armature::capturePose() const {
    std::vector<std::pair<std::string, glm::vec3>> pose;
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        const glm::vec3& e = m_boneEuler[i];
        if (e.x != 0.0f || e.y != 0.0f || e.z != 0.0f) {
            pose.emplace_back(m_boneNames[i], e);
        }
        const glm::vec3& t = m_boneTranslation[i];
        if (t.x != 0.0f || t.y != 0.0f || t.z != 0.0f) {
            pose.emplace_back(kPoseTranslationPrefix + m_boneNames[i], t);
        }
    }
    for (std::size_t i = 0; i < m_bonePinned.size() && i < m_bones.size(); ++i) {
        if (m_bonePinned[i]) {
            pose.emplace_back(kPosePinPrefix + m_boneNames[i], glm::vec3(1.0f, 0.0f, 0.0f));
        }
    }
    return pose;
}

void Armature::applyPose(const std::vector<std::pair<std::string, glm::vec3>>& pose) {
    for (glm::vec3& e : m_boneEuler) {
        e = glm::vec3(0.0f); // reset to bind pose first
    }
    for (glm::vec3& t : m_boneTranslation) {
        t = glm::vec3(0.0f);
    }
    m_bonePinned.assign(m_bones.size(), 0); // the snapshot's pins replace the current ones
    constexpr std::size_t prefixLen = sizeof(kPoseTranslationPrefix) - 1;
    constexpr std::size_t pinPrefixLen = sizeof(kPosePinPrefix) - 1;
    for (const auto& [name, value] : pose) {
        if (name.compare(0, pinPrefixLen, kPosePinPrefix) == 0) {
            const int index = boneIndex(name.substr(pinPrefixLen));
            if (index >= 0) {
                m_bonePinned[static_cast<std::size_t>(index)] = 1;
            }
            continue;
        }
        if (name.compare(0, prefixLen, kPoseTranslationPrefix) == 0) {
            const int index = boneIndex(name.substr(prefixLen));
            if (index >= 0) {
                // SANITIZE: unlike rotations (per-channel clamped below), translations have no
                // authored limits — a hand-edited/corrupt .pose row could tear a bone meters
                // off the skeleton or stream-parse to NaN, which poisons the skin matrices.
                // The engine itself only ever writes modest root offsets; a ±1m box per
                // component is far beyond any legitimate value.
                glm::vec3 t = value;
                if (!std::isfinite(t.x + t.y + t.z)) {
                    t = glm::vec3(0.0f);
                }
                m_boneTranslation[static_cast<std::size_t>(index)] =
                    glm::clamp(t, glm::vec3(-1.0f), glm::vec3(1.0f));
            }
            continue;
        }
        const int index = boneIndex(name);
        if (index >= 0) {
            m_boneEuler[static_cast<std::size_t>(index)] = value;
        }
    }
    // Recompute every bone's posed local transform from its (limit-clamped) Euler, then the skin
    // data once. Clamping here too keeps a loaded pose within the figure's range of motion.
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        clampBoneEuler(static_cast<int>(i));
        recomposePoseLocal(i);
    }
    computeSkinMatrices();
}

void Armature::collectSubtree(int index, std::vector<int>& out) const {
    if (index < 0 || index >= static_cast<int>(m_bones.size())) {
        return;
    }
    out.push_back(index);
    for (const int child : m_children[static_cast<std::size_t>(index)]) {
        collectSubtree(child, out);
    }
}

void Armature::reposeAll() {
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        clampBoneEuler(static_cast<int>(i));
        recomposePoseLocal(i);
    }
    computeSkinMatrices();
}

namespace {
// The sagittal-plane reflection of a pose rotation, per Euler channel, and of a parent-frame
// translation. Reflecting a rotation about an axis flips its angle and mirrors the axis: the
// x axis maps to -x (angle flip cancels), y and z stay (angle flips) — so (x, -y, -z), in
// any rotation order, since each factor transforms independently.
glm::vec3 mirrorEuler(const glm::vec3& e) { return glm::vec3(e.x, -e.y, -e.z); }
glm::vec3 mirrorTranslation(const glm::vec3& t) { return glm::vec3(-t.x, t.y, t.z); }
} // namespace

bool Armature::resetBone(int index, bool subtree) {
    if (index < 0 || index >= static_cast<int>(m_bones.size())) {
        return false;
    }
    std::vector<int> bones;
    if (subtree) {
        collectSubtree(index, bones);
    } else {
        bones.push_back(index);
    }
    for (const int b : bones) {
        m_boneEuler[static_cast<std::size_t>(b)] = glm::vec3(0.0f);
        m_boneTranslation[static_cast<std::size_t>(b)] = glm::vec3(0.0f);
    }
    reposeAll();
    return true;
}

void Armature::resetPose() {
    for (glm::vec3& e : m_boneEuler) {
        e = glm::vec3(0.0f);
    }
    for (glm::vec3& t : m_boneTranslation) {
        t = glm::vec3(0.0f);
    }
    reposeAll();
}

void Armature::mirrorPose() {
    // Read the whole pose first: a left/right pair writes each other's slot.
    const std::vector<glm::vec3> euler = m_boneEuler;
    const std::vector<glm::vec3> translation = m_boneTranslation;
    for (std::size_t i = 0; i < m_bones.size(); ++i) {
        const auto j = static_cast<std::size_t>(m_mirrorBone[i]);
        m_boneEuler[j] = mirrorEuler(euler[i]);
        m_boneTranslation[j] = mirrorTranslation(translation[i]);
    }
    reposeAll();
}

bool Armature::mirrorSubtreeToOpposite(int index) {
    if (index < 0 || index >= static_cast<int>(m_bones.size())) {
        return false;
    }
    std::vector<int> bones;
    collectSubtree(index, bones);
    const std::vector<glm::vec3> euler = m_boneEuler;
    const std::vector<glm::vec3> translation = m_boneTranslation;
    for (const int b : bones) {
        const auto i = static_cast<std::size_t>(b);
        const auto j = static_cast<std::size_t>(m_mirrorBone[i]);
        m_boneEuler[j] = mirrorEuler(euler[i]);
        m_boneTranslation[j] = mirrorTranslation(translation[i]);
    }
    reposeAll();
    return true;
}

} // namespace pose
