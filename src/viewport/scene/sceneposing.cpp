/**
 * @file sceneposing.cpp
 * @brief Scene's posing API: thin forwarders to the ACTIVE figure (Scene::figureModel) for
 *        FK/IK posing, pins, the pose utilities, grounding, and the pose snapshot + file.
 *
 * A second translation unit of Scene (no header of its own) so scene.cpp stays the rendering
 * orchestration and this file stays the one-liners. Every function here has the same shape —
 * find the figure, forward, no-op without one — and none of them knows anything the Model /
 * Armature don't; new posing behaviour belongs in the armature, never here (see armature.h).
 */

#include "scene.h"

#include "model.h"
#include "posefile.h"

namespace pose {

void Scene::nudgeSelectedBone(const glm::vec3& deltaEulerDegrees) {
    if (Model* fig = figureModel()) {
        fig->nudgeSelectedBone(deltaEulerDegrees);
    }
}

void Scene::finalizePose() {
    if (Model* fig = figureModel()) {
        fig->refreshCorrectives();
    }
}

bool Scene::beginBoneIkDrag() {
    Model* fig = figureModel();
    return fig != nullptr && fig->beginIkDrag();
}

bool Scene::dragBoneIkTo(const glm::vec3& targetWorld) {
    Model* fig = figureModel();
    return fig && fig->dragIkTo(targetWorld);
}

bool Scene::settleBoneIkTick() {
    Model* fig = figureModel();
    return fig && fig->settleIkTick();
}

void Scene::endBoneIkDrag() {
    if (Model* fig = figureModel()) {
        fig->endIkDrag();
    }
}

bool Scene::selectedBoneWorldPosition(glm::vec3& out) const {
    const Model* fig = figureModel();
    if (!fig || fig->selectedBone() < 0 ||
        fig->selectedBone() >= static_cast<int>(fig->boneCount())) {
        return false;
    }
    out = fig->boneWorldPosition(static_cast<std::size_t>(fig->selectedBone()));
    return true;
}

bool Scene::figureGroundGap(float& lowestY) const {
    const Model* fig = figureModel();
    return fig != nullptr && fig->groundGap(lowestY);
}

void Scene::translateModelY(int index, float dy) {
    if (index >= 0 && static_cast<std::size_t>(index) < m_models.size()) {
        m_models[static_cast<std::size_t>(index)]->translateY(dy);
    }
}

bool Scene::togglePinSelectedBone() {
    Model* fig = figureModel();
    return fig && fig->togglePinSelectedBone();
}

bool Scene::selectedBonePinned() const {
    const Model* fig = figureModel();
    return fig && fig->selectedBonePinned();
}

bool Scene::hasPinnedBones() const {
    const Model* fig = figureModel();
    return fig && fig->hasPinnedBones();
}

void Scene::unpinAllBones() {
    if (Model* fig = figureModel()) {
        fig->unpinAllBones();
    }
}

bool Scene::resetSelectedJoint(bool subtree) {
    Model* fig = figureModel();
    return fig && fig->resetSelectedBone(subtree);
}

void Scene::resetPose() {
    if (Model* fig = figureModel()) {
        fig->resetPose();
    }
}

void Scene::mirrorPose() {
    if (Model* fig = figureModel()) {
        fig->mirrorPose();
    }
}

bool Scene::mirrorSelectedLimb() {
    Model* fig = figureModel();
    return fig && fig->mirrorSelectedLimb();
}

std::vector<std::pair<std::string, glm::vec3>> Scene::capturePose() const {
    const Model* fig = figureModel();
    return fig ? fig->capturePose() : std::vector<std::pair<std::string, glm::vec3>>{};
}

void Scene::applyPose(const std::vector<std::pair<std::string, glm::vec3>>& pose) {
    if (Model* fig = figureModel()) {
        fig->applyPose(pose);
    }
}

bool Scene::savePose(const std::string& path) const {
    // The whole snapshot goes to the file — rotations plus the @trans:/@pin: rows (posefile.h).
    const Model* fig = figureModel();
    return fig != nullptr && writePoseFile(path, fig->capturePose());
}

bool Scene::loadPose(const std::string& path) {
    Model* fig = figureModel();
    if (!fig) {
        return false;
    }
    PoseRows pose;
    if (!readPoseFile(path, pose)) {
        return false; // unreadable or malformed: the figure's pose is left untouched
    }
    fig->applyPose(pose);
    return true;
}

} // namespace pose
