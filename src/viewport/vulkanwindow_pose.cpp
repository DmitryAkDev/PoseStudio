/**
 * @file vulkanwindow_pose.cpp
 * @brief VulkanWindow's discrete pose edits — utilities, pins, pose files, delete — and the
 *        unified undo/redo stack they and the drags commit into.
 *
 * One of the seven translation units of VulkanWindow — see vulkanwindow.h for the map. Every
 * pose edit here is BRACKETED the same way: refuse while a button-held drag owns the pose
 * (dragInFlight), close every self-completing edit (closeOpenPoseEdits — the release settle,
 * the fall, an X/Y/Z hold — each commits its own entry), snapshot m_preEditPose, edit, run the
 * settled-pose hook, commitPoseUndo (a no-op when nothing changed), request a frame. The undo
 * stack is ONE chronological list of pose and lighting entries; each pose entry records the
 * figure it belongs to, so undo re-activates that figure before applying it.
 */

#include "vulkanwindow.h"

#include "rendering/vulkanrenderer.h"
#include "scene/scene.h"

#include <utility>

namespace pose {

void VulkanWindow::closeSettlingEdits() {
    finishIkSettle();
    finishGroundFall();
    endAxisRotate();
}

void VulkanWindow::closeOpenPoseEdits() {
    closeSettlingEdits();
    if (m_ik.dragging) {
        abortIkDrag(); // a stale drag (or a delete mid-gesture): no settle, the entry is kept
    }
    if (m_posingBone) {
        endFkDrag();
    }
}

void VulkanWindow::commitPoseUndo() {
    if (m_renderer && scene().capturePose() != m_preEditPose) {
        UndoEntry entry;
        entry.kind = UndoEntry::Kind::Pose;
        entry.pose = m_preEditPose;
        entry.figure = scene().activeFigureIndex();
        m_undoStack.push_back(std::move(entry));
        m_redoStack.clear();
    }
}

void VulkanWindow::runPoseUtility(PoseUtility what) {
    if (!m_renderer || dragInFlight()) {
        return; // not mid-gesture: the rig captured its pins and pose at drag start
    }
    closeOpenPoseEdits(); // land the previous release / hold first; this edit starts from its result
    m_preEditPose = scene().capturePose();
    switch (what) {
    case PoseUtility::ResetJoint:
        scene().resetSelectedJoint(false);
        break;
    case PoseUtility::ResetLimb:
        scene().resetSelectedJoint(true);
        break;
    case PoseUtility::ResetPose:
        scene().resetPose();
        break;
    case PoseUtility::MirrorPose:
        scene().mirrorPose();
        break;
    case PoseUtility::MirrorLimb:
        scene().mirrorSelectedLimb();
        break;
    }
    scene().finalizePose(); // the settled-pose hook (correctives already follow per frame)
    commitPoseUndo();       // no-op if nothing changed (no selection, already at rest)
    requestUpdate();
}

void VulkanWindow::resetSelectedJoint() { runPoseUtility(PoseUtility::ResetJoint); }
void VulkanWindow::resetSelectedLimb() { runPoseUtility(PoseUtility::ResetLimb); }
void VulkanWindow::resetPose() { runPoseUtility(PoseUtility::ResetPose); }
void VulkanWindow::mirrorPose() { runPoseUtility(PoseUtility::MirrorPose); }
void VulkanWindow::mirrorSelectedLimb() { runPoseUtility(PoseUtility::MirrorLimb); }

void VulkanWindow::togglePinSelectedJoint() {
    if (!m_renderer || dragInFlight()) {
        return; // the rig captured its pins at drag start
    }
    closeOpenPoseEdits();
    m_preEditPose = scene().capturePose(); // a pin toggle is an undoable pose edit
    scene().togglePinSelectedBone();
    commitPoseUndo();
    requestUpdate();
}

void VulkanWindow::unpinAllJoints() {
    if (!m_renderer || dragInFlight()) {
        return;
    }
    closeOpenPoseEdits();
    m_preEditPose = scene().capturePose();
    scene().unpinAllBones();
    commitPoseUndo();
    requestUpdate();
}

void VulkanWindow::deleteSelectedObject() {
    if (m_renderer) {
        deleteModel(scene().selectedModelIndex());
    }
}

void VulkanWindow::deleteModel(int index) {
    if (!m_renderer || index < 0) {
        return;
    }
    // Deleting the dragged figure mid-gesture (left button still held while the context menu
    // opened), or with a release settle / fall / axis hold still animating: end them cleanly
    // first — a settle or fall would otherwise keep ticking past the delete and retarget to
    // whatever figure remains.
    closeOpenPoseEdits();
    m_renderer->deleteModel(static_cast<std::size_t>(index));
    // Model indices shift and the deleted figure's poses are meaningless: the pose history goes
    // with it (lighting entries too — one chronological stack).
    m_undoStack.clear();
    m_redoStack.clear();
    requestUpdate();
}

bool VulkanWindow::savePose(const QString& path) {
    closeOpenPoseEdits(); // serialize the LANDED pose, not a transient mid-settle/mid-fall frame
    return m_renderer && scene().savePose(path.toStdString());
}

bool VulkanWindow::loadPose(const QString& path) {
    if (!m_renderer) {
        m_deferred.pendingPose = path; // applied after the queued figure is drained (open-with a .pose)
        return true;
    }
    if (dragInFlight()) {
        return false; // the drag owns the pose until its release
    }
    // An undoable pose edit like any other: the file replaces the whole snapshot (rotations,
    // translations, pins), and Ctrl+Z brings the previous pose back.
    closeOpenPoseEdits(); // don't let a still-animating release settle fight the loaded pose
    m_preEditPose = scene().capturePose();
    if (!scene().loadPose(path.toStdString())) {
        return false;
    }
    scene().finalizePose();
    commitPoseUndo();
    requestUpdate(); // repaint with the restored pose
    return true;
}

void VulkanWindow::registerLightingUndo(const LightingSettings& preEdit) {
    UndoEntry entry;
    entry.kind = UndoEntry::Kind::Lighting;
    entry.lighting = preEdit;
    m_undoStack.push_back(std::move(entry));
    m_redoStack.clear(); // a fresh edit invalidates the redo branch, same as a pose edit
}

void VulkanWindow::swapUndoEntry(std::vector<UndoEntry>& from, std::vector<UndoEntry>& to) {
    if (dragInFlight()) {
        return; // mid-gesture (Ctrl+Z with the button held): the drag owns the pose right now
    }
    closeOpenPoseEdits(); // a pending release settle / axis hold must commit its own entry first
    if (!m_renderer || from.empty()) {
        return;
    }
    UndoEntry entry = std::move(from.back());
    from.pop_back();
    UndoEntry counter; // the state this entry replaces — what the opposite direction restores
    counter.kind = entry.kind;
    if (entry.kind == UndoEntry::Kind::Pose) {
        scene().setActiveFigure(entry.figure); // the snapshot belongs to that figure
        counter.figure = entry.figure;
        counter.pose = scene().capturePose();
        scene().applyPose(entry.pose);
        scene().finalizePose(); // re-runs correctives, like any pose change
    } else {
        counter.lighting = m_deferred.lighting; // the current dials
        setLightingSettings(entry.lighting);
        emit lightingRestored(entry.lighting); // the Environment panel syncs its widgets
    }
    to.push_back(std::move(counter));
    requestUpdate();
}

void VulkanWindow::undo() {
    swapUndoEntry(m_undoStack, m_redoStack);
}

void VulkanWindow::redo() {
    swapUndoEntry(m_redoStack, m_undoStack);
}

} // namespace pose
