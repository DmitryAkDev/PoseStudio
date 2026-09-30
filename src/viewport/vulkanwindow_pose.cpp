/**
 * @file vulkanwindow_pose.cpp
 * @brief VulkanWindow's discrete pose edits — utilities, pins, pose files, delete, the Transform
 *        tab's dials — and the unified undo/redo stack they and the drags commit into.
 *
 * One of the seven translation units of VulkanWindow — see vulkanwindow.h for the map. Every
 * pose edit here is BRACKETED the same way: refuse while a button-held drag owns the pose
 * (dragInFlight), close every self-completing edit (closeOpenPoseEdits — the release settle,
 * the fall, an open dial edit — each commits its own entry), snapshot m_preEditPose, edit, run the
 * settled-pose hook, commitPoseUndo (a no-op when nothing changed), request a frame. The undo
 * stack is ONE chronological list of pose and lighting entries; each pose entry records the
 * figure it belongs to, so undo re-activates that figure before applying it.
 */

#include "vulkanwindow.h"

#include "rendering/vulkanrenderer.h"
#include "scene/scene.h"

#include <QCursor>

#include <algorithm>
#include <cmath>
#include <utility>

namespace pose {

namespace {
/// THE JOINT MOUSE MODE's feel (jointModalMove). Where a dial's swing crosses the screen the limb
/// follows the mouse one to one — but never faster than a lever this long would (in pixels, per
/// radian): a finger seen with the whole figure in view is a few pixels long, and followed one to
/// one a pixel of mouse would be several degrees of knuckle. Also the rate of the plain
/// left-right fallback: 150 pixels of mouse a radian, 2.6 pixels a degree.
constexpr double kModalMinLeverPx = 150.0;
/// WHICH WAY the mouse turns a dial is settled when the mode begins, and kept for the gesture: a
/// bend whose swing crosses the screen with a lever of this many pixels or more is FOLLOWED (the
/// limb goes the way the mouse goes); one that goes toward or away from the camera — and every
/// twist, which turns the limb about its own length — is turned by the mouse's left-right, right
/// for more. (Settled afresh at every move it flipped mid-gesture: a thigh flexed in the front
/// view starts straight at the camera and comes up the screen as it rises, and the mouse that
/// had been bending it by going right stopped doing anything.)
constexpr double kModalFollowFromPx = 40.0;
/// ... and a followed swing that turns toward the camera mid-gesture keeps its last direction
/// once its lever on screen is shorter than this (a vector that short points nowhere).
constexpr double kModalMinDirectionPx = 12.0;
constexpr double kDegreesPerRadian = 57.29577951308232;
} // namespace

void VulkanWindow::closeSettlingEdits() {
    finishIkSettle();
    finishGroundFall();
    endJointTransformEdit(); // (the joint mouse mode's too: what it made is kept)
}

void VulkanWindow::closeOpenPoseEdits() {
    closeSettlingEdits();
    if (m_ik.dragging) {
        abortIkDrag(); // a stale drag (or a delete mid-gesture): no settle, the entry is kept
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

// --- The Transform tab's dials -------------------------------------------------------------------

JointTransform VulkanWindow::jointTransform() const {
    return m_renderer ? scene().selectedJointTransform() : JointTransform{};
}

bool VulkanWindow::beginJointTransformEdit() {
    if (m_transformEdit) {
        return true; // already open (a set opened it, or a nested begin)
    }
    if (!m_renderer || dragInFlight()) {
        return false; // not mid-gesture: the drag owns the pose until its release
    }
    closeOpenPoseEdits(); // land a release settle / fall first; this edit starts from its result
    m_preEditPose = scene().capturePose();
    m_transformEdit = true;
    return true;
}

void VulkanWindow::setJointTransformDial(int dial, double value) {
    if (!beginJointTransformEdit()) {
        return;
    }
    // (Limits clamp and the body volumes stop it inside; correctives follow live on the GPU.)
    if (scene().setSelectedJointDial(dial, static_cast<float>(value))) {
        requestUpdate();
    }
}

void VulkanWindow::endJointTransformEdit() {
    if (!m_transformEdit) {
        return;
    }
    m_transformEdit = false;
    const bool modal = m_modalKind >= 0; // the joint mouse mode is this bracket: it ends with it
    m_modalKind = -1;
    if (m_renderer) {
        scene().finalizePose(); // the settled-pose hook (correctives already followed the dial live)
        commitPoseUndo();       // no-op if the dial never moved the pose
        requestUpdate();
    }
    if (modal) {
        emit jointModalChanged(-1);
    }
}

// --- The joint mouse mode (B / S / T) ------------------------------------------------------------

bool VulkanWindow::beginJointModal(int kind, const QPointF& cursor) {
    if (!m_renderer || kind < 0 || kind >= kJointDialCount) {
        return false;
    }
    if (m_modalKind >= 0) {
        // The same key again drops the mode; another of the three drops it and begins that one.
        const bool same = m_modalKind == kind;
        cancelJointModal();
        if (same) {
            return false;
        }
    }
    if (dragInFlight()) {
        return false; // the drag owns the pose (and the mouse) until its release
    }
    closeSettlingEdits(); // a release settle, a fall, the tab's own open edit: this one starts from their result
    const JointTransform now = jointTransform();
    if (!now.valid || !now.dials[kind].enabled) {
        return false; // nothing selected, or a motion the joint does not have (a hinge elbow's side bend)
    }
    if (!beginJointTransformEdit()) {
        return false;
    }
    m_modalKind = kind;
    m_modalCursor = cursor;
    QPointF direction;
    double  leverPx = 0.0;
    m_modalFollow = kind != static_cast<int>(JointDialKind::Twist) && jointModalSwing(direction, leverPx) &&
                    leverPx >= kModalFollowFromPx;
    m_modalDirection = m_modalFollow ? direction : QPointF(1.0, 0.0);
    emit jointModalChanged(kind);
    return true;
}

bool VulkanWindow::toggleJointModal(int kind) {
    // (A scripted test's cursor stands in the middle of the view: the desktop's is somebody else's.)
    const QPointF cursor = m_script ? QPointF(width() * 0.5, height() * 0.5) : QPointF(mapFromGlobal(QCursor::pos()));
    return beginJointModal(kind, cursor);
}

void VulkanWindow::moveJointModalTo(const QPointF& globalPos) {
    jointModalMove(mapFromGlobal(globalPos)); // the viewport's own frame, as its mouseMoveEvent reads it
}

bool VulkanWindow::jointModalSwing(QPointF& direction, double& leverPx) const {
    JointDialSweep sweep;
    QPointF        from, to;
    if (m_modalKind < 0 || !m_renderer || !scene().selectedJointDialSweep(m_modalKind, sweep) ||
        !projectToScreen(sweep.tip, from) || !projectToScreen(sweep.tip + sweep.tipPerDegree, to)) {
        return false;
    }
    const QPointF perDegree = to - from;
    const double  length = std::hypot(perDegree.x(), perDegree.y());
    leverPx = length * kDegreesPerRadian;
    if (length <= 0.0) {
        return false;
    }
    direction = perDegree / length;
    return true;
}

void VulkanWindow::jointModalMove(const QPointF& cursor) {
    if (m_modalKind < 0 || !m_renderer) {
        return;
    }
    const QPointF move = cursor - m_modalCursor;
    m_modalCursor = cursor;
    if (move.isNull()) {
        return;
    }
    const JointTransform now = jointTransform();
    JointDialSweep       sweep;
    if (!now.valid || !now.dials[m_modalKind].enabled || !scene().selectedJointDialSweep(m_modalKind, sweep) ||
        sweep.degreesPerUnit <= 0.0f) {
        confirmJointModal(); // the joint went away under the mode (another figure, a cleared selection)
        return;
    }
    // THE LIMB GOES THE WAY THE MOUSE GOES: the dial's swing, projected — where the limb's far end
    // moves on screen per degree — is the direction that counts, and along it the end follows the
    // mouse one to one (a lever of so many pixels a radian). Read afresh at every move, so a
    // mouse carried round the joint carries the limb round with it. A dial that was not followed
    // when the mode began (see kModalFollowFromPx) is turned by left-right throughout.
    double leverPx = 0.0;
    if (m_modalFollow) {
        QPointF direction;
        if (jointModalSwing(direction, leverPx) && leverPx >= kModalMinDirectionPx) {
            m_modalDirection = direction;
        } else {
            leverPx = 0.0; // (turned toward the camera: the last direction, at the floor's pace)
        }
    }
    const QPointF direction = m_modalDirection;
    const double  along = move.x() * direction.x() + move.y() * direction.y();
    const double degrees = along / std::max(leverPx, kModalMinLeverPx) * kDegreesPerRadian;
    // (From the value the pose HAS: the limits and the body stop a dial short of what it is
    // asked, and the mouse turned back must move the joint at once, not first undo an overshoot.)
    setJointTransformDial(m_modalKind, now.dials[m_modalKind].value + degrees / sweep.degreesPerUnit);
}

void VulkanWindow::confirmJointModal() {
    if (m_modalKind >= 0) {
        endJointTransformEdit(); // the undo entry; ends the mode with the bracket
    }
}

void VulkanWindow::cancelJointModal() {
    if (m_modalKind < 0) {
        return;
    }
    m_modalKind = -1;
    m_transformEdit = false; // the bracket is dropped, not committed
    if (m_renderer) {
        scene().applyPose(m_preEditPose); // as it was at the key
        scene().finalizePose();
        requestUpdate();
    }
    emit jointModalChanged(-1);
}

void VulkanWindow::notifyJointTransform() {
    JointTransform now = jointTransform();
    if (now != m_announcedJointTransform) {
        m_announcedJointTransform = std::move(now);
        emit jointTransformChanged();
    }
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
    // opened), or with a release settle / fall still animating or a dial edit open: end them cleanly
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
    closeOpenPoseEdits(); // a pending release settle / open dial edit must commit its own entry first
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
