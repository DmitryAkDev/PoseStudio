/**
 * @file vulkanwindow_input.cpp
 * @brief VulkanWindow's mouse, wheel and keyboard handlers, the object context menu, and the
 *        cursor→world helpers.
 *
 * One of the seven translation units of VulkanWindow — see vulkanwindow.h for the map. The
 * handlers here are ROUTING: they decide which gesture a press/move/key belongs to and hand it
 * to the gesture's own code (the IK/fall/axis-hold machinery in vulkanwindow_ik.cpp, the pose
 * edits in vulkanwindow_pose.cpp, the camera in vulkanwindow_camera.cpp). The one rule every
 * branch obeys: a camera move acts only on buttons in m_activeDragButtons — pressed IN this
 * window — because a modal dialog closing over the native child window can leak a button-held
 * move to us and snap the camera (the "event-leak class" of bugs).
 */

#include "vulkanwindow.h"

#include "rendering/vulkanrenderer.h"
#include "scene/scene.h"

#include <QAction>
#include <QGuiApplication>
#include <QKeyEvent>
#include <QMenu>
#include <QMouseEvent>
#include <QStyleHints>
#include <QWheelEvent>

namespace pose {

namespace {
// Input tuning. Kept local for now; promote to Constants / a preference once the
// navigation-preferences panel grows a "viewport" section.
constexpr float kOrbitRadiansPerPixel = 0.008f;
constexpr float kPanPerPixel = 0.0015f;
constexpr float kDollyPerWheelStep = 0.12f; // per 120-unit wheel notch
} // namespace

Ray VulkanWindow::cursorRay(const QPointF& localPos) const {
    return m_renderer->camera().screenPointToRay(
        static_cast<float>(localPos.x()), static_cast<float>(localPos.y()),
        static_cast<float>(width()), static_cast<float>(height()));
}

int VulkanWindow::boneAt(const QPointF& localPos) {
    return scene().selectBoneAt(static_cast<float>(localPos.x()), static_cast<float>(localPos.y()),
                                static_cast<float>(width()), static_cast<float>(height()),
                                m_renderer->camera());
}

void VulkanWindow::mousePressEvent(QMouseEvent* event) {
    // A new interaction must not overlap a still-animating release settle, a figure still
    // falling (a drag captures the floor at its start), or an X/Y/Z wheel hold (its pose
    // snapshot would be overwritten below, and the wheel must mean DEPTH during an IK drag). A
    // live button-held drag is left alone here: a right press while the left button drags is
    // the context-menu-mid-drag case, handled by the menu's actions.
    closeSettlingEdits();
    m_lastMousePos = event->position();
    m_activeDragButtons |= event->button(); // a drag with this button started in the viewport

    // Left-press priority: (0) a joint -> select it + full-body-IK drag of it (the body follows:
    // feet pinned, auto-balanced) — THE posing gesture, no modifier; (1) Ctrl + a joint -> select
    // it + FK-rotate that ONE joint this drag; (2) empty space -> orbit the camera — or, if
    // released without dragging, a CLICK that selects the model under the cursor / clears the
    // selection (see the release).
    if (event->button() == Qt::LeftButton && m_renderer) {
        m_leftClickCandidate = false;
        m_leftPressPos = event->position();
        // A left press while a left-button gesture is still flagged means its release never
        // reached this window (a modal/shortcut swallowed the mouse-up — the m_activeDragButtons
        // event-leak class): close it out, or the 60 Hz timer runs forever and this press hijacks
        // into IK-dragging the old joint, and a stale FK flag would route the coming orbit drag
        // into rotating the selected joint.
        if (m_ik.dragging) {
            abortIkDrag();
        }
        if (m_posingBone) {
            endFkDrag();
        }
        if (!beginJointGesture(event->position(),
                               event->modifiers().testFlag(Qt::ControlModifier))) {
            m_leftClickCandidate = true; // the orbit gesture, until it moves
        }
        requestUpdate(); // reflect the new selection (outline / skeleton highlight)
    }
}

void VulkanWindow::selectModelAtClick(const QPointF& localPos) {
    // Box-level pick (Model::intersectRay): the nearest model whose bounds the cursor's ray
    // enters, else -1 = clear the selection. Selecting a figure also makes it the posing target;
    // deselecting drops its joint selection with it (Scene::setSelectedModel).
    const int picked = scene().pickModel(cursorRay(localPos));
    if (picked != scene().selectedModelIndex()) {
        scene().setSelectedModel(picked);
        requestUpdate(); // the outline moved / went away
    }
}

void VulkanWindow::mouseReleaseEvent(QMouseEvent* event) {
    m_activeDragButtons &= ~event->button();
    if (event->button() == Qt::LeftButton) {
        if (dragInFlight() && m_renderer) {
            if (m_ik.dragging && !m_ik.poseChanged) {
                // A CLICK on a joint: it was selected but no solve ever moved the pose — end the
                // drag with no settle and no undo (the settle would still walk a hovering
                // figure onto its ground-healed pins, turning a mere selection into an edit).
                abortIkDrag();
            } else if (m_ik.dragging) {
                // IK release: hand off to the ANIMATED settle — the timer keeps ticking, each
                // tick relaxing the body onto its pins so hovering feet visibly land instead of
                // popping in one frame. The settled-pose hook + the undo commit run when it
                // finishes (finishIkSettle).
                beginIkRelease();
            } else {
                endFkDrag(); // FK edit settled: the settled-pose hook + an undo entry if changed
            }
            requestUpdate();
        } else if (m_leftClickCandidate && m_renderer) {
            // The orbit gesture that never moved: a CLICK. Within the platform's drag distance
            // it selects what's under the cursor (or clears the selection on empty space); a
            // real orbit — however short — leaves the selection alone, so orbiting around a
            // figure can't deselect it.
            const bool moved = (event->position() - m_leftPressPos).manhattanLength() >=
                               QGuiApplication::styleHints()->startDragDistance();
            if (!moved) {
                selectModelAtClick(event->position());
            }
        }
        m_leftClickCandidate = false;
        m_posingBone = false;
        m_ik.dragging = false;
    }
    // Right-click (on release, the desktop convention) opens the object context menu. The right
    // button drives no camera motion, so there's nothing to disambiguate from a drag here.
    if (event->button() == Qt::RightButton) {
        showObjectContextMenu(event->position(), event->globalPosition().toPoint());
    }
}

void VulkanWindow::showObjectContextMenu(const QPointF& localPos, const QPoint& globalPos) {
    if (!m_renderer) {
        return;
    }
    // A joint under the cursor gets the pin actions (selecting it, so the skeleton overlay —
    // when shown — highlights which joint the menu acts on); the model under the cursor gets
    // Delete. Both can apply. Not while a pose edit owns the joints (a right-click mid-drag).
    const int joint = poseEditInFlight() ? -1 : boneAt(localPos);
    const int picked = scene().pickModel(cursorRay(localPos));
    if (joint < 0 && picked < 0) {
        return; // empty space — no menu (for now)
    }

    QMenu menu;
    QAction* pinAction = nullptr;
    QAction* unpinAllAction = nullptr;
    QAction* resetJointAction = nullptr;
    QAction* resetLimbAction = nullptr;
    QAction* mirrorLimbAction = nullptr;
    if (joint >= 0) {
        pinAction = menu.addAction(scene().selectedBonePinned() ? QStringLiteral("Unpin Joint\tP")
                                                                : QStringLiteral("Pin Joint\tP"));
        if (scene().hasPinnedBones()) {
            unpinAllAction = menu.addAction(QStringLiteral("Unpin All Joints"));
        }
        // Pose utilities on the clicked joint (boneAt selected it): "limb" = the joint and
        // everything below it. The Edit menu carries the same plus the whole-pose variants.
        menu.addSeparator();
        resetJointAction = menu.addAction(QStringLiteral("Reset Joint"));
        resetLimbAction = menu.addAction(QStringLiteral("Reset Limb"));
        mirrorLimbAction = menu.addAction(QStringLiteral("Mirror Limb to Other Side"));
        if (picked >= 0) {
            menu.addSeparator();
        }
    }
    QAction* deleteAction = picked >= 0 ? menu.addAction(QStringLiteral("Delete")) : nullptr;
    QAction* chosen = menu.exec(globalPos);
    if (chosen != nullptr && chosen == resetJointAction) {
        runPoseUtility(PoseUtility::ResetJoint);
    } else if (chosen != nullptr && chosen == resetLimbAction) {
        runPoseUtility(PoseUtility::ResetLimb);
    } else if (chosen != nullptr && chosen == mirrorLimbAction) {
        runPoseUtility(PoseUtility::MirrorLimb);
    } else if (chosen != nullptr && chosen == pinAction) {
        togglePinSelectedJoint();
    } else if (chosen != nullptr && chosen == unpinAllAction) {
        unpinAllJoints();
    } else if (chosen != nullptr && chosen == deleteAction) {
        deleteModel(picked);
    }
    requestUpdate(); // selection highlight / pin markers changed even if nothing was chosen
}

void VulkanWindow::mouseMoveEvent(QMouseEvent* event) {
    if (!m_renderer) {
        return;
    }
    const QPointF delta = event->position() - m_lastMousePos;
    m_lastMousePos = event->position();

    // Only act on buttons that are BOTH held now AND were pressed inside this window. Using
    // event->buttons() alone would let a leaked move (e.g. as the modal Import dialog closes
    // over us with a button still logically down) snap the camera from a stale last-position.
    const Qt::MouseButtons active = m_activeDragButtons & event->buttons();

    Camera& camera = m_renderer->camera();
    if ((active & Qt::LeftButton) && m_ik.dragging) {
        // Full-body IK: the grabbed joint tracks the cursor within the camera-parallel plane
        // through its grab point. The plane's depth changes ONLY by explicit wheel steps
        // (wheelEvent), never from the solve, so it can't feed back.
        glm::vec3 target;
        if (dragPlaneHit(event->position(), target)) {
            setIkTarget(target);
            // Deliberately NO solve here: the 60 Hz timer is the SOLE caller of
            // issueIkTarget(). Solving per mouse event stacked extra solves on top of the
            // timer's — with a high-polling-rate mouse the damped-motion dynamics (all tuned
            // in per-tick units) ran several times faster than designed.
        }
    } else if ((active & Qt::LeftButton) && m_posingBone) {
        // Ctrl+drag, FK: rotate the selected joint alone — horizontal about its Y axis, vertical
        // about X.
        constexpr float kDegPerPixel = 0.4f;
        scene().nudgeSelectedBone(glm::vec3(static_cast<float>(delta.y()) * kDegPerPixel,
                                            static_cast<float>(delta.x()) * kDegPerPixel, 0.0f));
    } else if (active & Qt::LeftButton) {
        // Drag right -> orbit right; drag up -> tilt up. Negated to feel like grabbing the scene.
        camera.orbit(-static_cast<float>(delta.x()) * kOrbitRadiansPerPixel,
                     -static_cast<float>(delta.y()) * kOrbitRadiansPerPixel);
        if (!delta.isNull()) {
            noteView(ViewPreset::Free); // orbited away from whatever named view this was
        }
    } else if (active & Qt::MiddleButton) {
        camera.pan(static_cast<float>(delta.x()) * kPanPerPixel,
                   static_cast<float>(delta.y()) * kPanPerPixel);
    } else {
        // Plain hover (QWindow gets move events with no buttons held): nothing changed, so don't
        // schedule a frame — an unconditional requestUpdate() here redraws the whole scene at
        // mouse-move rate, exactly the idle churn event-driven rendering exists to avoid.
        return;
    }
    requestUpdate();
}

void VulkanWindow::wheelEvent(QWheelEvent* event) {
    if (!m_renderer) {
        return;
    }
    const float steps = static_cast<float>(event->angleDelta().y()) / 120.0f;
    if (m_axisRotateKey >= 0) {
        // X/Y/Z held: the wheel rotates the selected joint about that channel, a fixed angle
        // per notch (trackpads deliver fractional notches and get proportionally finer steps).
        // Limits clamp inside nudgeSelectedBone; correctives follow live on the GPU, the
        // settled-pose hook runs at the key release.
        constexpr float kDegreesPerWheelNotch = 5.0f;
        glm::vec3 delta(0.0f);
        delta[m_axisRotateKey] = steps * kDegreesPerWheelNotch;
        scene().nudgeSelectedBone(delta);
        requestUpdate();
        return;
    }
    if (m_ik.dragging && (m_activeDragButtons & event->buttons() & Qt::LeftButton)) {
        // DEPTH during a full-body-IK drag. The cursor only ever places the grabbed joint within
        // the camera-parallel drag plane, so without this every "bring the hand forward" needed a
        // release, an orbit, and a second drag. Each notch moves the drag plane along the view
        // direction — scroll up (the dolly-in direction) pushes the joint AWAY from the camera,
        // scroll down pulls it toward it — by a fraction of the plane's distance from the camera,
        // so the step matches the view's scale, and the raw target is re-derived from the
        // CURRENT cursor through the moved plane so the joint stays under the pointer while its
        // depth changes. The 60 Hz timer picks the new target up like any cursor motion (same
        // filter, same governors), so a notch reads as a smooth push rather than a jump, and the
        // Armature's floor clamp on the drag target still applies. The plane is kept at least
        // 10cm in front of the camera: pulled through it, the cursor ray could no longer hit it.
        stepIkDepth(steps, event->position());
        requestUpdate(); // the drag timer issues the moved target on its next tick
        return;
    }
    m_renderer->camera().dolly(steps * kDollyPerWheelStep);
    requestUpdate();
}

void VulkanWindow::keyReleaseEvent(QKeyEvent* event) {
    if (!event->isAutoRepeat() && m_axisRotateKey >= 0 &&
        ((event->key() == Qt::Key_X && m_axisRotateKey == 0) ||
         (event->key() == Qt::Key_Y && m_axisRotateKey == 1) ||
         (event->key() == Qt::Key_Z && m_axisRotateKey == 2))) {
        endAxisRotate();
        event->accept();
        return;
    }
    QWindow::keyReleaseEvent(event);
}

void VulkanWindow::focusOutEvent(QFocusEvent* event) {
    // The releases will never reach us: don't leave the wheel in rotate mode, don't leave a
    // stale FK flag to route the next orbit drag into rotating the joint, and don't leave the
    // button mask set (it would keep blocking the view hotkeys). A live IK drag is left to the
    // stale close-out at the next press: its timer keeps re-issuing the last target, which is
    // harmless, and the mouse-up may still arrive.
    endAxisRotate();
    if (m_posingBone) {
        endFkDrag();
    }
    m_activeDragButtons = Qt::NoButton;
    m_leftClickCandidate = false;
    QWindow::focusOutEvent(event);
}

void VulkanWindow::keyPressEvent(QKeyEvent* event) {
    // Fallback path: the Edit-menu actions carry the app-wide Ctrl+Z/Ctrl+Y shortcuts, but keep
    // handling the raw keys here too in case a platform delivers them to the native window
    // without the shortcut map consuming them first.
    if (m_renderer && event->modifiers().testFlag(Qt::ControlModifier)) {
        if (event->key() == Qt::Key_Z) {
            undo();
            event->accept();
            return;
        }
        if (event->key() == Qt::Key_Y) {
            redo();
            event->accept();
            return;
        }
    }
    // X / Y / Z held with a joint selected: the mouse wheel rotates that joint about the channel
    // for as long as the key is down (see beginAxisRotate); the strip shows the axis badge. The
    // key's auto-repeats are swallowed so a long hold doesn't re-open the edit; no modifiers
    // (Ctrl+Z is undo, and Ctrl+X/Y/Z stay free). Refused while a drag owns the pose; a settle
    // or fall still animating is landed first.
    if (m_renderer && event->modifiers() == Qt::NoModifier &&
        (event->key() == Qt::Key_X || event->key() == Qt::Key_Y || event->key() == Qt::Key_Z)) {
        const int axis = event->key() == Qt::Key_X ? 0 : event->key() == Qt::Key_Y ? 1 : 2;
        if (!event->isAutoRepeat() && !dragInFlight() && m_axisRotateKey != axis &&
            scene().hasSelectedBone()) {
            closeSettlingEdits(); // ends a hold on another axis as its own undo step
            beginAxisRotate(axis);
        }
        event->accept();
        return;
    }
    // Camera views (Blender's numpad convention; the View menu's QAction shortcuts carry these
    // app-wide, this is the in-viewport fallback like Ctrl+Z). Not while a button drag owns the
    // camera or the pose.
    if (m_renderer && !dragInFlight() &&
        !(m_activeDragButtons & (Qt::LeftButton | Qt::MiddleButton)) && handleViewHotkey(event)) {
        event->accept();
        return;
    }
    // Delete — or Backspace, the key labelled Delete on Mac keyboards — removes the SELECTED
    // object, like the context menu's Delete. Handled here (viewport focus) rather than as a
    // window-level shortcut, which would hijack Delete from the Asset Manager's tree and grid.
    // Not mid-gesture: a drag owns the figure until it's released.
    if (m_renderer && (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace) &&
        event->modifiers() == Qt::NoModifier && !dragInFlight()) {
        deleteSelectedObject();
        event->accept();
        return;
    }
    // P toggles the pin on the selected joint (the joint is then held in place through IK drags
    // of other joints). Not mid-gesture: the rig captured its pins at drag start.
    if (m_renderer && event->key() == Qt::Key_P && event->modifiers() == Qt::NoModifier &&
        !dragInFlight() && scene().hasSelectedBone()) {
        togglePinSelectedJoint();
        event->accept();
        return;
    }
    QWindow::keyPressEvent(event);
}

} // namespace pose
