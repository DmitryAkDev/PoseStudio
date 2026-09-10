/**
 * @file vulkanwindow.h
 * @brief The QWindow that owns the Vulkan surface and the renderer, and turns every viewport
 *        gesture — camera, posing, pins, undo — into calls on the Qt-free engine.
 *
 * This is the bridge between Qt's windowing/event world and the Qt-free rendering/ + scene/
 * subtrees: it creates the per-window VkSurfaceKHR (via the app's QVulkanInstance), stands up
 * the VulkanContext + VulkanRenderer once the platform surface exists, pumps one frame per
 * UpdateRequest, and owns every interaction state machine the viewport has. It is never
 * parented into the widget tree directly — ViewportWidget wraps it with
 * QWidget::createWindowContainer(). The Vulkan objects' lifetime is tied to the platform
 * surface: they are torn down on SurfaceAboutToBeDestroyed, before Qt frees the surface.
 *
 * ONE class, SEVEN translation units (the armatureik.cpp pattern): input, IK, pose/undo and the
 * camera all mutate the same private gesture flags and must all call requestUpdate(), so they
 * stay one class with this header as the single contract, split by concern:
 *   - vulkanwindow.cpp             lifecycle: construction, Vulkan init/teardown/device loss,
 *                                  the remembered scene state and the import queue, expose /
 *                                  resize / the event-driven frame.
 *   - vulkanwindow_environment.cpp the HDRI: default path, the off-thread IBL bake, lighting dials.
 *   - vulkanwindow_input.cpp       mouse/wheel/key handlers — pure ROUTING into the gestures below,
 *                                  plus the object context menu and cursor→ray helpers.
 *   - vulkanwindow_camera.cpp      named views, the Blender-convention view hotkeys, projection.
 *   - vulkanwindow_ik.cpp          the full-body-IK drag loop (the 60 Hz tick, the release settle,
 *                                  wheel depth), the animated ground fall, the X/Y/Z wheel hold.
 *   - vulkanwindow_pose.cpp        pose edits as undo entries: commit, utilities, pins, save/load,
 *                                  delete, the unified undo/redo stack.
 *   - vulkanwindow_bench.cpp       the POSESTUDIO_IK_PERF / POSESTUDIO_IK_BENCH diagnostics.
 * The private declarations below follow the same order.
 *
 * Gesture contracts (each learned from a real failure; see CLAUDE.md's viewport section):
 * rendering is EVENT-DRIVEN — every visual mutation calls requestUpdate(); camera drags gate on
 * m_activeDragButtons (buttons whose press was seen HERE), never on QMouseEvent::buttons(); a
 * plain left drag on a joint is the full-body-IK gesture and Ctrl+drag the single-joint FK
 * rotate; the 60 Hz drag timer is the SOLE IK solve driver (mouse moves only record the target);
 * a left press released within the platform drag distance is a CLICK (model select); a release
 * hands off to an ANIMATED settle, a ground drop to an animated fall, and every other pose edit
 * closes those first (closeOpenPoseEdits) while a button-held drag REFUSES them (dragInFlight).
 */

#ifndef VULKANWINDOW_H
#define VULKANWINDOW_H

#include "scene/camera.h"           // AxisView (the view hotkeys), Ray
#include "scene/ik/cursorfilter.h"  // IkCursorFilter (the drag target low-pass)
#include "scene/lightingsettings.h" // stored by value; applied to the renderer once it exists
#include "scene/shademode.h"        // kDefaultShadeMode
#include "viewpreset.h"

#include <QElapsedTimer>
#include <QPointF>
#include <QWindow>

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class QSize;
class QTimer;
class QVulkanInstance;

namespace pose {

class Scene;
class VulkanContext;
class VulkanError;
class VulkanRenderer;
struct IkDiagnostics;

/**
 * @class VulkanWindow
 * @brief Hosts the Vulkan surface, owns the renderer, and runs every viewport interaction.
 */
class VulkanWindow : public QWindow {
    Q_OBJECT

public:
    /**
     * @param instance   The application-wide QVulkanInstance (owned by ViewportWidget).
     * @param apiVersion The Vulkan API version @p instance was created with (VK_API_VERSION_*).
     * @param shaderDir  Absolute path to the directory holding compiled *.spv files.
     */
    VulkanWindow(QVulkanInstance* instance, uint32_t apiVersion, QString shaderDir,
                 QWindow* parent = nullptr);
    ~VulkanWindow() override;

    /// Imports an OBJ into the scene. If the renderer isn't built yet (the window hasn't been
    /// exposed), the path is queued and loaded once initialisation completes. An interactive
    /// import closes every open pose edit first (an import selects the new model, and a settle
    /// still ticking would retarget to it).
    void importObj(const QString& path);

    /// Imports a native figure (`.duf`/`.dsf`) into the scene. Same queue-until-ready behaviour as
    /// importObj; routed to the FigureImportService.
    void importFigure(const QString& path);

    /// Whether the scene holds a posable figure (gates the pose save/load actions).
    bool hasPosableFigure() const;
    /// Saves / loads the posed figure's joint rotations to/from @p path. Returns false if there's
    /// no posable figure or the file can't be read/written. A load is an undoable pose edit
    /// (refused while a button-held drag owns the pose); before the renderer exists the path is
    /// queued and applied after the queued figure (open-with a .pose).
    bool savePose(const QString& path);
    bool loadPose(const QString& path);

    /// Sets the viewport shade mode (an index into the picker's table, scene/shademode.h).
    /// Remembered and applied once the renderer exists if it isn't built yet.
    void setShadeMode(int mode);

    /// Deletes the SELECTED object (the outlined one) — the Delete key and Edit → Delete. No-op
    /// without a selection or before the renderer exists; a drag in flight is ended first, and
    /// the undo history is cleared (model indices shift; the deleted figure's poses are moot).
    void deleteSelectedObject();

    /// Pose utilities (Edit menu / the joint context menu), each ONE undoable pose edit on the
    /// active figure — see runPoseUtility. "Limb" = the selected joint and everything below it.
    void resetSelectedJoint();
    void resetSelectedLimb();
    void resetPose();
    void mirrorPose();
    void mirrorSelectedLimb();

    /// Toggles the skeleton overlay (the joint→parent bone lines drawn over the figure). Off by
    /// default — joints are grabbed directly on the figure — but they stay grabbable either way.
    /// Remembered and applied once the renderer exists if it isn't built yet.
    void setShowSkeleton(bool on);
    bool showSkeleton() const;

    /// Restores the camera's default framing (the viewport's Home button). No-op before the
    /// renderer exists — the camera is created with that framing, so there'd be nothing to undo.
    void resetView();

    /// Camera hotkeys in Blender's numpad convention (View menu + the keys in keyPressEvent):
    /// snap to an axis-aligned view, flip to the opposite side (180° about the up axis), or
    /// frame the selected object (everything, with no selection). No-ops before the renderer
    /// exists.
    void setAxisView(AxisView view);
    void flipView();
    void frameSelected();

    /// Which named view the camera is in (see ViewPreset) — the View picker's label.
    ViewPreset currentView() const { return m_viewPreset; }

    /// Drops the posable figure onto the ground plane (the overlay's ground button): translates it
    /// so the current pose's lowest point rests at y = 0. A figure ABOVE the floor FALLS there —
    /// free fall from rest at 9.81 m/s² on real elapsed time (world units are metres, so a 1m drop
    /// lands in ~0.45s), animated on the fall timer; a figure sunk below the floor is lifted
    /// instantly (nothing falls upward). No-op before the renderer exists or mid-drag.
    void groundFigure();

    /// Loads @p hdrPath as the lighting environment and re-bakes the IBL. Remembered and applied
    /// once the renderer exists if it isn't built yet. Decoding happens in this Qt layer.
    void setEnvironmentFile(const QString& hdrPath);

    /// Applies the live lighting/exposure dials (Environment panel). Remembered and applied once the
    /// renderer exists if it isn't built yet. Never registers undo itself — committed gestures do,
    /// via registerLightingUndo (live scrubbing pushes many intermediate states through here).
    void setLightingSettings(const LightingSettings& settings);

    /// Undoes / redoes the most recent committed edit. Pose changes and Environment-panel lighting
    /// edits share ONE chronological stack, so Ctrl+Z steps back through both kinds in the order
    /// they were made. Reachable via Edit → Undo/Redo (any focus) and Ctrl+Z/Ctrl+Y in the
    /// viewport; no-op while that stack side is empty or the renderer doesn't exist yet.
    void undo();
    void redo();

    /// Pushes a lighting undo entry: @p preEdit is the dial state before a committed Environment-
    /// panel gesture (scrub, type-in, restore button, restore-all). The panel calls this once per
    /// gesture; undoing the entry emits lightingRestored so the panel's widgets follow.
    void registerLightingUndo(const LightingSettings& preEdit);

signals:
    /// Emitted when undo/redo restores a lighting state. The settings are already applied to the
    /// renderer; the Environment panel listens (via ViewportWidget) to sync its widgets.
    void lightingRestored(const LightingSettings& settings);
    /// Emitted whenever the camera enters or leaves a named view (an axis view, Home, a flip, or
    /// an orbit drag away from one) — the View picker's button follows it.
    void viewPresetChanged(ViewPreset view);
    /// Emitted when an axis-rotate key hold begins (@p axis 0/1/2 = X/Y/Z: while held, the mouse
    /// wheel rotates the selected joint about that channel) and when it ends (-1). The viewport
    /// strip shows an axis badge for the duration.
    void axisRotateKeyChanged(int axis);

protected:
    void exposeEvent(QExposeEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    bool event(QEvent* event) override;

    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;
    void focusOutEvent(QFocusEvent* event) override;

private:
    using PoseSnapshot = std::vector<std::pair<std::string, glm::vec3>>;

    // =========================================================================================
    // Lifecycle (vulkanwindow.cpp)
    // =========================================================================================

    /// Builds the context + renderer on the platform surface; safe to call repeatedly (no-ops
    /// once initialised or after a device failure). Applies the remembered scene state, kicks
    /// off the startup environment bake, and drains the import queue.
    void initializeVulkan();
    /// Tears down renderer + context (the surface stays valid) and resets every interaction
    /// state machine — a drag, settle, fall, or axis hold dies with the figure it acted on.
    /// Idempotent.
    void releaseVulkan();
    /// The ONE device-loss path (init, the environment upload, a frame): releases the Vulkan
    /// objects, marks the device failed so nothing retries every expose, and logs @p stage.
    void failDevice(const char* stage, const VulkanError& error);
    /// One frame, then schedules another only when the renderer says the swapchain was rebuilt
    /// (event-driven rendering — see renderFrame's comment).
    void renderFrame();
    /// Window size in physical pixels (the swapchain extent).
    QSize pixelExtent() const;
    /// Hands the renderer the current physical size + device pixel ratio and requests a frame
    /// (expose and resize both do exactly this).
    void syncRendererExtent();
    /// The renderer's scene — every posing/selection call goes through it. Requires m_renderer.
    Scene& scene() const;

    /// An import requested before the renderer existed, replayed in request order once it does
    /// (command-line "open with" hands several in a row — the order must survive).
    struct PendingImport {
        enum class Kind { Obj, Figure };
        Kind    kind = Kind::Obj;
        QString path;
    };
    /// The one import body: the interactive path (progress dialog) and the startup drain (no
    /// dialog — it runs inside the expose/init path, where a modal dialog's processEvents() could
    /// re-enter exposeEvent()/initializeVulkan()) share it. True if a model was added.
    bool runImport(const PendingImport& import, bool showProgress);
    /// Queues or runs an import depending on whether the renderer exists yet.
    void importOrQueue(PendingImport import);

    /// Scene state this window remembers on the renderer's behalf: chosen before the renderer
    /// exists (the picker/panel can be driven while the viewport is still hidden) and re-applied
    /// after a device loss. applyTo() pushes the live settings; the queues are drained by
    /// initializeVulkan().
    struct DeferredSceneState {
        int              shadeMode = kDefaultShadeMode; // picker-table index
        bool             showSkeleton = false;
        LightingSettings lighting;        // the live dials (also what undo restores from)
        QString          environmentPath; // chosen HDRI (empty = the default at init)
        std::vector<PendingImport> imports; // in request order
        QString          pendingPose;     // pose file to apply once the queued figure is loaded
        /// Applies shade mode, skeleton overlay and lighting dials to a (fresh) renderer.
        void applyTo(VulkanRenderer& renderer) const;
    };

    /// Applies @p f to the renderer and requests a frame — the body of every "remember, apply,
    /// redraw" setter. No-op before the renderer exists (the remembered value applies at init).
    template <class F>
    void withRenderer(F&& f) {
        if (m_renderer) {
            f(*m_renderer);
            requestUpdate();
        }
    }

    QVulkanInstance* m_instance = nullptr; // borrowed
    uint32_t         m_apiVersion = 0;
    QString          m_shaderDir;
    bool             m_initialized = false;
    bool             m_deviceFailed = false; // init/frame/upload threw; don't spam retries

    std::unique_ptr<VulkanContext>  m_context;
    std::unique_ptr<VulkanRenderer> m_renderer;
    DeferredSceneState              m_deferred;

    // =========================================================================================
    // Environment (vulkanwindow_environment.cpp)
    // =========================================================================================

    /// Decode + bake @p hdrPath off-thread, then swap it in on the GUI thread. @p autoAimKey:
    /// re-aim the key-light dials at the baked environment's dominant light when it lands — true
    /// for user-initiated HDRI switches (the shadow should follow the newly picked environment),
    /// false for the STARTUP bake, so the authored LightingSettings defaults are what a fresh
    /// launch actually shows.
    void beginEnvironmentBake(const QString& hdrPath, bool autoAimKey);
    /// The panorama to load when none was chosen (or the chosen one no longer exists):
    /// LibraryPaths::defaultHdri(). Empty = keep the procedural studio.
    QString defaultEnvironmentPath() const;

    quint64 m_environmentRequestId = 0; // ++ per HDRI request; a slower earlier bake with a stale id is discarded

    // =========================================================================================
    // Input (vulkanwindow_input.cpp) — the handlers ROUTE; the gestures live in the sections
    // below.
    // =========================================================================================

    /// The world-space picking ray through @p localPos (window pixels).
    Ray cursorRay(const QPointF& localPos) const;
    /// Selects and returns the figure joint under @p localPos (Scene::selectBoneAt — picks by
    /// bone body, independent of the overlay's visibility), or -1 on a miss (the selection is
    /// kept).
    int boneAt(const QPointF& localPos);
    /// Picks the object under @p localPos (window pixels) and, if a joint or model is hit, pops
    /// up the object context menu at @p globalPos: Pin/Unpin Joint (+ Unpin All), Reset Joint /
    /// Reset Limb / Mirror Limb, and Delete. No-op on empty space.
    void showObjectContextMenu(const QPointF& localPos, const QPoint& globalPos);
    /// A left CLICK (press + release without dragging) at @p localPos: selects the model under
    /// the cursor — the one the viewport outlines — or clears the selection on empty space.
    void selectModelAtClick(const QPointF& localPos);

    QPointF m_lastMousePos;
    // Buttons whose drag actually began with a press in this window. We gate camera moves on
    // this rather than QMouseEvent::buttons() so a modal dialog (e.g. the Import file picker)
    // can't leak a button-held move to us as it closes and snap the camera.
    Qt::MouseButtons m_activeDragButtons = Qt::NoButton;
    // True when the current CTRL+left-drag began on a figure joint: it FK-rotates that one joint
    // (horizontal = its Y channel, vertical = X) instead of orbiting the camera. Set on press
    // (near a joint, Ctrl held), cleared on release (endFkDrag).
    bool m_posingBone = false;
    // A left press that hit no joint — the orbit gesture — may still turn out to be a CLICK:
    // released within the platform's drag distance of the press, it selects the model under the
    // cursor (box-level pick) or, on empty space, clears the selection. The selected model is
    // the one the viewport outlines.
    bool    m_leftClickCandidate = false;
    QPointF m_leftPressPos;

    // =========================================================================================
    // Camera (vulkanwindow_camera.cpp)
    // =========================================================================================

    /// Records the named view the camera is now in and emits viewPresetChanged if it changed.
    void noteView(ViewPreset view);
    /// The in-viewport fallback for the View menu's app-wide shortcuts (Blender's numpad
    /// convention, number row + keypad with either NumLock state): true if @p event was a view
    /// key and was handled.
    bool handleViewHotkey(const QKeyEvent* event);
    /// Projects a world point to window coordinates (false if behind the camera).
    bool projectToScreen(const glm::vec3& world, QPointF& out) const;

    ViewPreset m_viewPreset = ViewPreset::Home; // the camera starts at its default framing

    // =========================================================================================
    // Full-body IK drag, ground fall, axis hold (vulkanwindow_ik.cpp)
    // =========================================================================================

    /// Wheel during a full-body-IK drag: how far the drag plane moves along the view direction
    /// per 120-unit notch, as a FRACTION of the plane's distance from the camera (~5cm framing a
    /// whole figure at the default 2.6m, millimetres in a close-up on a hand). Shared with the
    /// benchmark's ":depth" variant, which checks the geometry against it.
    static constexpr float kIkDepthPerWheelNotch = 0.02f;

    /// The 60 Hz drag timer's tick — the SOLE IK solve driver: re-issues the last (filtered)
    /// target while the button is held, so catch-up continues and settles even when the mouse
    /// stops moving (mouse-move events stop with it), and steps the animated release settle.
    void onIkTick();
    /// Issues the current (smoothed) IK target to the scene. Returns true if the solve actually
    /// changed the pose (the caller only requests a frame then). ONLY onIkTick calls this —
    /// solving per mouse event made the damped dynamics mouse-polling-rate-dependent.
    bool issueIkTarget();
    /// A left press on the joint under @p localPos: selects it and begins the gesture — a plain
    /// press the full-body-IK drag, @p fkModifier (Ctrl) the single-joint FK rotate — and
    /// snapshots the pose for undo. Returns false when no joint is under the cursor (the press
    /// is then the orbit/click gesture).
    bool beginJointGesture(const QPointF& localPos, bool fkModifier);
    /// Begins the FBIK drag of the selected joint (the drag plane anchored at its position) and
    /// starts the tick. Returns false without a figure/selection.
    bool beginIkDrag();
    /// Mouse-up of an IK drag that moved the pose: hands off to the ANIMATED settle — the timer
    /// keeps ticking, each tick relaxing the body onto its pins (finishIkSettle then commits).
    void beginIkRelease();
    /// Ends an IK drag WITHOUT a settle: a click on a joint (nothing moved), a stale drag whose
    /// release never reached this window, a delete mid-gesture. Commits the undo entry if the
    /// drag did move the pose.
    void abortIkDrag();
    /// Ends a Ctrl FK drag: the settled-pose hook runs and the undo entry is committed.
    void endFkDrag();
    /// Completes the post-release IK settle NOW (ends the drag, runs the settled-pose hook,
    /// commits the undo entry). Called by the timer when the animated settle lands, and by any
    /// interaction that must not overlap it (a new press, undo/redo) to cut it short cleanly.
    void finishIkSettle();
    /// Intersects the cursor ray through @p localPos with the camera-parallel drag plane through
    /// m_ik.planePoint. True (with @p outWorld) on a hit in front of the camera.
    bool dragPlaneHit(const QPointF& localPos, glm::vec3& outWorld) const;
    /// Records @p target as the raw drag target (seeding the cursor filter at the grab point on
    /// the first one). The tick consumes it.
    void setIkTarget(const glm::vec3& target);
    /// The IK drag's DEPTH control: moves the drag plane (m_ik.planePoint) along the view
    /// direction by @p notches wheel notches (+ = away from the camera) and re-derives the raw
    /// drag target from @p cursorPos (window coords) through the moved plane, so the grabbed joint
    /// stays under the pointer while its depth changes. Shared by wheelEvent and the scripted
    /// bench (POSESTUDIO_IK_BENCH=<bone>:depth), which exercises the same geometry with no
    /// desktop input. Returns false if the cursor ray missed the plane (target left unchanged).
    bool stepIkDepth(float notches, const QPointF& cursorPos);

    /// The full-body-IK drag's state (see the class comment for the gesture).
    struct IkDragState {
        // True while a PLAIN left-drag is full-body-IK-dragging the grabbed joint: the joint
        // follows the cursor in a camera-parallel plane through its grab point (planePoint,
        // world space — the mouse WHEEL moves that plane along the view direction during the
        // drag, the gesture's depth control), the body following via the FBIK solve (feet
        // pinned, auto-balanced).
        bool dragging = false;
        // True between IK-drag mouse RELEASE and the end of the ANIMATED release settle: the
        // timer keeps ticking, each tick relaxing the body one capped round onto its ground pins
        // (the old one-shot settle applied the whole landing in a single frame — a visible pose
        // pop at mouse-up). The settled-pose hook + the undo commit wait until it lands.
        bool settling = false;
        // A raw target has been recorded since the press (the first move or wheel seeds the
        // filter at the grab point).
        bool hasTarget = false;
        // True once a drag's solve has actually CHANGED the pose. A CLICK on a joint (select, no
        // motion) must be a no-op: without this gate the release still ran the settle onto the
        // drag's ground-healed pins, visibly shifting a hovering figure and committing an undo
        // entry for a gesture the user perceived as a click.
        bool      poseChanged = false;
        glm::vec3 planePoint{0.0f};
        glm::vec3 lastTarget{0.0f};
        // The adaptive low-pass the raw cursor target goes through before each solve tick (see
        // cursorfilter.h): raw cursor positions carry pixel noise even when "held still". Shared
        // with the IK harness so the app and the tests run the identical loop.
        IkCursorFilter filter;
    };
    IkDragState m_ik;
    QTimer*     m_ikTimer = nullptr;

    /// Completes an in-flight ground fall NOW (applies the remaining drop, stops the timer):
    /// any interaction that reads the figure's transform (a press, a pose save) calls this first.
    void finishGroundFall();
    /// The fall timer's tick: advances the free-fall curve on real elapsed time.
    void onFallTick();

    /// The animated ground drop (groundFigure): WHICH model falls (the active figure at the
    /// button press — undo can switch the active figure mid-fall, and the remaining drop must
    /// still land on the one that started falling), the height still to fall, how much has
    /// fallen so far, and the real-time clock the free-fall curve is evaluated against.
    struct GroundFall {
        QTimer*       timer = nullptr;
        QElapsedTimer clock;
        float         height = 0.0f;
        float         dropped = 0.0f;
        int           figure = -1;
    };
    GroundFall m_fall;

    // Hold-and-scroll joint rotation: while X, Y, or Z is held with a joint selected, the mouse
    // wheel rotates that joint about the matching Euler channel (its own oriented frame, limits
    // enforced) instead of zooming. One hold = one undo entry (the pose is snapshotted at the
    // press, the settled-pose hook + the commit run at the release); a focus loss, a mouse
    // press, undo/redo, or any other pose edit ends the hold too.
    void beginAxisRotate(int axis);
    void endAxisRotate();
    int  m_axisRotateKey = -1; // 0/1/2 while X/Y/Z is held, else -1

    // =========================================================================================
    // Pose edits and undo (vulkanwindow_pose.cpp)
    // =========================================================================================

    /// A mouse-button-held pose gesture owns the pose right now (an IK drag, or a Ctrl FK drag):
    /// every other pose edit — undo/redo, utilities, pins, a pose load, the view/delete keys —
    /// is REFUSED until its release. An IK drag in particular solves against pins captured at
    /// drag start, and re-posing underneath it would leave the solve fighting a stale stance.
    bool dragInFlight() const { return m_ik.dragging || m_posingBone; }
    /// Any pose edit in flight: a held drag, the animated release settle, or an X/Y/Z wheel hold.
    /// The self-completing ones can be closed early by closeOpenPoseEdits().
    bool poseEditInFlight() const {
        return dragInFlight() || m_ik.settling || m_axisRotateKey >= 0;
    }
    /// Lands / closes every edit that completes on its own — the release settle, the ground
    /// fall, an X/Y/Z hold — so the caller acts on a settled pose (each one commits its own undo
    /// entry). Leaves a live button-held drag alone.
    void closeSettlingEdits();
    /// closeSettlingEdits() plus the button-held drags: a STALE IK drag (its release never
    /// reached this window) is aborted, an FK drag ended. Every discrete pose edit (undo/redo,
    /// utilities, pins, pose load/save, delete, import) starts here so m_preEditPose can't be
    /// overwritten under an open bracket — the cross-figure/mis-bracketed undo entries of old.
    void closeOpenPoseEdits();

    /// Commits an undo entry for the pose edit bracketed by m_preEditPose (no-op if unchanged).
    void commitPoseUndo();
    /// Runs one reset/mirror utility as an undoable pose edit: refused while a drag owns the
    /// pose (the rig captured its pins and pose at drag start), every open edit is closed
    /// first, then the edit is bracketed with m_preEditPose/commitPoseUndo (a no-op when
    /// nothing changed — no selection, already at rest), correctives re-evaluate, and a frame is
    /// requested.
    enum class PoseUtility { ResetJoint, ResetLimb, ResetPose, MirrorPose, MirrorLimb };
    void runPoseUtility(PoseUtility what);
    /// Pin / unpin the selected joint, or drop every pin — each ONE undoable pose edit (pins
    /// ride in the pose snapshot as "@pin:" rows). Refused while a drag owns the pose.
    void togglePinSelectedJoint();
    void unpinAllJoints();
    /// Removes model @p index from the scene (the context menu's Delete and deleteSelectedObject
    /// share it): every open edit is closed first, then the undo history is cleared.
    void deleteModel(int index);

    // One committed, undoable edit. A single stack holds both kinds so undo walks pose and
    // lighting changes together, in the order the user made them. Joint PIN toggles are pose
    // edits too: the snapshot carries the pins ("@pin:" rows), so undoing a pin or a drag
    // restores the pins of that moment.
    struct UndoEntry {
        enum class Kind { Pose, Lighting };
        Kind             kind = Kind::Pose;
        PoseSnapshot     pose;        // the pre-edit pose  (kind == Pose)
        int              figure = -1; // which figure the pose belongs to (kind == Pose)
        LightingSettings lighting;    // the pre-edit dials (kind == Lighting)
    };
    /// The one undo/redo body: pops @p from, re-applies it, and pushes the state it replaced
    /// onto @p to. undo() and redo() are mirror images of each other through it.
    void swapUndoEntry(std::vector<UndoEntry>& from, std::vector<UndoEntry>& to);

    // Undo/redo: each committed edit (a pose drag, or a registered lighting gesture) pushes its
    // pre-edit state. m_preEditPose snapshots the pose at the start of every bracketed edit.
    PoseSnapshot           m_preEditPose;
    std::vector<UndoEntry> m_undoStack;
    std::vector<UndoEntry> m_redoStack;

    // =========================================================================================
    // Diagnostics (vulkanwindow_bench.cpp; see ikdiagnostics.h)
    // =========================================================================================

    // IK loop diagnostics (POSESTUDIO_IK_PERF=1) and the scripted IK benchmark
    // (POSESTUDIO_IK_BENCH=<bone>[:depth], e.g. lHand): the benchmark runs a plain (full-body-IK)
    // drag of that joint along a fixed cursor path with NO desktop input — the real timer, solve,
    // render, and present chain — and prints per-phase tick/frame intervals and the grabbed
    // joint's distance from the (virtual) cursor, then exits. This is how the loop's real rate
    // and the user-perceived lag are measured on the actual machine (the harness assumes 60 Hz
    // ticks). m_diag is null unless one of the env vars is set — the hot paths test only it.
    void startBench();
    void benchAdvance();
    void benchDepthNotch(int notch);
    void benchFinish();
    std::unique_ptr<IkDiagnostics> m_diag;
};

} // namespace pose

#endif // VULKANWINDOW_H
