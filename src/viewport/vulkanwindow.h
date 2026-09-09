/**
 * @file vulkanwindow.h
 * @brief The QWindow that actually owns a Vulkan surface and drives rendering.
 *
 * This is the bridge between Qt's windowing/event world and the Qt-free rendering/
 * subtree. It creates the per-window VkSurfaceKHR (via the app's QVulkanInstance),
 * stands up the VulkanContext + VulkanRenderer once the platform surface exists, pumps
 * one frame per UpdateRequest, and routes mouse/wheel input into the camera.
 *
 * It is never parented into the widget tree directly — ViewportWidget wraps it with
 * QWidget::createWindowContainer(). Lifetime of the Vulkan objects is tied to the
 * platform surface: they are torn down on SurfaceAboutToBeDestroyed, before Qt frees
 * the surface out from under them.
 */

#ifndef VULKANWINDOW_H
#define VULKANWINDOW_H

#include "scene/camera.h"           // AxisView (the view hotkeys)
#include "scene/ik/cursorfilter.h"   // IkCursorFilter (the drag target low-pass)
#include "scene/lightingsettings.h" // stored by value; applied to the renderer once it exists
#include "scene/shademode.h"        // kDefaultShadeMode

#include <QWindow>

class QTimer;

#include <vulkan/vulkan.h> // for VkExtent2D in the pixelExtent() signature

#include <glm/glm.hpp>

#include <QElapsedTimer>

#include <algorithm>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

class QVulkanInstance;

namespace pose {

class VulkanContext;
class VulkanRenderer;

/// The named views the viewport's View picker lists and reports: the six axis views, Home (the
/// default perspective framing), and Free — the user orbited away from any named view (shown as
/// "Perspective"). Flip swaps Front/Back and Left/Right; frame-selected and zoom keep the view.
enum class ViewPreset { Free, Top, Bottom, Front, Back, Left, Right, Home };

/**
 * @class VulkanWindow
 * @brief Hosts the Vulkan surface, owns the renderer, and translates input to camera moves.
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
    /// exposed), the path is queued and loaded once initialisation completes.
    void importObj(const QString& path);

    /// Imports a native figure (`.duf`/`.dsf`) into the scene. Same queue-until-ready behaviour as
    /// importObj; routed to the FigureImportService.
    void importFigure(const QString& path);

    /// Whether the scene holds a posable figure (gates the pose save/load actions).
    bool hasPosableFigure() const;
    /// Saves / loads the posed figure's joint rotations to/from @p path. Returns false if there's
    /// no posable figure or the file can't be read/written.
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
    /// lands in ~0.45s), animated on m_fallTimer; a figure sunk below the floor is lifted
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

    // One committed, undoable edit. A single stack holds both kinds so undo walks pose and
    // lighting changes together, in the order the user made them. Joint PIN toggles are pose
    // edits too: the snapshot carries the pins ("@pin:" rows), so undoing a pin or a drag
    // restores the pins of that moment.
    struct UndoEntry {
        enum class Kind { Pose, Lighting };
        Kind kind = Kind::Pose;
        PoseSnapshot pose;         // the pre-edit pose  (kind == Pose)
        int figure = -1;           // which figure the pose belongs to (kind == Pose)
        LightingSettings lighting; // the pre-edit dials (kind == Lighting)
    };

    void initializeVulkan();   // safe to call repeatedly; no-ops once initialised
    /// Decode + bake @p hdrPath off-thread, then swap it in on the GUI thread. @p autoAimKey:
    /// re-aim the key-light dials at the baked environment's dominant light when it lands — true
    /// for user-initiated HDRI switches (the shadow should follow the newly picked environment),
    /// false for the STARTUP bake, so the authored LightingSettings defaults are what a fresh
    /// launch actually shows.
    void beginEnvironmentBake(const QString& hdrPath, bool autoAimKey);
    QString defaultEnvironmentPath() const;            // <appDir>/environment.{hdr,exr} override, else the user library's hdri/
    void releaseVulkan();      // tears down renderer + context (surface still valid)
    void renderFrame();        // one frame, then schedules the next while exposed
    VkExtent2D pixelExtent() const; // window size in physical pixels

    /// Picks the object under @p localPos (window pixels) and, if one is hit, pops up the
    /// object context menu at @p globalPos (currently just "Delete"). No-op on empty space.
    void showObjectContextMenu(const QPointF& localPos, const QPoint& globalPos);

    /// A left CLICK (press + release without dragging) at @p localPos: selects the model under
    /// the cursor — the one the viewport outlines — or clears the selection on empty space.
    void selectModelAtClick(const QPointF& localPos);

    /// Removes model @p index from the scene (the context menu's Delete and deleteSelectedObject
    /// share it): ends a drag or settle in flight first, then clears the undo history.
    void deleteModel(int index);

    /// Records the named view the camera is now in and emits viewPresetChanged if it changed.
    void noteView(ViewPreset view);
    ViewPreset m_viewPreset = ViewPreset::Home; // the camera starts at its default framing

    // Hold-and-scroll joint rotation: while X, Y, or Z is held with a joint selected, the mouse
    // wheel rotates that joint about the matching Euler channel (its own oriented frame, limits
    // enforced) instead of zooming. One hold = one undo entry (the pose is snapshotted at the
    // press, correctives + the commit run at the release); a focus loss ends the hold too.
    void beginAxisRotate(int axis);
    void endAxisRotate();
    int  m_axisRotateKey = -1; // 0/1/2 while X/Y/Z is held, else -1

    QVulkanInstance* m_instance = nullptr; // borrowed
    uint32_t         m_apiVersion = 0;
    QString          m_shaderDir;
    bool             m_initialized = false;
    bool             m_deviceFailed = false; // init threw; don't spam retries

    std::unique_ptr<VulkanContext>  m_context;
    std::unique_ptr<VulkanRenderer> m_renderer;

    std::vector<QString> m_pendingModels;  // OBJ imports requested before the renderer existed
    std::vector<QString> m_pendingFigures; // figure imports requested before the renderer existed
    QString              m_pendingPose;    // pose file to apply once the queued figure is loaded
    int                  m_shadeMode = kDefaultShadeMode; // picker-table index; applied to the renderer once it exists
    bool                 m_showSkeleton = false; // skeleton overlay on/off; applied to the renderer once it exists
    QString              m_environmentPath;         // chosen HDRI (empty = the default at init); applied once the renderer exists
    quint64              m_environmentRequestId = 0; // ++ per HDRI request; a slower earlier bake with a stale id is discarded
    LightingSettings     m_lighting;                // live lighting dials; applied to the renderer once it exists

    QPointF          m_lastMousePos;
    // Buttons whose drag actually began with a press in this window. We gate camera moves on
    // this rather than QMouseEvent::buttons() so a modal dialog (e.g. the Import file picker)
    // can't leak a button-held move to us as it closes and snap the camera.
    Qt::MouseButtons m_activeDragButtons = Qt::NoButton;
    // True when the current CTRL+left-drag began on a figure joint: it FK-rotates that one joint
    // (horizontal = its Y channel, vertical = X) instead of orbiting the camera. Set on press
    // (near a joint, Ctrl held), cleared on release.
    bool             m_posingBone = false;
    // A left press that hit no joint — the orbit gesture — may still turn out to be a CLICK:
    // released within the platform's drag distance of the press, it selects the model under the
    // cursor (box-level pick) or, on empty space, clears the selection. The selected model is
    // the one the viewport outlines.
    bool             m_leftClickCandidate = false;
    QPointF          m_leftPressPos;
    // True while a PLAIN left-drag is full-body-IK-dragging the grabbed joint: the joint follows
    // the cursor in a camera-parallel plane through its grab point (m_ikPlanePoint, world space —
    // the mouse WHEEL moves that plane along the view direction during the drag, the gesture's
    // depth control; see wheelEvent), the body following via the FBIK solve (feet pinned,
    // auto-balanced). This is THE posing
    // gesture; Ctrl+drag is the single-joint FK rotate. The solve is rate-limited per call (it
    // can only move the pose so far per event), so while the button is held m_ikTimer keeps
    // re-issuing the LAST target — catch-up continues and settles even when the mouse stops
    // moving (mouse-move events stop with it).
    bool             m_ikDragging = false;
    // True between IK-drag mouse RELEASE and the end of the ANIMATED release settle: the timer
    // keeps ticking, each tick relaxing the body one capped round onto its ground pins (the old
    // one-shot settle applied the whole landing in a single frame — a visible pose pop at
    // mouse-up). finalizePose + the undo commit are deferred until the settle lands.
    bool             m_ikSettling = false;
    glm::vec3        m_ikPlanePoint{0.0f};
    glm::vec3        m_ikLastTarget{0.0f};
    // The adaptive low-pass the raw cursor target goes through before each solve tick (see
    // cursorfilter.h): raw cursor positions carry pixel noise even when "held still". Shared
    // with the IK harness so the app and the tests run the identical loop.
    IkCursorFilter   m_ikCursorFilter;
    bool             m_ikHasTarget = false;
    // True once a drag's solve has actually CHANGED the pose. A Ctrl+CLICK (select, no motion)
    // must be a no-op: without this gate the release still ran the settle onto the drag's
    // ground-healed pins, visibly shifting a hovering figure and committing an undo entry for
    // a gesture the user perceived as a click.
    bool             m_ikPoseChanged = false;
    QTimer*          m_ikTimer = nullptr;

    /// Issues the current (smoothed) IK target to the renderer — shared by the mouse-move path
    /// and the drag timer, so both apply the same target filtering. Returns true if the solve
    /// actually changed the pose (the caller only requests a frame then).
    bool issueIkTarget();
    /// The IK drag's DEPTH control: moves the drag plane (m_ikPlanePoint) along the view
    /// direction by @p notches wheel notches (+ = away from the camera) and re-derives the raw
    /// drag target from @p cursorPos (window coords) through the moved plane, so the grabbed joint
    /// stays under the pointer while its depth changes. Shared by wheelEvent and the scripted
    /// bench (POSESTUDIO_IK_BENCH=<bone>:depth), which exercises the same geometry with no
    /// desktop input. Returns false if the cursor ray missed the plane (target left unchanged).
    bool stepIkDepth(float notches, const QPointF& cursorPos);
    /// Projects a world point to window coordinates (false if behind the camera).
    bool projectToScreen(const glm::vec3& world, QPointF& out) const;
    /// Completes the post-release IK settle NOW (ends the drag, settles correctives, commits the
    /// undo entry). Called by the timer when the animated settle lands, and by any interaction
    /// that must not overlap it (a new press, undo/redo) to cut it short cleanly.
    void finishIkSettle();
    /// Commits an undo entry for the pose edit bracketed by m_preEditPose (no-op if unchanged).
    void commitPoseUndo();
    /// Runs one reset/mirror utility as an undoable pose edit: refused mid-drag (the rig
    /// captured its pins and pose at drag start), a pending release settle is landed first, a
    /// held X/Y/Z wheel edit is closed as its own undo step first, then the edit is bracketed
    /// with m_preEditPose/commitPoseUndo (a no-op when nothing changed — no selection, already
    /// at rest), correctives re-evaluate, and a frame is requested.
    enum class PoseUtility { ResetJoint, ResetLimb, ResetPose, MirrorPose, MirrorLimb };
    void runPoseUtility(PoseUtility what);
    /// Completes an in-flight ground fall NOW (applies the remaining drop, stops the timer):
    /// any interaction that reads the figure's transform (a press, a pose save) calls this first.
    void finishGroundFall();

    // The animated ground drop (groundFigure): the height still to fall, how much has fallen so
    // far, and the real-time clock the free-fall curve is evaluated against.
    QTimer*       m_fallTimer = nullptr;
    QElapsedTimer m_fallClock;
    float         m_fallHeight = 0.0f;
    float         m_fallDropped = 0.0f;

    // --- IK loop diagnostics (POSESTUDIO_IK_PERF=1) and the scripted IK benchmark
    // (POSESTUDIO_IK_BENCH=<bone>, e.g. lHand): the benchmark runs a Ctrl-drag of that joint
    // along a fixed cursor path with NO desktop input — the real timer, solve, render, and
    // present chain — and prints per-phase tick/frame intervals and the grabbed joint's
    // distance from the (virtual) cursor, then exits. This is how the loop's real rate and the
    // user-perceived lag are measured on the actual machine (the harness assumes 60 Hz ticks).
    struct PerfStat {
        double sum = 0.0;
        double max = 0.0;
        int    n = 0;
        void add(double v) { sum += v; max = std::max(max, v); ++n; }
        double mean() const { return n > 0 ? sum / n : 0.0; }
        void reset() { sum = 0.0; max = 0.0; n = 0; }
    };
    void perfReport(const char* label);
    void startBench();
    void benchAdvance();
    void benchFinish();
    bool          m_ikPerf = false;
    QElapsedTimer m_perfClock;
    qint64        m_perfLastTickNs = -1;
    qint64        m_perfLastFrameNs = -1;
    PerfStat      m_perfTickInterval;
    PerfStat      m_perfTickWork;
    PerfStat      m_perfFrameInterval;
    PerfStat      m_perfDraw;
    PerfStat      m_perfLag;
    // Oscillation of the grabbed joint: the accumulated overlap of consecutive per-tick steps
    // pointing against each other (a smooth track reverses never; trembling reverses every tick).
    double        m_perfOsc = 0.0;
    glm::vec3     m_perfPrevEff{0.0f};
    glm::vec3     m_perfPrevStep{0.0f};
    bool          m_perfHasPrevEff = false;
    int           m_perfTicks = 0;
    QString       m_benchSpec;
    bool          m_benchActive = false;
    int           m_benchTick = 0;
    int           m_benchRetries = 0;
    glm::vec3     m_benchStart{0.0f};
    // Bench ":depth" variant: wheel notches applied through stepIkDepth during the holds (five
    // pushes in hold-1, five pulls in hold-2). The displacement they produce is folded into the
    // scripted path (m_benchDepthOffset) so the path keeps its own shape on top of the depth.
    bool          m_benchDepth = false;
    glm::vec3     m_benchDepthOffset{0.0f};
    float         m_benchDepthExpected = 0.0f; // Σ expected along-view displacement (m)
    float         m_benchDepthActual = 0.0f;   // Σ measured along-view displacement (m)
    void benchDepthNotch(int notch);

    // Undo/redo: each committed edit (a pose drag, or a registered lighting gesture) pushes its
    // pre-edit state. m_preEditPose snapshots the pose at drag start (committed on release).
    PoseSnapshot           m_preEditPose;
    std::vector<UndoEntry> m_undoStack;
    std::vector<UndoEntry> m_redoStack;
};

} // namespace pose

#endif // VULKANWINDOW_H
