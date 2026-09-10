/**
 * @file viewportwidget.h
 * @brief The 3D viewport as a plain QWidget — the ONLY thing the rest of the app touches.
 *
 * Owns the application's QVulkanInstance and embeds a VulkanWindow via
 * QWidget::createWindowContainer(), so callers (e.g. main.cpp) can drop it into a
 * layout like any other widget and stay completely unaware of Vulkan. If the Vulkan
 * instance can't be created (no driver / no GPU), it degrades to an inline message
 * rather than taking the whole app down.
 *
 * It is also the SOLE wiring point between the window and the floating control strip
 * (ViewportStrip): the strip emits what the user picked, the window reports the state it is in,
 * and neither knows the other — every facade method here forwards to the window, and every
 * strip signal lands on one of those facade methods, so the menus, the keys and the strip all
 * take the same path.
 */

#ifndef VIEWPORTWIDGET_H
#define VIEWPORTWIDGET_H

#include "scene/camera.h" // AxisView (the View menu's camera entries)

#include <QStringList>
#include <QWidget>

#include <memory>

class QVulkanInstance;
class QShowEvent;
class QHideEvent;
class QResizeEvent;
class QMoveEvent;

namespace pose {

class ViewportStrip;
class VulkanWindow;
struct LightingSettings;

/**
 * @class ViewportWidget
 * @brief Self-contained 3D viewport widget backed by Vulkan.
 */
class ViewportWidget : public QWidget {
    Q_OBJECT

public:
    explicit ViewportWidget(QWidget* parent = nullptr);
    ~ViewportWidget() override; // out-of-line: completes unique_ptr<QVulkanInstance>

    /// Imports an OBJ file into the 3D scene. No-op if the viewport degraded to the
    /// Vulkan-unavailable message (no window to load into).
    void importObj(const QString& path);

    /// Imports a native figure file (`.duf`/`.dsf`) into the 3D scene. No-op if the viewport
    /// degraded to the Vulkan-unavailable message.
    void importFigure(const QString& path);

    /// Whether the scene holds a posable figure (gates the pose save/load menu actions).
    bool hasPosableFigure() const;
    /// Saves / loads the posed figure's joint rotations to/from @p path. Returns false on failure
    /// (no figure, degraded viewport, or unreadable/unwritable file).
    bool savePose(const QString& path);
    bool loadPose(const QString& path);

    /// Sets the viewport shade mode by index into the picker's table (scene/shademode.h — the
    /// order shaderModeNames() lists). No-op if the viewport degraded.
    void setShadeMode(int mode);

    /// Deletes the selected (outlined) object — Edit → Delete; the viewport's own Delete key
    /// handling reaches the same place. No-op without a selection or if the viewport degraded.
    void deleteSelectedObject();

    /// Pose utilities (Edit menu), all undoable pose edits on the active figure; no-ops without
    /// a figure/selection or if the viewport degraded. "Limb" = the selected joint and
    /// everything below it; Mirror Pose swaps the body's sides, Mirror Limb copies the selected
    /// limb, mirrored, onto the other side.
    void resetSelectedJoint();
    void resetSelectedLimb();
    void resetPose();
    void mirrorPose();
    void mirrorSelectedLimb();

    /// Returns the camera to the default perspective framing (the strip's Home button, the 5
    /// key). No-op if the viewport degraded.
    void resetView();

    /// Camera views in Blender's numpad convention. The View menu's QActions carry the keys
    /// APP-WIDE (1/3/7 + Ctrl for the opposite side, 9 = flip, 5 = Home, "." = frame selected,
    /// number row and keypad alike); the window handles the same keys itself only as the
    /// fallback for a platform that hands the native window the key first. No-ops if the
    /// viewport degraded.
    void setAxisView(AxisView view);
    void flipView();
    void frameSelected();

    /// Drops the posable figure onto the ground plane (the strip's Ground button): moves it so
    /// the current pose's lowest point rests at y = 0. No-op if the viewport degraded.
    void groundFigure();

    /// Toggles the skeleton overlay (the joint→parent bone lines drawn over the figure). Off by
    /// default — joints are grabbed directly on the figure — but joints stay clickable either way.
    /// Driven by the strip's Skeleton button and the View menu (kept in sync via the
    /// skeletonVisibilityChanged signal). No-op if the viewport degraded.
    void setShowSkeleton(bool on);
    bool showSkeleton() const;

    /// Loads @p hdrPath as the lighting environment (re-bakes the IBL). No-op if the viewport degraded.
    void setEnvironment(const QString& hdrPath);
    /// Applies the live lighting/exposure dials (Environment panel). No-op if the viewport degraded.
    void setLightingSettings(const LightingSettings& settings);

    /// Undo/redo the last committed viewport edit — pose changes and Environment-panel lighting
    /// gestures share one chronological stack. Driven by Edit → Undo/Redo (whose shortcuts make
    /// Ctrl+Z/Ctrl+Y work app-wide) and the viewport's own key handling. No-op if degraded.
    void undo();
    void redo();

    /// Registers the pre-edit dial state of a committed Environment-panel gesture on the undo
    /// stack (see VulkanWindow::registerLightingUndo). No-op if the viewport degraded.
    void registerLightingUndo(const LightingSettings& preEdit);

    /// The user-facing shade-mode names in picker order — the names of scene/shademode.h's table,
    /// which is the single source of truth (each row also says how the scene draws in that mode).
    static QStringList shaderModeNames();

signals:
    /// Re-emitted from the viewport when undo/redo restores a lighting state, so the Environment
    /// panel can sync its widgets. The settings are already applied renderer-side.
    void lightingRestored(const LightingSettings& settings);
    /// Emitted whenever the skeleton overlay's visibility changes (from the strip's Skeleton
    /// button or the View menu), so the two controls stay in sync. The renderer state is already
    /// updated by the time this fires.
    void skeletonVisibilityChanged(bool visible);

protected:
    // The control strip is a *top-level* window floating over the native viewport (a child widget
    // would be composited behind it), so it has to be re-anchored as the viewport moves/resizes.
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void moveEvent(QMoveEvent* event) override;
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    /// Builds the strip and wires it to the window — the one place the two meet.
    void createStrip();
    /// Glues the strip to the container's top-right corner (no-op while hidden / degraded).
    void anchorStrip();
    /// Applies @p f to the window — the body of every facade forwarder. No-op when the viewport
    /// degraded (no window).
    template <class F>
    void withWindow(F&& f) {
        if (m_window) {
            f(*m_window);
        }
    }

    std::unique_ptr<QVulkanInstance> m_instance;               // owns the VkInstance
    VulkanWindow*                    m_window = nullptr;        // owned by m_container
    QWidget*                         m_container = nullptr;     // the createWindowContainer wrapper
    ViewportStrip*                   m_strip = nullptr;         // the floating top-right control strip
    QWidget*                         m_filteredWindow = nullptr; // top-level we filter for move/resize
};

} // namespace pose

#endif // VIEWPORTWIDGET_H
