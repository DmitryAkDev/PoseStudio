/**
 * @file viewportwidget.h
 * @brief The 3D viewport as a plain QWidget — the ONLY thing the rest of the app touches.
 *
 * Owns the application's QVulkanInstance and embeds a VulkanWindow via
 * QWidget::createWindowContainer(), so callers (e.g. main.cpp) can drop it into a
 * layout like any other widget and stay completely unaware of Vulkan. If the Vulkan
 * instance can't be created (no driver / no GPU), it degrades to an inline message
 * rather than taking the whole app down.
 */

#ifndef VIEWPORTWIDGET_H
#define VIEWPORTWIDGET_H

#include "scene/camera.h" // AxisView (the View menu's camera entries)

#include <QWidget>
#include <QStringList>

#include <memory>

class QVulkanInstance;
class QPushButton;
class QShowEvent;
class QHideEvent;
class QResizeEvent;
class QMoveEvent;

namespace pose {

class AxisRotateBadge;
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

    /// Returns the camera to the default perspective framing (the overlay's Home button).
    /// No-op if the viewport degraded.
    void resetView();

    /// Camera views in Blender's numpad convention — View menu entries; the viewport handles the
    /// keys itself (1/3/7 + Ctrl, 9, "."). No-ops if the viewport degraded.
    void setAxisView(AxisView view);
    void flipView();
    void frameSelected();

    /// Drops the posable figure onto the ground plane (the overlay's ground button): moves it so
    /// the current pose's lowest point rests at y = 0. No-op if the viewport degraded.
    void groundFigure();

    /// Toggles the skeleton overlay (the joint→parent bone lines drawn over the figure). Off by
    /// default — joints are grabbed directly on the figure — but joints stay clickable either way.
    /// Driven by the overlay's Skeleton button and the View menu (kept in sync via the
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
    /// Emitted whenever the skeleton overlay's visibility changes (from the overlay's Skeleton
    /// button or the View menu), so the two controls stay in sync. The renderer state is already
    /// updated by the time this fires.
    void skeletonVisibilityChanged(bool visible);

protected:
    // The shader dropdown floats as a *top-level* window over the native viewport (a child widget would
    // be composited behind it), so it has to be repositioned as the viewport moves/resizes.
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void moveEvent(QMoveEvent* event) override;
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void createShaderOverlay();   // builds the floating top-right strip: shader + view pickers, buttons
    void syncOverlayPosition();   // glues it to the viewport's top-right corner (global coords)

    std::unique_ptr<QVulkanInstance> m_instance;           // owns the VkInstance
    VulkanWindow*                    m_window = nullptr;    // owned by m_container
    QWidget*                         m_container = nullptr; // the createWindowContainer wrapper
    QWidget*                         m_overlay = nullptr;   // top-level frameless host for the dropdown
    QPushButton*                     m_shaderButton = nullptr;   // opens the shader-mode QMenu
    QPushButton*                     m_viewButton = nullptr;     // opens the named-view QMenu; reads the current view
    QPushButton*                     m_homeButton = nullptr;     // resets the camera to default framing
    QPushButton*                     m_groundButton = nullptr;   // drops the figure onto the floor plane
    QPushButton*                     m_skeletonButton = nullptr; // toggles the skeleton overlay (View)
    AxisRotateBadge*                 m_axisBadge = nullptr;      // "rotating about X/Y/Z" badge under the strip
    QWidget*                         m_filteredWindow = nullptr; // top-level we filter for move/resize
};

} // namespace pose

#endif // VIEWPORTWIDGET_H
