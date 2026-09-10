/**
 * @file viewportwidget.cpp
 * @brief Implementation of the viewport widget facade. See viewportwidget.h.
 */

#include "viewportwidget.h"

#include "scene/shademode.h"
#include "viewportstrip.h"
#include "vulkanwindow.h"

#include <QCoreApplication>
#include <QDebug>
#include <QEvent>
#include <QHideEvent>
#include <QLabel>
#include <QMoveEvent>
#include <QResizeEvent>
#include <QShowEvent>
#include <QVBoxLayout>
#include <QVersionNumber>
#include <QVulkanInstance>

#include <vulkan/vulkan.h>

namespace pose {

namespace {
// Single source of truth for the Vulkan version the app targets. 1.1 is supported by
// effectively every current driver and is enough for VMA's dedicated-allocation path.
// Bump here (and nowhere else) when the renderer starts relying on newer core features.
constexpr uint32_t kVulkanApiVersion = VK_API_VERSION_1_1;
} // namespace

ViewportWidget::ViewportWidget(QWidget* parent) : QWidget(parent) {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    m_instance = std::make_unique<QVulkanInstance>();
    m_instance->setApiVersion(QVersionNumber(1, 1));
#ifndef NDEBUG
    // Standard validation layer in debug builds; QVulkanInstance routes its messages to
    // qDebug() automatically. Released builds skip it for performance.
    m_instance->setLayers({QByteArrayLiteral("VK_LAYER_KHRONOS_validation")});
#endif

    if (!m_instance->create()) {
        auto* message = new QLabel(
            tr("3D viewport unavailable.\n\n"
               "Could not create a Vulkan instance — check that your GPU supports Vulkan "
               "and that your graphics drivers are up to date."),
            this);
        message->setAlignment(Qt::AlignCenter);
        message->setWordWrap(true);
        layout->addWidget(message);
        qCritical() << "[Vulkan] QVulkanInstance::create() failed, error code"
                    << m_instance->errorCode();
        m_instance.reset();
        return;
    }

    // Compiled shaders are mirrored next to the executable by the build (see CMakeLists).
    const QString shaderDir = QCoreApplication::applicationDirPath() + QStringLiteral("/shaders");

    m_window = new VulkanWindow(m_instance.get(), kVulkanApiVersion, shaderDir);
    // Signal-to-signal forward: undo/redo restoring a lighting state surfaces on the facade,
    // where the Environment panel listens.
    connect(m_window, &VulkanWindow::lightingRestored, this, &ViewportWidget::lightingRestored);
    m_container = QWidget::createWindowContainer(m_window, this);
    m_container->setFocusPolicy(Qt::StrongFocus); // so the viewport can receive wheel/keys
    layout->addWidget(m_container);

    createStrip(); // the floating control strip in the top-right corner
}

void ViewportWidget::importObj(const QString& path) {
    withWindow([&](VulkanWindow& w) { w.importObj(path); });
}

void ViewportWidget::importFigure(const QString& path) {
    withWindow([&](VulkanWindow& w) { w.importFigure(path); });
}

bool ViewportWidget::hasPosableFigure() const {
    return m_window && m_window->hasPosableFigure();
}

bool ViewportWidget::savePose(const QString& path) {
    return m_window && m_window->savePose(path);
}

bool ViewportWidget::loadPose(const QString& path) {
    return m_window && m_window->loadPose(path);
}

void ViewportWidget::setShadeMode(int mode) {
    withWindow([mode](VulkanWindow& w) { w.setShadeMode(mode); });
}

void ViewportWidget::deleteSelectedObject() {
    withWindow([](VulkanWindow& w) { w.deleteSelectedObject(); });
}

void ViewportWidget::resetSelectedJoint() {
    withWindow([](VulkanWindow& w) { w.resetSelectedJoint(); });
}

void ViewportWidget::resetSelectedLimb() {
    withWindow([](VulkanWindow& w) { w.resetSelectedLimb(); });
}

void ViewportWidget::resetPose() {
    withWindow([](VulkanWindow& w) { w.resetPose(); });
}

void ViewportWidget::mirrorPose() {
    withWindow([](VulkanWindow& w) { w.mirrorPose(); });
}

void ViewportWidget::mirrorSelectedLimb() {
    withWindow([](VulkanWindow& w) { w.mirrorSelectedLimb(); });
}

void ViewportWidget::setShowSkeleton(bool on) {
    // The window de-duplicates the state itself; only a real change is announced, so the
    // strip ↔ View-menu sync (both driven by the signal) stays one round-trip.
    if (!m_window || m_window->showSkeleton() == on) {
        return;
    }
    m_window->setShowSkeleton(on);
    emit skeletonVisibilityChanged(on);
}

bool ViewportWidget::showSkeleton() const {
    return m_window && m_window->showSkeleton();
}

void ViewportWidget::resetView() {
    withWindow([](VulkanWindow& w) { w.resetView(); });
}

void ViewportWidget::setAxisView(AxisView view) {
    withWindow([view](VulkanWindow& w) { w.setAxisView(view); });
}

void ViewportWidget::flipView() {
    withWindow([](VulkanWindow& w) { w.flipView(); });
}

void ViewportWidget::frameSelected() {
    withWindow([](VulkanWindow& w) { w.frameSelected(); });
}

void ViewportWidget::groundFigure() {
    withWindow([](VulkanWindow& w) { w.groundFigure(); });
}

void ViewportWidget::setEnvironment(const QString& hdrPath) {
    withWindow([&](VulkanWindow& w) { w.setEnvironmentFile(hdrPath); });
}

void ViewportWidget::setLightingSettings(const LightingSettings& settings) {
    withWindow([&](VulkanWindow& w) { w.setLightingSettings(settings); });
}

void ViewportWidget::undo() {
    withWindow([](VulkanWindow& w) { w.undo(); });
}

void ViewportWidget::redo() {
    withWindow([](VulkanWindow& w) { w.redo(); });
}

void ViewportWidget::registerLightingUndo(const LightingSettings& preEdit) {
    withWindow([&](VulkanWindow& w) { w.registerLightingUndo(preEdit); });
}

QStringList ViewportWidget::shaderModeNames() {
    // Picker order = the table's order (scene/shademode.h); the index handed to setShadeMode is
    // a row of that table, and the row maps itself onto mesh.frag's mode + the draw variants.
    QStringList names;
    for (const ShadeMode& mode : kShadeModes) {
        names << QString::fromUtf8(mode.name);
    }
    return names;
}

void ViewportWidget::createStrip() {
    m_strip = new ViewportStrip(this);

    // Strip -> window, always through the facade methods so the menus, the keys and the strip
    // take one path (and the skeleton toggle announces itself once, via the signal below).
    connect(m_strip, &ViewportStrip::shadeModeSelected, this, &ViewportWidget::setShadeMode);
    connect(m_strip, &ViewportStrip::viewSelected, this, [this](ViewPreset preset) {
        switch (preset) {
        case ViewPreset::Top:    setAxisView(AxisView::Top);    break;
        case ViewPreset::Bottom: setAxisView(AxisView::Bottom); break;
        case ViewPreset::Front:  setAxisView(AxisView::Front);  break;
        case ViewPreset::Back:   setAxisView(AxisView::Back);   break;
        case ViewPreset::Left:   setAxisView(AxisView::Left);   break;
        case ViewPreset::Right:  setAxisView(AxisView::Right);  break;
        case ViewPreset::Home:
        case ViewPreset::Free:   resetView();                   break;
        }
    });
    connect(m_strip, &ViewportStrip::homeClicked, this, &ViewportWidget::resetView);
    connect(m_strip, &ViewportStrip::groundClicked, this, &ViewportWidget::groundFigure);
    connect(m_strip, &ViewportStrip::skeletonToggled, this, &ViewportWidget::setShowSkeleton);

    // Window -> strip: the view caption follows the camera (keys, View menu, orbit), the badge
    // follows the X/Y/Z hold, and the Skeleton button follows the overlay state whoever changed
    // it (the signal is the single source of truth).
    connect(m_window, &VulkanWindow::viewPresetChanged, m_strip, &ViewportStrip::setViewPreset);
    connect(m_window, &VulkanWindow::axisRotateKeyChanged, m_strip, &ViewportStrip::setAxisBadge);
    connect(this, &ViewportWidget::skeletonVisibilityChanged, m_strip,
            &ViewportStrip::setSkeletonChecked);
    m_strip->setViewPreset(m_window->currentView());
    m_strip->setSkeletonChecked(m_window->showSkeleton());
}

void ViewportWidget::anchorStrip() {
    if (m_strip && m_container) {
        m_strip->anchorTo(m_container);
    }
}

void ViewportWidget::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    if (!m_strip) {
        return;
    }
    // Follow the top-level window's moves/resizes (a child moveEvent doesn't fire when the whole app
    // window is dragged). Re-target the filter if we've been reparented into a different window.
    if (QWidget* w = window(); w && w != m_filteredWindow) {
        if (m_filteredWindow) {
            m_filteredWindow->removeEventFilter(this);
        }
        w->installEventFilter(this);
        m_filteredWindow = w;
    }
    m_strip->show();
    m_strip->raise();
    anchorStrip();
}

void ViewportWidget::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);
    if (m_strip) {
        m_strip->hide();
    }
}

void ViewportWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    anchorStrip();
}

void ViewportWidget::moveEvent(QMoveEvent* event) {
    QWidget::moveEvent(event);
    anchorStrip();
}

bool ViewportWidget::eventFilter(QObject* watched, QEvent* event) {
    if (watched == m_filteredWindow) {
        const QEvent::Type t = event->type();
        if (t == QEvent::Move || t == QEvent::Resize || t == QEvent::WindowStateChange) {
            anchorStrip();
        } else if (t == QEvent::WindowActivate) {
            // The main window was activated (focus, un-minimize, user click): re-raise the strip
            // so it can't sit BEHIND the main window. On X11 a top-level strip's stacking
            // relative to its owner is the WM's call, so we re-assert it whenever the owner is
            // raised. WindowActivate, not ActivationChange (which is also sent on deactivation):
            // the strip must not be raised over a modal dialog or the splash while they hold the
            // focus.
            if (m_strip && m_strip->isVisible()) {
                m_strip->raise();
            }
        }
    }
    return QWidget::eventFilter(watched, event);
}

ViewportWidget::~ViewportWidget() {
    // The QVulkanInstance (m_instance) owns the VkInstance, and the VulkanWindow's
    // device/renderer were created from it. Qt would otherwise destroy the window via the
    // base QWidget destructor — i.e. AFTER m_instance is gone — leaving the renderer to
    // call vkDestroyDevice on a dead instance. Destroy the container (hence the window,
    // hence all Vulkan objects) here, explicitly, while m_instance is still alive.
    //
    // The container goes BEFORE the strip: the window's teardown (releaseVulkan) ends an open
    // X/Y/Z hold by emitting axisRotateKeyChanged(-1), which lands on the strip's badge — with
    // the strip already deleted that was a use-after-free. The strip is also unhooked first so
    // that emit can't re-anchor it against a container mid-destruction.
    if (m_window && m_strip) {
        disconnect(m_window, nullptr, m_strip, nullptr);
    }
    delete m_container;
    m_container = nullptr;
    m_window = nullptr; // was owned by m_container; now dangling — clear it

    // The strip is an owned top-level window with no Vulkan resources (Qt would also delete it
    // as a child, but explicit keeps every pointer here accounted for).
    delete m_strip;
    m_strip = nullptr;
}

} // namespace pose
